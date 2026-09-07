#!/usr/bin/env python3
"""ESCape32 Programmer - serial bootloader and ESCape32 Link utility.

Supports:
- COM/serial port discovery
- ESCape32 bootloader probe/info
- application firmware programming and read-back verification
- bootloader update and write-protection control
- ESP32-C3 ESCape32 Link adapter control (Wi-Fi state / reboot / ownership)
"""

from __future__ import annotations

import argparse
import hashlib
import struct
import sys
import time
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


TOOL_VERSION = "1.2.0"

CMD_PROBE = 0
CMD_INFO = 1
CMD_READ = 2
CMD_WRITE = 3
CMD_UPDATE = 4
CMD_SETWRP = 5

RES_OK = 0
APP_SIGNATURE = 0x32EA

BAUDRATE = 38400
READ_TIMEOUT_S = 0.300
DEFAULT_PROBE_TIMEOUT_S = 5.0
TX_CHUNK_SIZE = 32
TX_BYTE_TIME_S = 260e-6
MAX_APP_IMAGE_SIZE = 128 * 1024
MAX_BOOT_IMAGE_SIZE = 4 * 1024
BLOCK_SIZE = 1024
WRP_LEVELS = {0: 0x33, 1: 0x44, 2: 0x55}

ADAPTER_REQUEST_MAGIC = b"\xA5\x5AESC32!"
ADAPTER_RESPONSE_MAGIC = b"\x5A\xA5ESC32!"
ADAPTER_PROTOCOL_VERSION = 1

ADAPTER_CMD_INFO = 1
ADAPTER_CMD_WIFI_GET = 2
ADAPTER_CMD_WIFI_SET = 3
ADAPTER_CMD_REBOOT = 4
ADAPTER_CMD_ACQUIRE_ESC = 5
ADAPTER_CMD_RELEASE_ESC = 6

ADAPTER_STATUS = {
    0: "OK",
    1: "BUSY",
    2: "BAD_ARG",
    3: "NVS_ERROR",
    4: "INTERNAL",
}

ADAPTER_INFO_STRUCT = struct.Struct("<BBBBBBBBIBB")


class ProgrammerError(RuntimeError):
    """Expected communication, protocol, image, or validation failure."""


@dataclass
class BootInfo:
    revision: int
    io_pin: int
    idcode: int

    @property
    def dev_id(self) -> int:
        return self.idcode & 0x0FFF

    @property
    def silicon_revision(self) -> int:
        return (self.idcode >> 16) & 0xFFFF


@dataclass
class FirmwareInfo:
    installed: bool
    signature: int
    revision: Optional[int] = None
    patch: Optional[int] = None
    name: str = ""

    @property
    def version_string(self) -> str:
        if not self.installed or self.revision is None:
            return "not installed"
        if self.patch is None:
            return f"rev{self.revision}"
        return f"rev{self.revision}.{self.patch}"


@dataclass
class ImageInfo:
    path: Path
    raw_size: int
    padded: bytes
    firmware: FirmwareInfo
    sha256: str

    @property
    def padded_size(self) -> int:
        return len(self.padded)

    @property
    def block_count(self) -> int:
        return (len(self.padded) + BLOCK_SIZE - 1) // BLOCK_SIZE


@dataclass
class AdapterInfo:
    status: int
    fw_major: int
    fw_minor: int
    fw_patch: int
    wifi_configured: bool
    wifi_active: bool
    reboot_required: bool
    owner: int
    uart_baud: int
    uart_rx: int
    uart_tx: int

    @property
    def version_string(self) -> str:
        return f"{self.fw_major}.{self.fw_minor}.{self.fw_patch}"

    @property
    def owner_name(self) -> str:
        return {0: "NONE", 1: "USB", 2: "WIFI"}.get(
            self.owner, f"UNKNOWN({self.owner})"
        )


def require_pyserial() -> None:
    if serial is None:
        raise ProgrammerError(
            "pyserial is not installed. Install it with:\n"
            "  py -m pip install pyserial"
        )


def list_serial_ports() -> int:
    require_pyserial()
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 1
    print("Available serial ports:")
    for p in ports:
        vid_pid = ""
        if p.vid is not None and p.pid is not None:
            vid_pid = f" VID:PID={p.vid:04X}:{p.pid:04X}"
        print(f"  {p.device:8s}  {p.description}{vid_pid}")
    return 0


def crc32_escape32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def encode_value(value: int) -> bytes:
    value &= 0xFF
    return bytes((value, (~value) & 0xFF))


def decode_value(raw: bytes) -> int:
    if len(raw) != 2:
        raise ProgrammerError(
            f"Expected 2-byte value, received {len(raw)} byte(s)"
        )
    if (raw[0] ^ raw[1]) != 0xFF:
        raise ProgrammerError(f"Invalid complemented value: {raw.hex(' ')}")
    return raw[0]


def pad4(data: bytes) -> bytes:
    return data + b"\xFF" * ((-len(data)) & 3)


def parse_firmware_metadata(data: bytes) -> FirmwareInfo:
    if len(data) < 2:
        return FirmwareInfo(installed=False, signature=0)
    signature = struct.unpack_from("<H", data, 0)[0]
    if signature != APP_SIGNATURE:
        return FirmwareInfo(installed=False, signature=signature)
    revision = data[2] if len(data) > 2 else None
    patch = data[3] if len(data) > 3 else None
    name_raw = data[4:20]
    nul = name_raw.find(b"\x00")
    if nul >= 0:
        name_raw = name_raw[:nul]
    return FirmwareInfo(
        installed=True,
        signature=signature,
        revision=revision,
        patch=patch,
        name=name_raw.decode("ascii", errors="replace"),
    )


def load_application_image(filename: str) -> ImageInfo:
    path = Path(filename)
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise ProgrammerError(f"Cannot read image '{path}': {exc}") from exc
    if not raw:
        raise ProgrammerError(f"Image '{path}' is empty")
    if len(raw) > MAX_APP_IMAGE_SIZE:
        raise ProgrammerError(
            f"Application image is too large: {len(raw)} bytes "
            f"(maximum {MAX_APP_IMAGE_SIZE})"
        )
    padded = pad4(raw)
    return ImageInfo(
        path=path,
        raw_size=len(raw),
        padded=padded,
        firmware=parse_firmware_metadata(padded[:20]),
        sha256=hashlib.sha256(raw).hexdigest(),
    )


def load_bootloader_image(filename: str) -> tuple[Path, bytes, str]:
    path = Path(filename)
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise ProgrammerError(f"Cannot read image '{path}': {exc}") from exc
    if not raw:
        raise ProgrammerError(f"Image '{path}' is empty")
    if len(raw) > MAX_BOOT_IMAGE_SIZE:
        raise ProgrammerError(
            f"Bootloader image is too large: {len(raw)} bytes "
            f"(maximum {MAX_BOOT_IMAGE_SIZE})"
        )
    data = pad4(raw)
    if len(data) % BLOCK_SIZE == 0 and len(data) != MAX_BOOT_IMAGE_SIZE:
        data += b"\xFF" * 4
    return path, data, hashlib.sha256(raw).hexdigest()


def confirm_destructive(action: str, assume_yes: bool) -> None:
    if assume_yes:
        return
    if not sys.stdin.isatty():
        raise ProgrammerError(
            f"{action} requires confirmation; rerun with --yes for non-interactive use"
        )
    answer = input(f"{action}. Continue? [y/N] ").strip().lower()
    if answer not in ("y", "yes"):
        raise ProgrammerError("Operation cancelled")


class Escape32Serial:
    def __init__(
        self,
        port: str,
        read_timeout: float = READ_TIMEOUT_S,
        pacing: bool = True,
        verbose: bool = False,
    ) -> None:
        require_pyserial()
        self.port = port
        self.read_timeout = read_timeout
        self.pacing = pacing
        self.verbose = verbose
        self.ser = None

    def __enter__(self) -> "Escape32Serial":
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=BAUDRATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.050,
                write_timeout=1.0,
            )
        except serial.SerialException as exc:
            raise ProgrammerError(f"Cannot open {self.port}: {exc}") from exc
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.ser is not None:
            try:
                self.ser.close()
            finally:
                self.ser = None

    def _trace_tx(self, data: bytes) -> None:
        if self.verbose:
            print(f"TX  {data.hex(' ')}")

    def _trace_rx(self, data: bytes) -> None:
        if self.verbose:
            print(f"RX  {data.hex(' ')}")

    def flush_input(self) -> None:
        assert self.ser is not None
        self.ser.reset_input_buffer()

    def send_buf(self, data: bytes) -> None:
        assert self.ser is not None
        pos = 0
        while pos < len(data):
            chunk = data[pos : pos + TX_CHUNK_SIZE]
            self._trace_tx(chunk)
            try:
                written = self.ser.write(chunk)
            except serial.SerialException as exc:
                raise ProgrammerError(f"Serial write failed: {exc}") from exc
            if written != len(chunk):
                raise ProgrammerError(
                    f"Short serial write: {written}/{len(chunk)} bytes"
                )
            if self.pacing and written:
                time.sleep(written * TX_BYTE_TIME_S)
            pos += written

    def recv_exact(self, length: int, timeout: Optional[float] = None) -> bytes:
        assert self.ser is not None
        if timeout is None:
            timeout = self.read_timeout
        deadline = time.monotonic() + timeout
        out = bytearray()
        while len(out) < length:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self.ser.timeout = min(0.050, remaining)
            try:
                chunk = self.ser.read(length - len(out))
            except serial.SerialException as exc:
                raise ProgrammerError(f"Serial read failed: {exc}") from exc
            if chunk:
                out.extend(chunk)
        data = bytes(out)
        if data:
            self._trace_rx(data)
        if len(data) != length:
            raise ProgrammerError(
                f"Serial timeout: expected {length} byte(s), received {len(data)}"
            )
        return data

    def recv_until_magic(self, magic: bytes, timeout: float) -> None:
        assert self.ser is not None
        deadline = time.monotonic() + timeout
        matched = 0
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            self.ser.timeout = min(0.050, max(remaining, 0.001))
            try:
                b = self.ser.read(1)
            except serial.SerialException as exc:
                raise ProgrammerError(f"Serial read failed: {exc}") from exc
            if not b:
                continue
            if self.verbose:
                self._trace_rx(b)
            if b[0] == magic[matched]:
                matched += 1
                if matched == len(magic):
                    return
            else:
                matched = 1 if b[0] == magic[0] else 0
        raise ProgrammerError("Adapter response timeout")

    def send_value(self, value: int) -> None:
        self.send_buf(encode_value(value))

    def recv_value(self, timeout: Optional[float] = None) -> int:
        return decode_value(self.recv_exact(2, timeout))

    def recv_data(self, expected_length: Optional[int] = None) -> bytes:
        count = self.recv_value()
        length = (count + 1) << 2
        payload = self.recv_exact(length)
        crc_wire = struct.unpack("<I", self.recv_exact(4))[0]
        crc_calc = crc32_escape32(payload)
        if crc_wire != crc_calc:
            raise ProgrammerError(
                f"CRC mismatch: wire=0x{crc_wire:08X}, "
                f"calculated=0x{crc_calc:08X}"
            )
        if expected_length is not None and length != expected_length:
            raise ProgrammerError(
                f"Unexpected data length: {length}, expected {expected_length}"
            )
        return payload

    def send_data(self, payload: bytes) -> None:
        if not payload:
            raise ProgrammerError("Zero-length ESCape32 data frame is invalid")
        if len(payload) > BLOCK_SIZE:
            raise ProgrammerError(
                f"ESCape32 data frame too large: {len(payload)} bytes"
            )
        if len(payload) & 3:
            raise ProgrammerError(
                f"ESCape32 data length must be 4-byte aligned: {len(payload)} bytes"
            )
        self.send_value((len(payload) >> 2) - 1)
        self.send_buf(payload)
        self.send_buf(struct.pack("<I", crc32_escape32(payload)))


class Escape32Bootloader:
    def __init__(
        self,
        transport: Escape32Serial,
        probe_timeout: float = DEFAULT_PROBE_TIMEOUT_S,
    ) -> None:
        self.io = transport
        self.probe_timeout = probe_timeout

    def probe(self) -> None:
        deadline = time.monotonic() + self.probe_timeout
        attempts = 0
        last_error = None
        self.io.flush_input()
        while time.monotonic() < deadline:
            attempts += 1
            try:
                self.io.send_value(CMD_PROBE)
                if self.io.recv_value() == RES_OK:
                    return
                last_error = "unexpected probe response"
            except ProgrammerError as exc:
                last_error = str(exc)
            self.io.flush_input()
            time.sleep(0.050)
        if last_error and "Serial write failed" in last_error:
            raise ProgrammerError(
                f"Serial TX failed while probing bootloader after "
                f"{attempts} attempt(s): {last_error}"
            )
        raise ProgrammerError(
            f"No ESCape32 bootloader response within "
            f"{self.probe_timeout:.1f}s ({attempts} probe attempt(s))"
        )

    def recv_ack(self, context: str, timeout: Optional[float] = None) -> None:
        value = self.io.recv_value(timeout=timeout)
        if value != RES_OK:
            raise ProgrammerError(f"{context}: result {value}, expected {RES_OK}")

    def get_info(self) -> BootInfo:
        self.io.send_value(CMD_INFO)
        data = self.io.recv_data(expected_length=32)
        return BootInfo(
            revision=data[0],
            io_pin=data[1],
            idcode=struct.unpack_from("<I", data, 2)[0],
        )

    def read_block(self, block_num: int, length: int) -> bytes:
        if not 0 <= block_num <= 0xFF:
            raise ProgrammerError(f"Invalid block number: {block_num}")
        if not 4 <= length <= BLOCK_SIZE or (length & 3):
            raise ProgrammerError(f"Invalid read length: {length}")
        self.io.send_value(CMD_READ)
        self.io.send_value(block_num)
        self.io.send_value((length >> 2) - 1)
        return self.io.recv_data(expected_length=length)

    def read_firmware_info(self) -> FirmwareInfo:
        return parse_firmware_metadata(self.read_block(0, 20))

    def write_block(self, block_num: int, payload: bytes) -> None:
        if not 0 <= block_num <= 0xFF:
            raise ProgrammerError(f"Invalid block number: {block_num}")
        self.io.send_value(CMD_WRITE)
        self.io.send_value(block_num)
        self.io.send_data(payload)
        self.recv_ack(f"Error writing block {block_num}")

    def flash_application(self, image: ImageInfo, verify: bool = True) -> None:
        data = image.padded
        block_count = image.block_count
        if block_count > 2:
            dummy = b"\xFF" * 8
            self.write_block(0, dummy)
            self.write_block(1, dummy)
        for i in range(block_count):
            block_num = (i + 2) % block_count if block_count > 2 else i
            pos = block_num * BLOCK_SIZE
            payload = data[pos : min(pos + BLOCK_SIZE, len(data))]
            print(
                f"Programming block {i + 1:3d}/{block_count:3d} "
                f"(index {block_num:3d})...",
                end=" ",
                flush=True,
            )
            self.write_block(block_num, payload)
            print("OK")
        if verify:
            for block_num in range(block_count):
                pos = block_num * BLOCK_SIZE
                expected = data[pos : min(pos + BLOCK_SIZE, len(data))]
                print(
                    f"Verifying block  {block_num + 1:3d}/{block_count:3d} "
                    f"(index {block_num:3d})...",
                    end=" ",
                    flush=True,
                )
                actual = self.read_block(block_num, len(expected))
                if actual != expected:
                    mismatch = next(
                        (
                            i
                            for i, (a, b) in enumerate(zip(actual, expected))
                            if a != b
                        ),
                        0,
                    )
                    raise ProgrammerError(
                        f"Verify failed at image offset 0x{pos + mismatch:05X}"
                    )
                print("OK")

    def update_bootloader(self, data: bytes) -> None:
        self.io.send_value(CMD_UPDATE)
        chunks = (len(data) + BLOCK_SIZE - 1) // BLOCK_SIZE
        for i, pos in enumerate(range(0, len(data), BLOCK_SIZE), start=1):
            payload = data[pos : pos + BLOCK_SIZE]
            print(
                f"Programming bootloader {i:2d}/{chunks:2d}...",
                end=" ",
                flush=True,
            )
            self.io.send_data(payload)
            self.recv_ack("Bootloader block write failed")
            print("OK")
        self.recv_ack("Bootloader update failed after reboot", timeout=2.0)

    def set_write_protection(self, level: int) -> None:
        if level not in WRP_LEVELS:
            raise ProgrammerError(f"Invalid write-protection level: {level}")
        self.io.send_value(CMD_SETWRP)
        self.io.send_value(WRP_LEVELS[level])
        self.recv_ack("Write-protection operation failed", timeout=2.0)


class Escape32Adapter:
    def __init__(self, transport: Escape32Serial) -> None:
        self.io = transport
        self.sequence = int(time.time() * 1000) & 0xFFFFFFFF

    def request(
        self,
        command: int,
        payload: bytes = b"",
        timeout: float = 1.0,
    ) -> AdapterInfo:
        if len(payload) > 64:
            raise ProgrammerError("Adapter control payload is too large")
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        header = struct.pack(
            "<BBHI",
            ADAPTER_PROTOCOL_VERSION,
            command,
            len(payload),
            self.sequence,
        )
        crc = crc32_escape32(header + payload)
        frame = (
            ADAPTER_REQUEST_MAGIC
            + header
            + payload
            + struct.pack("<I", crc)
        )

        self.io.flush_input()
        self.io.send_buf(frame)
        self.io.recv_until_magic(ADAPTER_RESPONSE_MAGIC, timeout)
        rsp_header = self.io.recv_exact(8, timeout)
        version, rsp_command, length, sequence = struct.unpack(
            "<BBHI", rsp_header
        )
        if version != ADAPTER_PROTOCOL_VERSION:
            raise ProgrammerError(
                f"Unsupported adapter protocol version: {version}"
            )
        if rsp_command != (command | 0x80):
            raise ProgrammerError(
                f"Unexpected adapter response command: 0x{rsp_command:02X}"
            )
        if sequence != self.sequence:
            raise ProgrammerError(
                f"Adapter response sequence mismatch: {sequence} != {self.sequence}"
            )
        if length > 64:
            raise ProgrammerError(f"Invalid adapter response length: {length}")
        rsp_payload = self.io.recv_exact(length, timeout)
        crc_wire = struct.unpack("<I", self.io.recv_exact(4, timeout))[0]
        crc_calc = crc32_escape32(rsp_header + rsp_payload)
        if crc_wire != crc_calc:
            raise ProgrammerError(
                f"Adapter CRC mismatch: wire=0x{crc_wire:08X}, "
                f"calculated=0x{crc_calc:08X}"
            )
        info = self._parse_info(rsp_payload)
        if info.status != 0:
            label = ADAPTER_STATUS.get(
                info.status, f"STATUS_{info.status}"
            )
            raise ProgrammerError(f"Adapter command failed: {label}")
        return info

    @staticmethod
    def _parse_info(payload: bytes) -> AdapterInfo:
        if len(payload) != ADAPTER_INFO_STRUCT.size:
            raise ProgrammerError(
                f"Unexpected adapter info length: {len(payload)}, "
                f"expected {ADAPTER_INFO_STRUCT.size}"
            )
        values = ADAPTER_INFO_STRUCT.unpack(payload)
        return AdapterInfo(
            status=values[0],
            fw_major=values[1],
            fw_minor=values[2],
            fw_patch=values[3],
            wifi_configured=bool(values[4]),
            wifi_active=bool(values[5]),
            reboot_required=bool(values[6]),
            owner=values[7],
            uart_baud=values[8],
            uart_rx=values[9],
            uart_tx=values[10],
        )

    def info(self) -> AdapterInfo:
        return self.request(ADAPTER_CMD_INFO)

    def wifi_status(self) -> AdapterInfo:
        return self.request(ADAPTER_CMD_WIFI_GET)

    def wifi_set(self, enabled: bool) -> AdapterInfo:
        return self.request(
            ADAPTER_CMD_WIFI_SET,
            bytes((1 if enabled else 0,)),
        )

    def reboot(self) -> AdapterInfo:
        return self.request(ADAPTER_CMD_REBOOT)

    def acquire(self) -> AdapterInfo:
        return self.request(ADAPTER_CMD_ACQUIRE_ESC)

    def release(self) -> AdapterInfo:
        return self.request(ADAPTER_CMD_RELEASE_ESC)


def print_boot_info(info: BootInfo) -> None:
    print(f"Bootloader revision..... {info.revision}")
    print(f"Bootloader I/O code..... {info.io_pin}")
    print(f"DBGMCU_IDCODE........... 0x{info.idcode:08X}")
    print(f"STM32 DEV_ID............ 0x{info.dev_id:03X}")
    print(f"Silicon REV_ID.......... 0x{info.silicon_revision:04X}")


def print_firmware_info(fw: FirmwareInfo) -> None:
    if fw.installed:
        print(f"Application signature... 0x{fw.signature:04X}")
        print(f"Firmware revision....... {fw.version_string}")
        print(f"Firmware target......... {fw.name or '(empty)'}")
    else:
        print(
            f"Application signature... 0x{fw.signature:04X} "
            f"(expected 0x{APP_SIGNATURE:04X})"
        )
        print("Firmware................ NOT INSTALLED / INVALID")


def print_image_info(image: ImageInfo) -> None:
    print(f"Image................... {image.path}")
    print(f"Image size.............. {image.raw_size} bytes")
    print(f"Padded size............. {image.padded_size} bytes")
    print(f"Blocks.................. {image.block_count}")
    print(f"SHA-256................. {image.sha256}")
    print_firmware_info(image.firmware)


def print_adapter_info(info: AdapterInfo) -> None:
    print(f"Adapter firmware........ {info.version_string}")
    print("Transport............... USB Serial/JTAG")
    print(f"ESC UART................ {info.uart_baud} baud")
    print(f"ESC UART RX GPIO........ {info.uart_rx}")
    print(f"ESC UART TX GPIO........ {info.uart_tx}")
    print(
        f"Wi-Fi configured........ {'ON' if info.wifi_configured else 'OFF'}"
    )
    print(f"Wi-Fi active............ {'ON' if info.wifi_active else 'OFF'}")
    print(
        f"Reboot required......... {'YES' if info.reboot_required else 'NO'}"
    )
    print(f"ESC owner............... {info.owner_name}")


def validate_expectations(
    args,
    info: BootInfo,
    fw: Optional[FirmwareInfo],
) -> None:
    issues = []
    if (
        args.expect_boot_revision is not None
        and info.revision != args.expect_boot_revision
    ):
        issues.append(
            f"bootloader revision {info.revision} "
            f"(expected {args.expect_boot_revision})"
        )
    if args.expect_io_pin is not None and info.io_pin != args.expect_io_pin:
        issues.append(
            f"bootloader IO code {info.io_pin} (expected {args.expect_io_pin})"
        )
    if args.expect_dev_id is not None and info.dev_id != args.expect_dev_id:
        issues.append(
            f"STM32 DEV_ID 0x{info.dev_id:03X} "
            f"(expected 0x{args.expect_dev_id:03X})"
        )
    if args.expect_target is not None:
        if fw is None or not fw.installed:
            issues.append(
                f"installed firmware target unavailable "
                f"(expected '{args.expect_target}')"
            )
        elif fw.name != args.expect_target:
            issues.append(
                f"firmware target '{fw.name}' (expected '{args.expect_target}')"
            )
    if issues:
        raise ProgrammerError(
            "Target validation failed: " + "; ".join(issues)
        )


def self_test() -> int:
    assert crc32_escape32(b"123456789") == 0xCBF43926
    for value in (0x00, 0x01, 0x02, 0x03, 0x04, 0x55, 0xFF):
        assert decode_value(encode_value(value)) == value
    sample = (
        struct.pack("<HBB", APP_SIGNATURE, 16, 0)
        + b"GENERIC\x00"
        + b"\x00" * 8
    )
    fw = parse_firmware_metadata(sample)
    assert fw.installed
    assert fw.revision == 16
    assert fw.patch == 0
    assert fw.name == "GENERIC"
    assert pad4(b"1") == b"1\xFF\xFF\xFF"
    assert pad4(b"1234") == b"1234"
    for n, expected in (
        (1, [0]),
        (2, [0, 1]),
        (3, [2, 0, 1]),
        (5, [2, 3, 4, 0, 1]),
    ):
        order = [((i + 2) % n if n > 2 else i) for i in range(n)]
        assert order == expected
    payload = ADAPTER_INFO_STRUCT.pack(
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
        38400,
        4,
        2,
    )
    info = Escape32Adapter._parse_info(payload)
    assert info.status == 0
    assert info.uart_baud == 38400
    print("Self-test: PASS")
    return 0


def parse_int(value: str) -> int:
    try:
        return int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid integer: {value}") from exc


def add_connection_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--port",
        help="Serial port, e.g. COM7 or /dev/ttyACM0",
    )
    parser.add_argument(
        "--probe-timeout",
        type=float,
        default=DEFAULT_PROBE_TIMEOUT_S,
        help=(
            "Bootloader probe timeout in seconds "
            f"(default: {DEFAULT_PROBE_TIMEOUT_S})"
        ),
    )
    parser.add_argument(
        "--no-pacing",
        action="store_true",
        help="Disable conservative 38400-baud TX pacing",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Show raw TX/RX bytes",
    )
    parser.add_argument(
        "--expect-boot-revision",
        type=parse_int,
        help="Require a specific bootloader revision",
    )
    parser.add_argument(
        "--expect-io-pin",
        type=parse_int,
        help="Require a specific bootloader I/O code",
    )
    parser.add_argument(
        "--expect-dev-id",
        type=parse_int,
        help="Require a specific STM32 DEV_ID, e.g. 0x468",
    )
    parser.add_argument(
        "--expect-target",
        help="Require the currently installed ESCape32 target name",
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "ESCape32 bootloader programmer and ESCape32 Link controller"
        )
    )
    parser.add_argument(
        "--version",
        action="version",
        version=f"%(prog)s {TOOL_VERSION}",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="List serial ports and exit",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Run local protocol self-test",
    )
    add_connection_args(parser)

    sub = parser.add_subparsers(dest="command")
    sub.add_parser(
        "info",
        help="Probe bootloader and read installed firmware info",
    )

    inspect_p = sub.add_parser(
        "inspect-image",
        help="Inspect an application .bin without hardware",
    )
    inspect_p.add_argument("image", help="Application binary image")

    flash_p = sub.add_parser(
        "flash",
        help="Program an ESCape32 application .bin",
    )
    flash_p.add_argument("image", help="Application binary image")
    flash_p.add_argument(
        "--force",
        action="store_true",
        help=(
            "Allow programming an image without the ESCape32 "
            "application signature"
        ),
    )
    flash_p.add_argument(
        "--no-verify",
        action="store_true",
        help="Skip read-back verification",
    )
    flash_p.add_argument(
        "--yes",
        action="store_true",
        help="Skip interactive confirmation",
    )

    boot_p = sub.add_parser(
        "bootloader",
        help="Update the ESCape32 bootloader image",
    )
    boot_p.add_argument(
        "image",
        help="Bootloader binary image (maximum 4096 bytes)",
    )
    boot_p.add_argument(
        "--yes",
        action="store_true",
        help="Skip interactive confirmation",
    )

    wrp_p = sub.add_parser("set-wrp", help="Set write protection")
    wrp_p.add_argument(
        "level",
        type=int,
        choices=(0, 1, 2),
        help="0=off, 1=bootloader, 2=full",
    )
    wrp_p.add_argument(
        "--yes",
        action="store_true",
        help="Skip interactive confirmation",
    )

    adapter_p = sub.add_parser(
        "adapter",
        help="Control the ESP32-C3 ESCape32 Link",
    )
    adapter_sub = adapter_p.add_subparsers(
        dest="adapter_command",
        required=True,
    )
    adapter_sub.add_parser("info", help="Read adapter state")

    wifi_p = adapter_sub.add_parser(
        "wifi",
        help="Read or change persistent Wi-Fi state",
    )
    wifi_sub = wifi_p.add_subparsers(
        dest="wifi_action",
        required=True,
    )
    wifi_sub.add_parser("status", help="Read Wi-Fi state")
    wifi_on = wifi_sub.add_parser("on", help="Enable Wi-Fi after reboot")
    wifi_on.add_argument(
        "--reboot",
        action="store_true",
        help="Reboot adapter after changing the setting",
    )
    wifi_off = wifi_sub.add_parser(
        "off",
        help="Disable Wi-Fi after reboot",
    )
    wifi_off.add_argument(
        "--reboot",
        action="store_true",
        help="Reboot adapter after changing the setting",
    )

    adapter_sub.add_parser(
        "reboot",
        help="Reboot the ESP32-C3 adapter",
    )
    adapter_sub.add_parser(
        "acquire",
        help="Acquire ESC ownership for USB",
    )
    adapter_sub.add_parser(
        "release",
        help="Release ESC ownership from USB",
    )
    return parser


def open_transport(args) -> Escape32Serial:
    if not args.port:
        raise ProgrammerError("--port is required for this command")
    if args.probe_timeout <= 0:
        raise ProgrammerError("--probe-timeout must be greater than zero")
    return Escape32Serial(
        port=args.port,
        pacing=not args.no_pacing,
        verbose=args.verbose,
    )


def run_adapter_command(args) -> int:
    with open_transport(args) as transport:
        adapter = Escape32Adapter(transport)
        print(f"Port.................... {args.port}")

        if args.adapter_command == "info":
            info = adapter.info()
            print_adapter_info(info)
        elif args.adapter_command == "wifi":
            if args.wifi_action == "status":
                info = adapter.wifi_status()
            else:
                enabled = args.wifi_action == "on"
                info = adapter.wifi_set(enabled)
                print(
                    f"Wi-Fi configured........ "
                    f"{'ON' if info.wifi_configured else 'OFF'}"
                )
                print(
                    f"Wi-Fi active............ "
                    f"{'ON' if info.wifi_active else 'OFF'}"
                )
                print(
                    f"Reboot required......... "
                    f"{'YES' if info.reboot_required else 'NO'}"
                )
                if args.reboot:
                    print("Rebooting adapter........ ", end="", flush=True)
                    adapter.reboot()
                    print("OK")
                    print("RESULT: PASS")
                    return 0
            print_adapter_info(info)
        elif args.adapter_command == "reboot":
            print("Rebooting adapter........ ", end="", flush=True)
            adapter.reboot()
            print("OK")
        elif args.adapter_command == "acquire":
            info = adapter.acquire()
            print_adapter_info(info)
        elif args.adapter_command == "release":
            info = adapter.release()
            print_adapter_info(info)
        else:
            raise ProgrammerError(
                f"Unsupported adapter command: {args.adapter_command}"
            )

        print()
        print("RESULT: PASS")
        return 0


def run_bootloader_command(args) -> int:
    command = args.command or "info"
    with open_transport(args) as transport:
        boot = Escape32Bootloader(
            transport,
            probe_timeout=args.probe_timeout,
        )
        print(f"Port.................... {args.port}")
        print("Probing bootloader...... ", end="", flush=True)
        boot.probe()
        print("OK")
        print("Reading boot info....... ", end="", flush=True)
        boot_info = boot.get_info()
        print("OK")
        print_boot_info(boot_info)

        installed_fw = None
        if command in ("info", "flash") or args.expect_target is not None:
            print("Reading firmware info... ", end="", flush=True)
            installed_fw = boot.read_firmware_info()
            print("OK")
            print_firmware_info(installed_fw)

        validate_expectations(args, boot_info, installed_fw)

        if command == "info":
            print()
            print("RESULT: PASS")
            return 0

        if command == "flash":
            image = load_application_image(args.image)
            print()
            print("New application image")
            print_image_info(image)
            if not image.firmware.installed and not args.force:
                raise ProgrammerError(
                    "Application image does not contain ESCape32 signature "
                    "0x32EA; use --force only if this is intentional"
                )
            if (
                args.expect_target is not None
                and image.firmware.installed
                and image.firmware.name != args.expect_target
            ):
                raise ProgrammerError(
                    f"Image target '{image.firmware.name}' does not match "
                    f"--expect-target '{args.expect_target}'"
                )
            confirm_destructive(
                f"Program {image.raw_size} bytes to ESC application flash",
                args.yes,
            )
            print()
            boot.flash_application(
                image,
                verify=not args.no_verify,
            )
            final_fw = boot.read_firmware_info()
            if image.firmware.installed:
                if not final_fw.installed:
                    raise ProgrammerError(
                        "Post-flash metadata check: application signature missing"
                    )
                if (
                    final_fw.revision != image.firmware.revision
                    or final_fw.patch != image.firmware.patch
                    or final_fw.name != image.firmware.name
                ):
                    raise ProgrammerError(
                        "Post-flash metadata mismatch: "
                        f"got {final_fw.version_string} [{final_fw.name}], "
                        f"expected {image.firmware.version_string} "
                        f"[{image.firmware.name}]"
                    )
            print()
            print(
                "Post-flash firmware......",
                final_fw.version_string,
                f"[{final_fw.name}]",
            )
            print("RESULT: PASS")
            return 0

        if command == "bootloader":
            path, data, sha256 = load_bootloader_image(args.image)
            print(f"Bootloader image........ {path}")
            print(f"Transfer size........... {len(data)} bytes")
            print(f"SHA-256................. {sha256}")
            confirm_destructive(
                "Update ESCape32 bootloader",
                args.yes,
            )
            print()
            boot.update_bootloader(data)
            print()
            print("RESULT: PASS")
            return 0

        if command == "set-wrp":
            labels = {0: "off", 1: "bootloader", 2: "full"}
            confirm_destructive(
                f"Set write protection to level {args.level} "
                f"({labels[args.level]})",
                args.yes,
            )
            boot.set_write_protection(args.level)
            print()
            print("RESULT: PASS")
            return 0

        raise ProgrammerError(f"Unsupported command: {command}")


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    print(f"ESCape32 Programmer v{TOOL_VERSION}")
    print()

    try:
        if args.self_test:
            return self_test()
        if args.list_ports:
            return list_serial_ports()
        if args.command == "inspect-image":
            image = load_application_image(args.image)
            print_image_info(image)
            return 0
        if args.command == "adapter":
            return run_adapter_command(args)
        return run_bootloader_command(args)
    except ProgrammerError as exc:
        print()
        print("RESULT: FAIL")
        print(f"ERROR : {exc}")
        return 2
    except KeyboardInterrupt:
        print()
        print("RESULT: ABORTED")
        return 130


if __name__ == "__main__":
    sys.exit(main())
