#!/usr/bin/env python3
"""ESCape32 Programmer - bootloader, adapter, Wi-Fi and signal-generator utility."""

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

TOOL_VERSION = "1.4.0"

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
ADAPTER_CMD_SIGNAL_GET = 7
ADAPTER_CMD_SIGNAL_PWM = 8
ADAPTER_CMD_SIGNAL_DSHOT = 9
ADAPTER_CMD_SIGNAL_STOP = 10
ADAPTER_CMD_SIGNAL_KEEPALIVE = 11
ADAPTER_CMD_GPIO_GET = 12
ADAPTER_CMD_GPIO_SET = 13
ADAPTER_CMD_GPIO_RELEASE = 14

ADAPTER_STATUS = {
    0: "OK",
    1: "BUSY",
    2: "BAD_ARG",
    3: "NVS_ERROR",
    4: "INTERNAL",
}

ADAPTER_INFO_STRUCT = struct.Struct("<BBBBBBBBIBB")
SIGNAL_INFO_STRUCT = struct.Struct("<BBBBHHHHHH")
SIGNAL_PWM_STRUCT = struct.Struct("<HHH")
SIGNAL_DSHOT_STRUCT = struct.Struct("<HHHHB")
GPIO_INFO_STRUCT = struct.Struct("<BBBB")


class ProgrammerError(RuntimeError):
    pass


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
        return {0: "NONE", 1: "USB", 2: "WIFI"}.get(self.owner, f"UNKNOWN({self.owner})")


@dataclass
class SignalInfo:
    status: int
    mode: int
    gpio: int
    telemetry: bool
    pwm_freq_hz: int
    pwm_pulse_us: int
    dshot_speed: int
    dshot_value: int
    dshot_rate_hz: int
    watchdog_ms: int

    @property
    def mode_name(self) -> str:
        return {0: "UART", 1: "PWM", 2: "DSHOT"}.get(self.mode, f"UNKNOWN({self.mode})")


@dataclass
class GpioInfo:
    status: int
    gpio: int
    level: int
    override_active: bool


def require_pyserial() -> None:
    if serial is None:
        raise ProgrammerError("pyserial is not installed. Install it with:\n  py -m pip install pyserial")


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
    if len(raw) != 2 or (raw[0] ^ raw[1]) != 0xFF:
        raise ProgrammerError(f"Invalid complemented value: {raw.hex(' ')}")
    return raw[0]


def pad4(data: bytes) -> bytes:
    return data + b"\xFF" * ((-len(data)) & 3)


def parse_firmware_metadata(data: bytes) -> FirmwareInfo:
    if len(data) < 2:
        return FirmwareInfo(False, 0)
    signature = struct.unpack_from("<H", data, 0)[0]
    if signature != APP_SIGNATURE:
        return FirmwareInfo(False, signature)
    revision = data[2] if len(data) > 2 else None
    patch = data[3] if len(data) > 3 else None
    name_raw = data[4:20]
    nul = name_raw.find(b"\x00")
    if nul >= 0:
        name_raw = name_raw[:nul]
    return FirmwareInfo(True, signature, revision, patch, name_raw.decode("ascii", errors="replace"))


def load_application_image(filename: str) -> ImageInfo:
    path = Path(filename)
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise ProgrammerError(f"Cannot read image '{path}': {exc}") from exc
    if not raw:
        raise ProgrammerError(f"Image '{path}' is empty")
    if len(raw) > MAX_APP_IMAGE_SIZE:
        raise ProgrammerError(f"Application image is too large: {len(raw)} bytes")
    padded = pad4(raw)
    return ImageInfo(path, len(raw), padded, parse_firmware_metadata(padded[:20]), hashlib.sha256(raw).hexdigest())


def load_bootloader_image(filename: str) -> tuple[Path, bytes, str]:
    path = Path(filename)
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise ProgrammerError(f"Cannot read image '{path}': {exc}") from exc
    if not raw:
        raise ProgrammerError(f"Image '{path}' is empty")
    if len(raw) > MAX_BOOT_IMAGE_SIZE:
        raise ProgrammerError(f"Bootloader image is too large: {len(raw)} bytes")
    data = pad4(raw)
    if len(data) % BLOCK_SIZE == 0 and len(data) != MAX_BOOT_IMAGE_SIZE:
        data += b"\xFF" * 4
    return path, data, hashlib.sha256(raw).hexdigest()


def confirm_destructive(action: str, assume_yes: bool) -> None:
    if assume_yes:
        return
    if not sys.stdin.isatty():
        raise ProgrammerError(f"{action} requires confirmation; rerun with --yes")
    if input(f"{action}. Continue? [y/N] ").strip().lower() not in ("y", "yes"):
        raise ProgrammerError("Operation cancelled")


class Escape32Serial:
    def __init__(self, port: str, read_timeout: float = READ_TIMEOUT_S, pacing: bool = True, verbose: bool = False):
        require_pyserial()
        self.port = port
        self.read_timeout = read_timeout
        self.pacing = pacing
        self.verbose = verbose
        self.ser = None

    def __enter__(self):
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

    def __exit__(self, exc_type, exc, tb):
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def _trace_tx(self, data: bytes) -> None:
        if self.verbose:
            print(f"TX  {data.hex(' ')}")

    def _trace_rx(self, data: bytes) -> None:
        if self.verbose:
            print(f"RX  {data.hex(' ')}")

    def flush_input(self) -> None:
        self.ser.reset_input_buffer()

    def send_buf(self, data: bytes) -> None:
        pos = 0
        while pos < len(data):
            chunk = data[pos:pos + TX_CHUNK_SIZE]
            self._trace_tx(chunk)
            try:
                written = self.ser.write(chunk)
            except serial.SerialException as exc:
                raise ProgrammerError(f"Serial write failed: {exc}") from exc
            if written != len(chunk):
                raise ProgrammerError(f"Short serial write: {written}/{len(chunk)} bytes")
            if self.pacing and written:
                time.sleep(written * TX_BYTE_TIME_S)
            pos += written

    def recv_exact(self, length: int, timeout: Optional[float] = None) -> bytes:
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
            raise ProgrammerError(f"Serial timeout: expected {length} byte(s), received {len(data)}")
        return data

    def recv_until_magic(self, magic: bytes, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        matched = 0
        while time.monotonic() < deadline:
            self.ser.timeout = min(0.050, max(deadline - time.monotonic(), 0.001))
            b = self.ser.read(1)
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
            raise ProgrammerError(f"CRC mismatch: wire=0x{crc_wire:08X}, calculated=0x{crc_calc:08X}")
        if expected_length is not None and length != expected_length:
            raise ProgrammerError(f"Unexpected data length: {length}, expected {expected_length}")
        return payload

    def send_data(self, payload: bytes) -> None:
        if not payload or len(payload) > BLOCK_SIZE or len(payload) & 3:
            raise ProgrammerError(f"Invalid ESCape32 data length: {len(payload)}")
        self.send_value((len(payload) >> 2) - 1)
        self.send_buf(payload)
        self.send_buf(struct.pack("<I", crc32_escape32(payload)))


class Escape32Bootloader:
    def __init__(self, transport: Escape32Serial, probe_timeout: float = DEFAULT_PROBE_TIMEOUT_S):
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
            except ProgrammerError as exc:
                last_error = str(exc)
            self.io.flush_input()
            time.sleep(0.050)
        if last_error and "Serial write failed" in last_error:
            raise ProgrammerError(f"Serial TX failed while probing after {attempts} attempt(s): {last_error}")
        raise ProgrammerError(f"No ESCape32 bootloader response within {self.probe_timeout:.1f}s ({attempts} attempt(s))")

    def recv_ack(self, context: str, timeout: Optional[float] = None) -> None:
        value = self.io.recv_value(timeout)
        if value != RES_OK:
            raise ProgrammerError(f"{context}: result {value}, expected {RES_OK}")

    def get_info(self) -> BootInfo:
        self.io.send_value(CMD_INFO)
        data = self.io.recv_data(32)
        return BootInfo(data[0], data[1], struct.unpack_from("<I", data, 2)[0])

    def read_block(self, block_num: int, length: int) -> bytes:
        if not 0 <= block_num <= 0xFF or not 4 <= length <= BLOCK_SIZE or length & 3:
            raise ProgrammerError("Invalid block read request")
        self.io.send_value(CMD_READ)
        self.io.send_value(block_num)
        self.io.send_value((length >> 2) - 1)
        return self.io.recv_data(length)

    def read_firmware_info(self) -> FirmwareInfo:
        return parse_firmware_metadata(self.read_block(0, 20))

    def write_block(self, block_num: int, payload: bytes) -> None:
        self.io.send_value(CMD_WRITE)
        self.io.send_value(block_num)
        self.io.send_data(payload)
        self.recv_ack(f"Error writing block {block_num}")

    def flash_application(self, image: ImageInfo, verify: bool = True) -> None:
        data = image.padded
        n = image.block_count
        if n > 2:
            dummy = b"\xFF" * 8
            self.write_block(0, dummy)
            self.write_block(1, dummy)
        for i in range(n):
            num = (i + 2) % n if n > 2 else i
            pos = num * BLOCK_SIZE
            payload = data[pos:min(pos + BLOCK_SIZE, len(data))]
            print(f"Programming block {i + 1:3d}/{n:3d} (index {num:3d})... ", end="", flush=True)
            self.write_block(num, payload)
            print("OK")
        if verify:
            for num in range(n):
                pos = num * BLOCK_SIZE
                expected = data[pos:min(pos + BLOCK_SIZE, len(data))]
                print(f"Verifying block  {num + 1:3d}/{n:3d} (index {num:3d})... ", end="", flush=True)
                actual = self.read_block(num, len(expected))
                if actual != expected:
                    mismatch = next((i for i, (a, b) in enumerate(zip(actual, expected)) if a != b), 0)
                    raise ProgrammerError(f"Verify failed at image offset 0x{pos + mismatch:05X}")
                print("OK")

    def update_bootloader(self, data: bytes) -> None:
        self.io.send_value(CMD_UPDATE)
        chunks = (len(data) + BLOCK_SIZE - 1) // BLOCK_SIZE
        for i, pos in enumerate(range(0, len(data), BLOCK_SIZE), 1):
            payload = data[pos:pos + BLOCK_SIZE]
            print(f"Programming bootloader {i:2d}/{chunks:2d}... ", end="", flush=True)
            self.io.send_data(payload)
            self.recv_ack("Bootloader block write failed")
            print("OK")
        self.recv_ack("Bootloader update failed after reboot", 2.0)

    def set_write_protection(self, level: int) -> None:
        self.io.send_value(CMD_SETWRP)
        self.io.send_value(WRP_LEVELS[level])
        self.recv_ack("Write-protection operation failed", 2.0)


class Escape32Adapter:
    def __init__(self, transport: Escape32Serial):
        self.io = transport
        self.sequence = int(time.time() * 1000) & 0xFFFFFFFF

    def request_raw(self, command: int, payload: bytes = b"", timeout: float = 1.0) -> bytes:
        if len(payload) > 64:
            raise ProgrammerError("Adapter payload too large")
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        header = struct.pack("<BBHI", ADAPTER_PROTOCOL_VERSION, command, len(payload), self.sequence)
        frame = ADAPTER_REQUEST_MAGIC + header + payload + struct.pack("<I", crc32_escape32(header + payload))
        self.io.flush_input()
        self.io.send_buf(frame)
        self.io.recv_until_magic(ADAPTER_RESPONSE_MAGIC, timeout)
        rsp_header = self.io.recv_exact(8, timeout)
        version, rsp_command, length, sequence = struct.unpack("<BBHI", rsp_header)
        if version != ADAPTER_PROTOCOL_VERSION:
            raise ProgrammerError(f"Unsupported adapter protocol version: {version}")
        if rsp_command != (command | 0x80) or sequence != self.sequence or length > 64:
            raise ProgrammerError("Invalid adapter response header")
        rsp_payload = self.io.recv_exact(length, timeout)
        crc_wire = struct.unpack("<I", self.io.recv_exact(4, timeout))[0]
        if crc_wire != crc32_escape32(rsp_header + rsp_payload):
            raise ProgrammerError("Adapter response CRC mismatch")
        if not rsp_payload:
            raise ProgrammerError("Empty adapter response")
        status = rsp_payload[0]
        if status != 0:
            raise ProgrammerError(f"Adapter command failed: {ADAPTER_STATUS.get(status, f'STATUS_{status}')}")
        return rsp_payload

    @staticmethod
    def parse_info(payload: bytes) -> AdapterInfo:
        if len(payload) != ADAPTER_INFO_STRUCT.size:
            raise ProgrammerError(f"Unexpected adapter info length: {len(payload)}")
        v = ADAPTER_INFO_STRUCT.unpack(payload)
        return AdapterInfo(v[0], v[1], v[2], v[3], bool(v[4]), bool(v[5]), bool(v[6]), v[7], v[8], v[9], v[10])

    @staticmethod
    def parse_signal(payload: bytes) -> SignalInfo:
        if len(payload) != SIGNAL_INFO_STRUCT.size:
            raise ProgrammerError(f"Unexpected signal info length: {len(payload)}")
        v = SIGNAL_INFO_STRUCT.unpack(payload)
        return SignalInfo(v[0], v[1], v[2], bool(v[3]), v[4], v[5], v[6], v[7], v[8], v[9])

    @staticmethod
    def parse_gpio(payload: bytes) -> GpioInfo:
        if len(payload) != GPIO_INFO_STRUCT.size:
            raise ProgrammerError(f"Unexpected GPIO info length: {len(payload)}")
        v = GPIO_INFO_STRUCT.unpack(payload)
        return GpioInfo(v[0], v[1], v[2], bool(v[3]))

    def info(self) -> AdapterInfo:
        return self.parse_info(self.request_raw(ADAPTER_CMD_INFO))

    def wifi_status(self) -> AdapterInfo:
        return self.parse_info(self.request_raw(ADAPTER_CMD_WIFI_GET))

    def wifi_set(self, enabled: bool) -> AdapterInfo:
        return self.parse_info(self.request_raw(ADAPTER_CMD_WIFI_SET, bytes((1 if enabled else 0,))))

    def reboot(self) -> AdapterInfo:
        return self.parse_info(self.request_raw(ADAPTER_CMD_REBOOT))

    def acquire(self) -> AdapterInfo:
        return self.parse_info(self.request_raw(ADAPTER_CMD_ACQUIRE_ESC))

    def release(self) -> AdapterInfo:
        return self.parse_info(self.request_raw(ADAPTER_CMD_RELEASE_ESC))

    def signal_status(self) -> SignalInfo:
        return self.parse_signal(self.request_raw(ADAPTER_CMD_SIGNAL_GET))

    def signal_pwm(self, freq_hz: int, pulse_us: int, watchdog_ms: int) -> SignalInfo:
        payload = SIGNAL_PWM_STRUCT.pack(freq_hz, pulse_us, watchdog_ms)
        return self.parse_signal(self.request_raw(ADAPTER_CMD_SIGNAL_PWM, payload))

    def signal_dshot(self, speed: int, value: int, rate_hz: int, telemetry: bool, watchdog_ms: int) -> SignalInfo:
        payload = SIGNAL_DSHOT_STRUCT.pack(speed, value, rate_hz, watchdog_ms, 1 if telemetry else 0)
        return self.parse_signal(self.request_raw(ADAPTER_CMD_SIGNAL_DSHOT, payload))

    def signal_keepalive(self) -> SignalInfo:
        return self.parse_signal(self.request_raw(ADAPTER_CMD_SIGNAL_KEEPALIVE))

    def signal_stop(self) -> SignalInfo:
        return self.parse_signal(self.request_raw(ADAPTER_CMD_SIGNAL_STOP))

    def gpio_get(self, gpio: int) -> GpioInfo:
        return self.parse_gpio(self.request_raw(ADAPTER_CMD_GPIO_GET, bytes((gpio,))))

    def gpio_set(self, gpio: int, level: int) -> GpioInfo:
        return self.parse_gpio(self.request_raw(ADAPTER_CMD_GPIO_SET, bytes((gpio, level))))

    def gpio_release(self, gpio: int) -> GpioInfo:
        return self.parse_gpio(self.request_raw(ADAPTER_CMD_GPIO_RELEASE, bytes((gpio,))))


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
        print(f"Application signature... 0x{fw.signature:04X} (expected 0x{APP_SIGNATURE:04X})")
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
    print(f"Wi-Fi configured........ {'ON' if info.wifi_configured else 'OFF'}")
    print(f"Wi-Fi active............ {'ON' if info.wifi_active else 'OFF'}")
    print(f"Reboot required......... {'YES' if info.reboot_required else 'NO'}")
    print(f"ESC owner............... {info.owner_name}")


def print_signal_info(info: SignalInfo) -> None:
    print(f"Signal mode............. {info.mode_name}")
    print(f"Signal GPIO............. {info.gpio}")
    if info.mode == 1:
        print(f"PWM frequency........... {info.pwm_freq_hz} Hz")
        print(f"PWM pulse............... {info.pwm_pulse_us} us")
    elif info.mode == 2:
        print(f"DShot speed............. DShot{info.dshot_speed}")
        print(f"DShot value............. {info.dshot_value}")
        print(f"DShot frame rate........ {info.dshot_rate_hz} Hz")
        print(f"DShot telemetry......... {'ON' if info.telemetry else 'OFF'}")
    if info.mode != 0:
        print(f"Watchdog................ {info.watchdog_ms} ms")


def print_gpio_info(info: GpioInfo) -> None:
    print(f"GPIO.................... {info.gpio}")
    print(f"Raw level............... {info.level}")
    print(f"Manual override......... {'ON' if info.override_active else 'OFF'}")


def validate_expectations(args, info: BootInfo, fw: Optional[FirmwareInfo]) -> None:
    issues = []
    if args.expect_boot_revision is not None and info.revision != args.expect_boot_revision:
        issues.append(f"bootloader revision {info.revision} (expected {args.expect_boot_revision})")
    if args.expect_io_pin is not None and info.io_pin != args.expect_io_pin:
        issues.append(f"bootloader IO code {info.io_pin} (expected {args.expect_io_pin})")
    if args.expect_dev_id is not None and info.dev_id != args.expect_dev_id:
        issues.append(f"STM32 DEV_ID 0x{info.dev_id:03X} (expected 0x{args.expect_dev_id:03X})")
    if args.expect_target is not None:
        if fw is None or not fw.installed or fw.name != args.expect_target:
            issues.append(f"firmware target mismatch (expected '{args.expect_target}')")
    if issues:
        raise ProgrammerError("Target validation failed: " + "; ".join(issues))


def self_test() -> int:
    assert crc32_escape32(b"123456789") == 0xCBF43926
    for value in (0, 1, 2, 3, 4, 0x55, 0xFF):
        assert decode_value(encode_value(value)) == value
    sample = struct.pack("<HBB", APP_SIGNATURE, 16, 0) + b"GENERIC\x00" + b"\x00" * 8
    fw = parse_firmware_metadata(sample)
    assert fw.installed and fw.revision == 16 and fw.name == "GENERIC"
    a = ADAPTER_INFO_STRUCT.pack(0, 1, 0, 0, 0, 0, 0, 0, 38400, 4, 2)
    assert Escape32Adapter.parse_info(a).uart_baud == 38400
    s = SIGNAL_INFO_STRUCT.pack(0, 2, 4, 0, 0, 0, 600, 48, 1000, 1500)
    sig = Escape32Adapter.parse_signal(s)
    assert sig.mode == 2 and sig.dshot_speed == 600 and sig.dshot_value == 48
    g = Escape32Adapter.parse_gpio(GPIO_INFO_STRUCT.pack(0, 8, 1, 1))
    assert g.gpio == 8 and g.level == 1 and g.override_active
    print("Self-test: PASS")
    return 0


def parse_int(value: str) -> int:
    try:
        return int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid integer: {value}") from exc


def add_connection_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", help="Serial port, e.g. COM7 or /dev/ttyACM0")
    parser.add_argument("--probe-timeout", type=float, default=DEFAULT_PROBE_TIMEOUT_S)
    parser.add_argument("--no-pacing", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--expect-boot-revision", type=parse_int)
    parser.add_argument("--expect-io-pin", type=parse_int)
    parser.add_argument("--expect-dev-id", type=parse_int)
    parser.add_argument("--expect-target")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ESCape32 programmer and ESCape32 Link controller")
    parser.add_argument("--version", action="version", version=f"%(prog)s {TOOL_VERSION}")
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    add_connection_args(parser)
    sub = parser.add_subparsers(dest="command")
    sub.add_parser("info")

    inspect_p = sub.add_parser("inspect-image")
    inspect_p.add_argument("image")

    flash_p = sub.add_parser("flash")
    flash_p.add_argument("image")
    flash_p.add_argument("--force", action="store_true")
    flash_p.add_argument("--no-verify", action="store_true")
    flash_p.add_argument("--yes", action="store_true")

    boot_p = sub.add_parser("bootloader")
    boot_p.add_argument("image")
    boot_p.add_argument("--yes", action="store_true")

    wrp_p = sub.add_parser("set-wrp")
    wrp_p.add_argument("level", type=int, choices=(0, 1, 2))
    wrp_p.add_argument("--yes", action="store_true")

    adapter_p = sub.add_parser("adapter")
    adapter_sub = adapter_p.add_subparsers(dest="adapter_command", required=True)
    adapter_sub.add_parser("info")

    wifi_p = adapter_sub.add_parser("wifi")
    wifi_sub = wifi_p.add_subparsers(dest="wifi_action", required=True)
    wifi_sub.add_parser("status")
    wifi_on = wifi_sub.add_parser("on")
    wifi_on.add_argument("--reboot", action="store_true")
    wifi_off = wifi_sub.add_parser("off")
    wifi_off.add_argument("--reboot", action="store_true")

    adapter_sub.add_parser("reboot")
    adapter_sub.add_parser("acquire")
    adapter_sub.add_parser("release")

    gpio_p = adapter_sub.add_parser("gpio")
    gpio_sub = gpio_p.add_subparsers(dest="gpio_action", required=True)
    gpio_get = gpio_sub.add_parser("get")
    gpio_get.add_argument("pin", type=int)
    gpio_set = gpio_sub.add_parser("set")
    gpio_set.add_argument("pin", type=int)
    gpio_set.add_argument("level", type=int, choices=(0, 1))
    gpio_release = gpio_sub.add_parser("release")
    gpio_release.add_argument("pin", type=int)

    signal_p = adapter_sub.add_parser("signal")
    signal_sub = signal_p.add_subparsers(dest="signal_action", required=True)
    signal_sub.add_parser("status")
    signal_sub.add_parser("stop")

    pwm_p = signal_sub.add_parser("pwm")
    pwm_p.add_argument("--freq", type=int, required=True, choices=range(50, 491), metavar="50..490")
    pwm_value = pwm_p.add_mutually_exclusive_group(required=True)
    pwm_value.add_argument("--pulse-us", type=int)
    pwm_value.add_argument("--throttle", type=float)
    pwm_p.add_argument("--watchdog-ms", type=int, default=1500)
    pwm_p.add_argument("--duration", type=float, help="Seconds; omit to run until Ctrl+C")

    dshot_p = signal_sub.add_parser("dshot")
    dshot_p.add_argument("--speed", type=int, required=True, choices=(150, 300, 600))
    dshot_p.add_argument("--value", type=int, required=True)
    dshot_p.add_argument("--rate-hz", type=int, default=1000)
    dshot_p.add_argument("--telemetry", action="store_true")
    dshot_p.add_argument("--watchdog-ms", type=int, default=1500)
    dshot_p.add_argument("--duration", type=float, help="Seconds; omit to run until Ctrl+C")
    return parser


def open_transport(args) -> Escape32Serial:
    if not args.port:
        raise ProgrammerError("--port is required for this command")
    return Escape32Serial(args.port, pacing=not args.no_pacing, verbose=args.verbose)


def run_signal_session(adapter: Escape32Adapter, start_info: SignalInfo, watchdog_ms: int, duration: Optional[float]) -> int:
    print_signal_info(start_info)
    print()
    if duration is None:
        print("Signal generator running. Press Ctrl+C to stop.")
    else:
        if duration <= 0:
            raise ProgrammerError("--duration must be greater than zero")
        print(f"Signal generator running for {duration:g} s.")

    refresh = max(0.10, min(0.50, watchdog_ms / 3000.0))
    deadline = None if duration is None else time.monotonic() + duration
    try:
        while deadline is None or time.monotonic() < deadline:
            time.sleep(refresh)
            adapter.signal_keepalive()
    except KeyboardInterrupt:
        print()
    finally:
        try:
            stopped = adapter.signal_stop()
            print_signal_info(stopped)
        except ProgrammerError as exc:
            print(f"Signal stop warning...... {exc}")
    print()
    print("RESULT: PASS")
    return 0


def run_adapter_command(args) -> int:
    with open_transport(args) as transport:
        adapter = Escape32Adapter(transport)
        print(f"Port.................... {args.port}")

        if args.adapter_command == "info":
            print_adapter_info(adapter.info())
        elif args.adapter_command == "wifi":
            if args.wifi_action == "status":
                info = adapter.wifi_status()
            else:
                info = adapter.wifi_set(args.wifi_action == "on")
                if args.reboot:
                    print_adapter_info(info)
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
            print_adapter_info(adapter.acquire())
        elif args.adapter_command == "release":
            print_adapter_info(adapter.release())
        elif args.adapter_command == "gpio":
            if not 0 <= args.pin <= 255:
                raise ProgrammerError("GPIO pin must be 0..255")
            if args.gpio_action == "get":
                print_gpio_info(adapter.gpio_get(args.pin))
            elif args.gpio_action == "set":
                print_gpio_info(adapter.gpio_set(args.pin, args.level))
            elif args.gpio_action == "release":
                print_gpio_info(adapter.gpio_release(args.pin))
        elif args.adapter_command == "signal":
            if args.signal_action == "status":
                print_signal_info(adapter.signal_status())
            elif args.signal_action == "stop":
                print_signal_info(adapter.signal_stop())
            elif args.signal_action == "pwm":
                if not 250 <= args.watchdog_ms <= 10000:
                    raise ProgrammerError("--watchdog-ms must be 250..10000")
                pulse_us = args.pulse_us
                if args.throttle is not None:
                    if not 0.0 <= args.throttle <= 100.0:
                        raise ProgrammerError("--throttle must be 0..100")
                    pulse_us = int(round(1000.0 + args.throttle * 10.0))
                if not 500 <= pulse_us <= 2500:
                    raise ProgrammerError("--pulse-us must be 500..2500")
                info = adapter.signal_pwm(args.freq, pulse_us, args.watchdog_ms)
                return run_signal_session(adapter, info, args.watchdog_ms, args.duration)
            elif args.signal_action == "dshot":
                if not 0 <= args.value <= 2047:
                    raise ProgrammerError("--value must be 0..2047")
                if not 50 <= args.rate_hz <= 4000:
                    raise ProgrammerError("--rate-hz must be 50..4000")
                if not 250 <= args.watchdog_ms <= 10000:
                    raise ProgrammerError("--watchdog-ms must be 250..10000")
                info = adapter.signal_dshot(args.speed, args.value, args.rate_hz, args.telemetry, args.watchdog_ms)
                return run_signal_session(adapter, info, args.watchdog_ms, args.duration)
        else:
            raise ProgrammerError(f"Unsupported adapter command: {args.adapter_command}")

        print()
        print("RESULT: PASS")
        return 0


def run_bootloader_command(args) -> int:
    command = args.command or "info"
    with open_transport(args) as transport:
        boot = Escape32Bootloader(transport, args.probe_timeout)
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
            print("\nRESULT: PASS")
            return 0
        if command == "flash":
            image = load_application_image(args.image)
            print("\nNew application image")
            print_image_info(image)
            if not image.firmware.installed and not args.force:
                raise ProgrammerError("Application image does not contain ESCape32 signature 0x32EA")
            if args.expect_target and image.firmware.installed and image.firmware.name != args.expect_target:
                raise ProgrammerError("Image target does not match --expect-target")
            confirm_destructive(f"Program {image.raw_size} bytes to ESC application flash", args.yes)
            print()
            boot.flash_application(image, not args.no_verify)
            final_fw = boot.read_firmware_info()
            if image.firmware.installed and (
                not final_fw.installed or
                final_fw.revision != image.firmware.revision or
                final_fw.patch != image.firmware.patch or
                final_fw.name != image.firmware.name
            ):
                raise ProgrammerError("Post-flash metadata mismatch")
            print(f"\nPost-flash firmware...... {final_fw.version_string} [{final_fw.name}]")
            print("RESULT: PASS")
            return 0
        if command == "bootloader":
            path, data, sha256 = load_bootloader_image(args.image)
            print(f"Bootloader image........ {path}")
            print(f"Transfer size........... {len(data)} bytes")
            print(f"SHA-256................. {sha256}")
            confirm_destructive("Update ESCape32 bootloader", args.yes)
            boot.update_bootloader(data)
            print("\nRESULT: PASS")
            return 0
        if command == "set-wrp":
            confirm_destructive(f"Set write protection to level {args.level}", args.yes)
            boot.set_write_protection(args.level)
            print("\nRESULT: PASS")
            return 0
        raise ProgrammerError(f"Unsupported command: {command}")


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    print(f"ESCape32 Programmer v{TOOL_VERSION}\n")
    try:
        if args.self_test:
            return self_test()
        if args.list_ports:
            return list_serial_ports()
        if args.command == "inspect-image":
            print_image_info(load_application_image(args.image))
            return 0
        if args.command == "adapter":
            return run_adapter_command(args)
        return run_bootloader_command(args)
    except ProgrammerError as exc:
        print("\nRESULT: FAIL")
        print(f"ERROR : {exc}")
        return 2
    except KeyboardInterrupt:
        print("\nRESULT: ABORTED")
        return 130


if __name__ == "__main__":
    sys.exit(main())
