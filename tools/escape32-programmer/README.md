# ESCape32 Programmer

A general-purpose Python programmer for the native ESCape32 serial bootloader.
It is intended to work through a transparent serial transport such as the
USB-only ESCape32 WiFi-Link firmware in this repository.

## Features

- List serial ports.
- Repeated native bootloader probe (`CMD_PROBE`).
- Read bootloader information (`CMD_INFO`).
- Read installed application metadata (`CMD_READ`).
- Program an ESCape32 application image (`CMD_WRITE`).
- Signature-last update ordering to reduce the risk of leaving a half-flashed
  application bootable.
- CRC-32 framing and per-block ACK checking.
- Application read-back verification by default.
- ESCape32 bootloader update (`CMD_UPDATE`).
- Write-protection control (`CMD_SETWRP`).
- Optional generic target checks (`--expect-*`).
- Raw TX/RX trace (`--verbose`).
- Local protocol self-test.

The implementation follows the public ESCape32 bootloader protocol and the
reference update sequence in `neoxic/ESCape32-Tools/escape32-update`.

## Requirements

- Python 3
- `pyserial`
- An ESCape32-compatible serial link
- ESC bootloader signal connection and common ground

Install:

```powershell
py -m pip install -r requirements.txt
```

## List ports

```powershell
py .\escape32_programmer.py --list-ports
```

## Read target / firmware information

```powershell
py .\escape32_programmer.py --port COM7 info
```

For compatibility with the earlier bring-up tool, this also defaults to
`info`:

```powershell
py .\escape32_programmer.py --port COM7
```

Raw protocol trace:

```powershell
py .\escape32_programmer.py --port COM7 --verbose info
```

## Inspect an application image without hardware

```powershell
py .\escape32_programmer.py inspect-image .\ESCape32-target.bin
```

## Program application firmware

```powershell
py .\escape32_programmer.py --port COM7 flash .\ESCape32-target.bin
```

The tool asks for confirmation before writing. For scripted use:

```powershell
py .\escape32_programmer.py --port COM7 flash .\ESCape32-target.bin --yes
```

By default, application programming performs read-back verification after all
blocks are written. To disable it:

```powershell
py .\escape32_programmer.py --port COM7 flash .\ESCape32-target.bin --no-verify
```

### Application update sequence

For images larger than two 1 KiB blocks, the tool follows the reference
ESCape32 updater strategy:

1. Invalidate the signature area in blocks 0 and 1.
2. Program blocks 2..N-1.
3. Program blocks 0 and 1 last.
4. Require a bootloader ACK for every block.
5. Read every programmed block back and compare it with the image (default).
6. Re-read firmware metadata and check the programmed revision / target name.

Application images are limited to 128 KiB, matching the reference utility.

## Update the ESCape32 bootloader

Bootloader programming is a separate, higher-risk operation:

```powershell
py .\escape32_programmer.py --port COM7 bootloader .\bootloader.bin
```

The image is limited to 4096 bytes, matching the reference ESCape32 updater.

## Write protection

```powershell
py .\escape32_programmer.py --port COM7 set-wrp 0
py .\escape32_programmer.py --port COM7 set-wrp 1
py .\escape32_programmer.py --port COM7 set-wrp 2
```

Levels:

- `0`: protection off
- `1`: bootloader protected
- `2`: full protection

## Generic target validation

The tool does not hard-code any product name, MCU, bootloader I/O pin, or
firmware target. Optional checks are available when a workflow requires them:

```powershell
py .\escape32_programmer.py \
  --port COM7 \
  --expect-boot-revision 4 \
  --expect-io-pin 2 \
  --expect-dev-id 0x468 \
  --expect-target TARGET_NAME \
  info
```

The same checks can be used before `flash`.

## Self-test

No hardware is required:

```powershell
py .\escape32_programmer.py --self-test
```

Expected:

```text
Self-test: PASS
```

## Notes

- The serial protocol is 38400 baud, 8N1.
- ESCape32 values are encoded as a byte plus its bitwise complement.
- Data frames contain a 4-byte-aligned payload plus CRC-32.
- Host writes are kept to 32-byte chunks with conservative pacing to match the
  behavior of the reference ESCape32 update utility.
- Application flash and bootloader flash are destructive operations. Use the
  expected-target checks in automated or production workflows.
