# ESCape32 Programmer

General-purpose Python utility for the ESCape32 serial bootloader and the
ESP32-C3 ESCape32 Link adapter.

## Requirements

- Python 3
- `pyserial`

```powershell
py -m pip install -r requirements.txt
```

## Serial ports

```powershell
py .\escape32_programmer.py --list-ports
```

## ESCape32 target operations

Read bootloader and installed firmware information:

```powershell
py .\escape32_programmer.py --port COM7 info
```

Inspect an application image without hardware:

```powershell
py .\escape32_programmer.py inspect-image .\ESCape32-target.bin
```

Program an application image and read it back for verification:

```powershell
py .\escape32_programmer.py --port COM7 flash .\ESCape32-target.bin
```

Skip the interactive confirmation:

```powershell
py .\escape32_programmer.py --port COM7 flash .\ESCape32-target.bin --yes
```

Skip read-back verification only when explicitly required:

```powershell
py .\escape32_programmer.py --port COM7 flash .\ESCape32-target.bin --no-verify
```

Update the ESCape32 bootloader:

```powershell
py .\escape32_programmer.py --port COM7 bootloader .\bootloader.bin
```

Set write protection:

```powershell
py .\escape32_programmer.py --port COM7 set-wrp 0
py .\escape32_programmer.py --port COM7 set-wrp 1
py .\escape32_programmer.py --port COM7 set-wrp 2
```

The application updater follows the ESCape32 signature-last strategy:
for images larger than two 1 KiB blocks, blocks 0 and 1 are invalidated first,
blocks 2..N are written, and blocks 0 and 1 are written last.

## ESCape32 Link adapter operations

Adapter information:

```powershell
py .\escape32_programmer.py --port COM7 adapter info
```

Wi-Fi state:

```powershell
py .\escape32_programmer.py --port COM7 adapter wifi status
```

Enable Wi-Fi persistently:

```powershell
py .\escape32_programmer.py --port COM7 adapter wifi on
```

Enable and reboot immediately:

```powershell
py .\escape32_programmer.py --port COM7 adapter wifi on --reboot
```

Disable Wi-Fi persistently:

```powershell
py .\escape32_programmer.py --port COM7 adapter wifi off --reboot
```

Wi-Fi is OFF by default. The setting is stored in the adapter NVS and becomes
the active runtime state after reboot.

When Wi-Fi is active, the original WiFi-Link behavior is retained:

```text
AP        ESCape32-WiFi-Link
Browser   http://192.168.4.1
mDNS      http://escape32.local
```

Manual ownership commands are also available for diagnostics:

```powershell
py .\escape32_programmer.py --port COM7 adapter acquire
py .\escape32_programmer.py --port COM7 adapter release
```

Normal USB ESC traffic automatically acquires a short USB ownership lease, so
these commands are not required for normal `info` or `flash` operation.

## Target validation

The programmer is not tied to a specific board. Optional expectations can be
used when a test needs stricter validation:

```powershell
py .\escape32_programmer.py `
  --port COM7 `
  --expect-boot-revision 4 `
  --expect-io-pin 2 `
  --expect-dev-id 0x468 `
  --expect-target TARGET_NAME `
  info
```

## Diagnostics

Raw serial traffic:

```powershell
py .\escape32_programmer.py --port COM7 --verbose info
```

Local protocol/parser self-test:

```powershell
py .\escape32_programmer.py --self-test
```

## Transport notes

The ESCape32 bootloader link is 38400 baud, 8N1, using the ESCape32
single-wire request/response protocol. On ESP32-C3 WiFi-Link hardware the host
side uses native USB Serial/JTAG; the COM-port baud setting does not determine
the USB physical transfer rate.
