# ESCape32 Programmer

General-purpose Python utility for the ESCape32 serial bootloader and the ESP32-C3 ESCape32 Link adapter.

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

```powershell
py .\escape32_programmer.py --port COM7 info
py .\escape32_programmer.py inspect-image .\ESCape32-target.bin
py .\escape32_programmer.py --port COM7 flash .\ESCape32-target.bin
py .\escape32_programmer.py --port COM7 bootloader .\bootloader.bin
py .\escape32_programmer.py --port COM7 set-wrp 0
```

Application flashing uses the ESCape32 signature-last strategy and performs read-back verification by default.

## Adapter / Wi-Fi

```powershell
py .\escape32_programmer.py --port COM7 adapter info
py .\escape32_programmer.py --port COM7 adapter wifi status
py .\escape32_programmer.py --port COM7 adapter wifi on --reboot
py .\escape32_programmer.py --port COM7 adapter wifi off --reboot
```

Wi-Fi is OFF by default. The setting is stored in NVS and takes effect after reboot. With Wi-Fi enabled, the original WiFi-Link behavior remains available:

```text
AP        ESCape32-WiFi-Link
Browser   http://192.168.4.1
mDNS      http://escape32.local
```

## PWM / DShot signal generator

The ESP32-C3 can temporarily switch the ESC signal path from UART programming mode to PWM or DShot generation.

Signal status:

```powershell
py .\escape32_programmer.py --port COM7 adapter signal status
```

PWM, 50..490 Hz:

```powershell
py .\escape32_programmer.py --port COM7 adapter signal pwm --freq 50 --pulse-us 1500
```

A 0..100% helper is also available and maps to 1000..2000 us:

```powershell
py .\escape32_programmer.py --port COM7 adapter signal pwm --freq 50 --throttle 50
```

DShot150 / DShot300 / DShot600:

```powershell
py .\escape32_programmer.py --port COM7 adapter signal dshot --speed 600 --value 500
```

Optional DShot controls:

```powershell
py .\escape32_programmer.py --port COM7 adapter signal dshot --speed 300 --value 500 --rate-hz 1000 --telemetry
```

PWM/DShot commands run in the foreground and refresh the adapter watchdog. Press `Ctrl+C` to stop and automatically return the signal pin to UART mode. A fixed test duration can be used instead:

```powershell
py .\escape32_programmer.py --port COM7 adapter signal pwm --freq 50 --pulse-us 1500 --duration 10
```

Explicit stop:

```powershell
py .\escape32_programmer.py --port COM7 adapter signal stop
```

Signal modes:

```text
UART   GPIO2 TX + GPIO4 RX, ESCape32 bootloader / CLI
PWM    GPIO4 push-pull output through the existing 220-ohm SIG path
DSHOT  GPIO4 push-pull RMT output through the existing 220-ohm SIG path
```

When PWM or DShot is active, USB owns the ESC signal path and Wi-Fi ESC traffic is blocked. The control watchdog returns the adapter to UART mode if host refresh stops.

## Diagnostics

```powershell
py .\escape32_programmer.py --self-test
py .\escape32_programmer.py --port COM7 --verbose info
```

## Optional target validation

```powershell
py .\escape32_programmer.py `
  --port COM7 `
  --expect-boot-revision 4 `
  --expect-io-pin 2 `
  --expect-dev-id 0x468 `
  --expect-target TARGET_NAME `
  info
```

The programmer itself is not tied to a specific ESC board or project name.
