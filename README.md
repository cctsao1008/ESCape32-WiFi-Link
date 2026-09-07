ESCape32 Wi-Fi Link
===================

ESP32-based link/configurator for [ESCape32](https://github.com/neoxic/ESCape32) electronic speed controllers.

USB + Optional Wi-Fi + Signal Generator
---------------------------------------

This branch keeps the original WiFi-Link AP/Web UI, adds an always-available native USB Serial/JTAG path for the ESCape32 Programmer, and can temporarily use the ESC signal pin as a PWM/DShot generator.

Runtime model:

```text
Host PC
  <-> ESP32-C3 native USB Serial/JTAG
       |
       +-- ESCape32 Programmer / adapter control
       +-- UART 38400 single-wire -> ESCape32
       +-- PWM 50..490 Hz         -> ESC signal
       +-- DShot150/300/600       -> ESC signal

Phone / Notebook
  <-> Wi-Fi AP (optional, default OFF)
  <-> original Web UI
  <-> ESCape32
```

Key behavior:

- USB transport is always enabled.
- Wi-Fi is **OFF by default** to reduce power consumption and heat.
- Wi-Fi ON/OFF is persistent in NVS and takes effect after reboot.
- With Wi-Fi enabled, the original AP, `192.168.4.1`, `escape32.local`, WebSocket protocol, and browser UI are retained.
- USB and Wi-Fi share the ESC UART path through ownership arbitration.
- PWM/DShot uses USB-exclusive ownership and blocks Wi-Fi ESC traffic until signal generation stops.
- A control watchdog returns the signal pin to UART mode if host keepalive stops.

ESP32-C3 signal use:

| Function | GPIO |
|---|---:|
| UART RX | 4 |
| UART TX | 2 |
| PWM/DShot output | 4 |
| LED | *8 |

(*) active low

UART mode keeps the original GPIO2/GPIO4 single-wire topology. PWM/DShot reuses GPIO4 as a push-pull output through the existing 220-ohm SIG path.

Building
--------

ESP-IDF v5.5 is the current baseline:

```text
idf.py set-target esp32c3
idf.py build
idf.py -p <PORT> flash
```

ESCape32 Programmer
-------------------

The general-purpose host utility is under:

```text
tools/escape32-programmer/
```

Examples:

```powershell
py .\escape32_programmer.py --list-ports
py .\escape32_programmer.py --port COM7 info
py .\escape32_programmer.py --port COM7 flash .\ESCape32-target.bin
```

Adapter / Wi-Fi:

```powershell
py .\escape32_programmer.py --port COM7 adapter info
py .\escape32_programmer.py --port COM7 adapter wifi on --reboot
py .\escape32_programmer.py --port COM7 adapter wifi off --reboot
```

PWM / DShot:

```powershell
py .\escape32_programmer.py --port COM7 adapter signal pwm --freq 50 --pulse-us 1500
py .\escape32_programmer.py --port COM7 adapter signal dshot --speed 600 --value 500
py .\escape32_programmer.py --port COM7 adapter signal stop
```

Signal-generator commands run in the foreground by default and automatically send watchdog keepalives. Press `Ctrl+C` to stop and return to UART mode.

Upstream
--------

The original project is maintained at [neoxic/ESCape32-WiFi-Link](https://github.com/neoxic/ESCape32-WiFi-Link).
