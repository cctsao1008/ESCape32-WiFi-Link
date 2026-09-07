ESCape32 Wi-Fi Link
===================

ESP32-based link/configurator for [ESCape32](https://github.com/neoxic/ESCape32)
electronic speed controllers.

USB + Optional Wi-Fi
--------------------

This branch keeps the original WiFi-Link AP/Web UI and adds an always-available
native USB Serial/JTAG path for the ESCape32 Programmer.

Runtime model:

```text
Host PC
  <-> ESP32-C3 native USB Serial/JTAG
  <-> ownership mux
  <-> UART 38400, single-wire
  <-> ESCape32

Phone / Notebook
  <-> Wi-Fi AP (optional, default OFF)
  <-> original Web UI
  <-> ownership mux
  <-> ESCape32
```

Key behavior:

- USB transport is always enabled.
- Wi-Fi is **OFF by default** to reduce power consumption and heat.
- Wi-Fi ON/OFF is persistent in NVS and takes effect after reboot.
- When Wi-Fi is ON, the original AP, `192.168.4.1`, `escape32.local`,
  WebSocket protocol, and browser UI are retained.
- USB and Wi-Fi share the ESC interface through ownership arbitration.
- USB traffic is continuously consumed even while Wi-Fi owns the ESC, avoiding
  native USB back-pressure/write-timeout behavior.

ESP32-C3 defaults:

| Signal | GPIO |
|--------|-----:|
| RX     |    4 |
| TX     |    2 |
| LED    |   *8 |

(*) active low

Building
--------

ESP-IDF v5.5 is the current baseline:

```text
idf.py set-target esp32c3
idf.py build
idf.py -p <PORT> flash
```

The Web UI assets are gzip-compressed during the build, matching the upstream
WiFi-Link build flow.

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

Adapter control:

```powershell
py .\escape32_programmer.py --port COM7 adapter info
py .\escape32_programmer.py --port COM7 adapter wifi status
py .\escape32_programmer.py --port COM7 adapter wifi on --reboot
py .\escape32_programmer.py --port COM7 adapter wifi off --reboot
```

When Wi-Fi is enabled and the adapter reboots, connect to the original
`ESCape32-WiFi-Link` AP and open `escape32.local` or `192.168.4.1`.

Upstream
--------

The original project is maintained at
[neoxic/ESCape32-WiFi-Link](https://github.com/neoxic/ESCape32-WiFi-Link).
