ESCape32 Wi-Fi Link
===================

ESP32-based embedded configurator for [ESCape32](https://github.com/neoxic/ESCape32) electronic speed controllers.

USB-only branch
---------------

The `usb-only` branch repurposes the ESP32-C3 WiFi-Link hardware as a transparent
USB Serial/JTAG <-> ESC serial bridge. Wi-Fi, HTTP, WebSocket, mDNS, DNS, and the
embedded Web UI are not initialized or built in this branch.

Data path:

```
Host PC
  <-> ESP32-C3 native USB Serial/JTAG
  <-> UART1, 38400 baud, 8N1
  <-> ESCape32 single-wire interface
```

ESP32-C3 defaults:

| Signal | GPIO |
|--------|-----:|
| RX     |    4 |
| TX     |    2 |
| LED    |   *8 |

(*) active low

A general-purpose host programmer is included under:

```
tools/escape32-programmer/
```

It supports bootloader probe/info, application firmware programming and
read-back verification, bootloader update, and write-protection control.

Building the USB-only firmware
------------------------------

Install ESP-IDF v5.5, then run:

```
idf.py set-target esp32c3
idf.py build
```

To flash the ESP32-C3 bridge:

```
idf.py -p <PORT> flash
```

ESCape32 Programmer
-------------------

Install the Python dependency:

```powershell
cd tools\escape32-programmer
py -m pip install -r requirements.txt
```

List serial ports:

```powershell
py .\escape32_programmer.py --list-ports
```

Read bootloader / firmware information:

```powershell
py .\escape32_programmer.py --port COM7 info
```

Program an ESCape32 application image:

```powershell
py .\escape32_programmer.py --port COM7 flash .\ESCape32-target.bin
```

See `tools/escape32-programmer/README.md` for the complete CLI.

Upstream Wi-Fi Link
-------------------

The original Wi-Fi Link implementation and releases are maintained by
[neoxic/ESCape32-WiFi-Link](https://github.com/neoxic/ESCape32-WiFi-Link).
