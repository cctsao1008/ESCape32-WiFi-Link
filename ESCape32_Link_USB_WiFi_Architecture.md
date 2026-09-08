# ESCape32 Link USB / Wi-Fi Architecture

## Document Information

| Item | Value |
|---|---|
| Document | ESCape32 Link USB / Wi-Fi Architecture |
| Status | Working architecture baseline |
| Upstream baseline | `neoxic/ESCape32-WiFi-Link` |
| Extended implementation | `cctsao1008/ESCape32-WiFi-Link`, branch `usb-only` |
| Architecture originator | Ricardo Tsao |
| Document maintainer | Ricardo Tsao |
| Technical contact | GitHub: `cctsao1008` |
| Last updated | 2026-09-08 |

## 1. Scope and Upstream Baseline

The upstream ESCape32 Wi-Fi Link is an ESP32-based embedded configurator for ESCape32 electronic speed controllers. Its normal runtime path is Wi-Fi: the ESP32 provides an access point, Web UI, HTTP/WebSocket service, DNS/mDNS, and a 38400-baud UART connection to the ESC.

The upstream ESCape32-Tools project provides separate POSIX host utilities for direct serial CLI access and ESC firmware/bootloader update.

The extended implementation retains the upstream Wi-Fi configuration path while adding a wired USB control path and shared ESC signal services required by the target workflow.

## 2. Design Drivers

The extended architecture is driven by the following system requirements:

- A wired host interface shall remain available independently of Wi-Fi state.
- Wi-Fi shall remain available for compatibility with the upstream Web UI, but shall not be a mandatory runtime dependency.
- ESC programming, adapter control, signal generation, and diagnostics shall be accessible from one host-side control path.
- The existing single-wire ESC signal interface shall support UART programming/CLI operation and temporary PWM or DShot generation.
- USB and Wi-Fi access to the ESC shall be arbitrated so that multiple transports do not drive the shared ESC interface concurrently.
- Signal-generation modes shall return the interface to UART operation when generation stops or host control is lost.

## 3. Software Architecture

### 3.1 Runtime Overview

```text
                           Host PC
                              |
                   Native USB Serial/JTAG
                              |
                              v
                    +-------------------+
                    |    ESP32-C3       |
                    |   ESCape32 Link   |
                    |                   |
                    | USB control/data  |
                    | Optional Wi-Fi    |
                    | Ownership control |
                    | UART/PWM/DShot    |
                    +---------+---------+
                              |
                         ESC SIG path
                              |
                              v
                         ESCape32 ESC

            Phone / Notebook
                   |
             Optional Wi-Fi
                   |
                   +------> Upstream Web UI
```

The implementation contains two host-facing runtime planes:

- **Native USB plane** — always available for ESC UART bridging and adapter control.
- **Wi-Fi plane** — optional; when enabled, the upstream AP, Web UI, WebSocket, DNS, and mDNS services remain available.

### 3.2 Native USB Control and Data Plane

The ESP32-C3 native USB Serial/JTAG interface is used as the wired host transport. USB traffic serves two logical purposes:

1. transparent ESC UART traffic;
2. ESCape32 Link adapter-control traffic.

The host-side ESCape32 Programmer uses this path for ESC programming and adapter control. The implementation therefore does not require a separate external USB-to-UART bridge for normal host operation.

### 3.3 Optional Wi-Fi Plane

The upstream Wi-Fi service is retained but made optional. The current configuration stores the Wi-Fi enable state in NVS and initializes Wi-Fi services only when enabled.

When active, the upstream service model is retained:

- AP: `ESCape32-WiFi-Link`
- browser endpoint: `192.168.4.1`
- mDNS hostname: `escape32.local`
- HTTP/WebSocket configuration and update path

### 3.4 ESC UART Bridge

In UART mode, the ESP32-C3 uses the existing ESC single-wire interface at 38400 baud. Native USB traffic can be bridged transparently to the ESC while the adapter-control parser consumes only valid ESCape32 Link control frames.

### 3.5 Ownership and Arbitration

USB and Wi-Fi share the ESC UART resource. Runtime ownership prevents both transports from driving the ESC interface concurrently.

The same resource model also covers signal generation: while PWM or DShot is active, the ESC signal path is reserved for the USB-controlled generator and normal Wi-Fi ESC traffic is blocked.

### 3.6 UART / PWM / DShot Mode Management

The ESC signal interface has three mutually exclusive runtime modes:

| Mode | ESP32-C3 function | ESC path |
|---|---|---|
| UART | UART TX/RX | ESC programming / CLI |
| PWM | LEDC output | GPIO4 -> SIG |
| DShot | RMT output | GPIO4 -> SIG |

PWM currently supports 50..490 Hz. DShot generation supports DShot150, DShot300, and DShot600. When signal generation stops, the interface is returned to UART mode.

### 3.7 Host Control Role

`tools/escape32-programmer/escape32_programmer.py` is the host control application for the wired path. Its architectural role includes:

- ESC information and programming operations;
- application and bootloader update;
- write-protection control;
- adapter and Wi-Fi control;
- PWM/DShot signal control;
- diagnostic GPIO control.

Detailed command syntax is maintained in the Programmer README rather than in this architecture document.

## 4. Upstream and Extended Implementation

### 4.1 Architecture Difference

```text
Upstream WiFi-Link

Browser
   |
 Wi-Fi
   |
ESP32 WiFi-Link
   |
 UART
   |
  ESC


Extended implementation

Host PC -------- Native USB --------+
                                    |
Browser -------- Optional Wi-Fi ----+--> ESP32-C3 ESCape32 Link
                                             |
                                  ownership / mode control
                                             |
                                      UART / PWM / DShot
                                             |
                                            ESC
```

### 4.2 Functional Comparison

| Capability | Upstream implementation | Extended implementation |
|---|---|---|
| Primary runtime host path | Wi-Fi Web UI | Native USB + optional Wi-Fi |
| Wi-Fi service | Initialized at startup | Configurable; retained when enabled |
| Web UI / WebSocket | Present | Retained |
| Native USB runtime transport | Not used as ESC runtime path | Always available |
| USB-to-ESC UART bridge | Not present | Present |
| ESC host utility | Separate POSIX tools | Integrated Python Programmer |
| ESC firmware / bootloader update | Web UI and separate tools | Programmer plus retained Web UI |
| Transport arbitration | Not required by upstream model | USB / Wi-Fi ownership |
| PWM generation | Not present | Present |
| DShot generation | Not present | Present |
| Signal watchdog / UART restore | Not present | Present |
| Diagnostic GPIO control | Not present | Present |

The extended implementation is not a replacement for the upstream Web UI. It adds a wired runtime control path and additional shared-signal services while retaining the upstream Wi-Fi workflow.

## 5. Hardware Context

### 5.1 ESP32-C3 Interfaces

The board uses the ESP32-C3 native USB Serial/JTAG interface for the host-side wired connection. The ESC interface is not a conventional two-wire UART connection.

### 5.2 Single-Wire ESC Signal Interface

GPIO2 and GPIO4 are combined into the shared ESC `SIG` path through the board-level isolation network and 220-ohm series path.

```text
                         ESC SIG
                            |
                          220 ohm
                            |
                   +--------+--------+
                   |                 |
             GPIO4 / RX        1N4148 path
                                     |
                                 GPIO2 / TX
```

In UART mode:

- GPIO2 is used as UART TX;
- GPIO4 is used as UART RX;
- TX and RX share the external ESC `SIG` connection;
- the resulting interface is single-wire and half-duplex.

This differs from a conventional USB-to-UART adapter, where TX and RX remain separate between the adapter and the target.

In PWM or DShot mode, GPIO4 is reconfigured as a push-pull output and drives the existing `SIG` path through the 220-ohm series connection. GPIO2 UART transmission is inactive while the generator owns the signal path.

### 5.3 GPIO Mapping

| Function | ESP32-C3 resource | Note |
|---|---|---|
| Native USB | USB Serial/JTAG | Host transport |
| ESC UART TX | GPIO2 | Single-wire UART TX path |
| ESC UART RX | GPIO4 | Shared `SIG` path |
| PWM output | GPIO4 / LEDC | Reuses `SIG` |
| DShot output | GPIO4 / RMT | Reuses `SIG` |
| Status / diagnostic path | GPIO8 | Board-level LED behavior is implementation-specific |

## 6. References

- [neoxic/ESCape32-WiFi-Link](https://github.com/neoxic/ESCape32-WiFi-Link)
- [neoxic/ESCape32-Tools](https://github.com/neoxic/ESCape32-Tools)
- [ESCape32 Wiki - WiFiLink](https://github.com/neoxic/ESCape32/wiki/WiFiLink)
- [Extended implementation: `cctsao1008/ESCape32-WiFi-Link`, `usb-only`](https://github.com/cctsao1008/ESCape32-WiFi-Link/tree/usb-only)
