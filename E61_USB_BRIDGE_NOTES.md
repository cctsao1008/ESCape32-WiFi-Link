# E61 WiFi-Link USB Bridge v0.1

Baseline: `neoxic/ESCape32-WiFi-Link` commit `40af21ccfafd9cb8d1c9b951d57b54f3e36bd21f` (v1.10).

## Intent

Preserve the upstream WiFi AP, Web UI, WebSocket CLI and WebSocket firmware-update behavior, and add an ESP32-C3 native USB Serial/JTAG transparent byte bridge to the existing ESC UART.

The bridge does not parse ESCape32 commands, CRCs, blocks, or firmware images.

## Shared UART ownership

The ESC UART has exactly one logical owner at a time:

- `ESC_OWNER_WIFI`: existing WebSocket request/update transaction.
- `ESC_OWNER_USB`: raw USB Serial/JTAG session.
- `ESC_OWNER_NONE`: idle.

A WebSocket firmware update retains WiFi ownership across all binary frames. The update lease is tied to the WebSocket file descriptor and is released when the final frame completes, an update error occurs, or that socket disconnects. USB ownership is released after `CONFIG_USB_BRIDGE_IDLE_MS` of inactivity.

## Important upstream behavior retained

The existing WiFi `sendbuf()` keeps its `xQueueReset()` + `uart_flush()` behavior. The USB bridge does **not** call `sendbuf()` and never flushes per USB chunk. It flushes once only when a new USB ownership session begins.

## ESP32-C3 transport

Existing upstream UART configuration remains the source of truth:

- UART1 default
- RX GPIO4
- TX GPIO2
- 38400 baud
- 8N1
- no flow control
- `UART_MODE_RS485_HALF_DUPLEX`

The ESP32-C3 target-specific sdkconfig disables console/log output on USB so the USB byte stream is not contaminated by application or bootloader logs.

## Build status

Source candidate only. Not BUILD VERIFIED in the current ChatGPT environment because ESP-IDF is not installed here.

## First hardware test

With ESC disconnected:

```text
escape32-update-diag.exe -d COM5
```

Expected first improvement:

```text
WriteFile: SUCCESS
requested: 2
written:   2
```

The later RX timeout is expected with no ESC connected.

Then connect E61 and validate only `CMD_PROBE`, `CMD_INFO`, and `CMD_READ` before any write/update command.
