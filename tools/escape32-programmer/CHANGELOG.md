# Changelog

## 1.3.0

- Added ESP32-C3 PWM generator support at 50..490 Hz.
- Added DShot150, DShot300, and DShot600 generation using the RMT peripheral.
- Added `adapter signal status`, `pwm`, `dshot`, and `stop` commands.
- Added foreground watchdog keepalive handling; Ctrl+C or duration expiry returns the adapter to UART mode.
- Added USB ownership enforcement while PWM/DShot is active so Wi-Fi ESC traffic cannot collide with generated output.
- Uses GPIO4 as the push-pull PWM/DShot output path while retaining GPIO2/GPIO4 UART operation for ESCape32 programming.

## 1.2.0

- Added ESP32-C3 ESCape32 Link adapter control protocol.
- Added `adapter info`.
- Added persistent Wi-Fi `status`, `on`, and `off` commands.
- Added adapter reboot command.
- Added diagnostic USB ownership acquire/release commands.
- Kept the programmer target-generic; no project-specific target is hard-coded.
- Retained application programming, signature-last update ordering, read-back verification, bootloader update, and write-protection control.

## 1.1.0

- Added general ESCape32 application programming via `CMD_WRITE`.
- Added read-back verification.
- Added bootloader update via `CMD_UPDATE`.
- Added write-protection control.
- Added application image inspection and optional target expectations.

## 1.0.1

- Initial read-only bootloader bring-up utility.
- Removed per-write serial flush that could aggravate USB bridge back-pressure.
