# Changelog

## 1.2.0

- Added ESP32-C3 ESCape32 Link adapter control protocol.
- Added `adapter info`.
- Added persistent Wi-Fi `status`, `on`, and `off` commands.
- Added adapter reboot command.
- Added diagnostic USB ownership acquire/release commands.
- Kept the programmer target-generic; no project-specific target is hard-coded.
- Retained application programming, signature-last update ordering, read-back
  verification, bootloader update, and write-protection control.

## 1.1.0

- Added general ESCape32 application programming via `CMD_WRITE`.
- Added read-back verification.
- Added bootloader update via `CMD_UPDATE`.
- Added write-protection control.
- Added application image inspection and optional target expectations.

## 1.0.1

- Initial read-only bootloader bring-up utility.
- Removed per-write serial flush that could aggravate USB bridge back-pressure.
