# Changelog

## 1.1.0

- Renamed the tool to the generic **ESCape32 Programmer**.
- Removed product-specific identity assumptions from the default workflow.
- Added application firmware programming via `CMD_WRITE`.
- Added signature invalidation and signature-last block ordering based on the
  reference ESCape32 updater.
- Added per-block CRC / ACK handling.
- Added application read-back verification (enabled by default).
- Added post-flash firmware metadata verification.
- Added ESCape32 bootloader update via `CMD_UPDATE`.
- Added write-protection control via `CMD_SETWRP`.
- Added generic `--expect-*` validation options.
- Preserved Windows USB serial behavior: 32-byte host TX chunks without
  per-chunk `Serial.flush()`.
