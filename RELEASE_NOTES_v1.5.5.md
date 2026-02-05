# Release Notes v1.5.5

## Bug Fixes

- **Fixed thread safety race conditions**: Six pairing/polling state fields were accessed across threads without mutex protection, risking missed IRK captures on dual-core ESP32 variants.
- **Fixed 32-bit timer overflow**: Absolute timer comparisons would fail after ~49.5 days of uptime. All timer checks now use wraparound-safe arithmetic.
- **Fixed post-disconnect IRK lookup**: Used the event's embedded connection descriptor instead of `ble_gap_conn_find`, which could fail after NimBLE removed the descriptor.
- **Fixed config defaults mismatch**: Python schema defaults now match the intended C++ behavior (`continuous_mode=true`, `max_captures=10`).
- **Fixed BLE name validation**: Compile-time limit reduced from 29 to 12 bytes to match the runtime Samsung S24/S25 compatibility constraint.

## Improvements

- IRK cache size now scales with `max_captures` setting instead of being hardcoded to 10.
- Replaced `esp_restart()` with `App.safe_reboot()` to avoid watchdog timeouts on profile change.
- Consolidated `dump_config` output into a single log line, removing `vTaskDelay` workarounds.
- Replaced magic number for NimBLE DHKey error with a named constant.
- Simplified `is_valid_irk`, MAC generation, and NVS health check logic.
- Added input validation for BLE profile select to reject invalid values.
- Removed unused `on_auth_complete` method and `to_hex_fwd` dead code.
