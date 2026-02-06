# Release Notes v1.5.6

## Reliability Fixes

- **Fixed MAC rotation race condition**: `refresh_mac()` now sets the `suppress_next_adv_` flag before triggering disconnect, preventing `handle_gap_disconnect` from restarting advertising while the MAC rotation is still in progress.
- **Component fails cleanly on critical init errors**: `setup()` now calls `mark_failed()` and returns early if the FreeRTOS mutex cannot be created, and `register_gatt_services()` marks the component as failed if GATT registration fails.
- **NVS profile range validation**: Persisted BLE profile values are now range-checked before `static_cast`, preventing undefined behavior from corrupted NVS data.
- **BLE name whitespace trimming**: `sanitize_ble_name()` now trims leading and trailing spaces after character filtering, preventing names like `" IRK "` from wasting bytes in the advertising packet.
- **Fixed `host_synced_` cross-core visibility**: Changed from plain `bool` to `std::atomic<bool>` to ensure the NimBLE task's write is visible to the ESPHome main loop on dual-core ESP32 variants.

## Code Quality

- **Added explicit standard library includes**: Added `<string>`, `<vector>` to header and `<cstring>` to implementation, removing reliance on transitive includes from ESPHome/NimBLE headers.
