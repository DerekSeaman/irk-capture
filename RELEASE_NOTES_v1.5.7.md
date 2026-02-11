# Release Notes v1.5.7

## Stability and Safety Improvements

- **Serialized BLE control operations across tasks**: Added a dedicated BLE operation mutex (`ble_op_mutex_`) with RAII guard (`BleOpGuard`) to serialize sensitive NimBLE calls from mixed contexts (ESPHome main loop and NimBLE callbacks), including advertising start/stop, device name updates, MAC rotation, and connection termination paths.
- **Advertising switch state now reflects reality**: The advertising switch now publishes the actual runtime advertising state after start/stop attempts instead of echoing only the requested state.
- **Hardened NVS initialization and failure handling**:
  - Added strict return-code checks for `nvs_flash_init()`, erase/re-init paths, NVS health-check writes/commits, and cleanup operations.
  - Removed destructive fallback behavior that erased the full NVS partition on test-namespace open failure.
  - Component now fails cleanly (`mark_failed`) when NVS is not usable.
- **Added defensive return-code checks for core BLE/NimBLE setup**:
  - `ble_store_config_init()`
  - boot-time `ble_store_clear()`
  - `ble_svc_gap_device_name_set()`
  - additional terminate/start/stop paths now log failures consistently.
- **Setup flow hardening**: `setup()` now exits early if `setup_ble()` marked the component failed, preventing entity/UI initialization on failed BLE startup.

## Code Quality and Maintainability

- **Removed dead code**:
  - unused helper `read_peer_bond_by_conn()`
  - unused constant `IRK_MIN_POLL_INTERVAL_MS`
  - unused local `adv_success`
- **Replaced magic GAP event numbers with named constants** for readability and safer maintenance across SDK variants.
- **Made getters thread-safe**: `get_ble_profile()` and `get_ble_name()` now acquire `state_mutex_` before reading shared state.

## Files Updated

- `components/irk_capture/irk_capture.cpp`
- `components/irk_capture/irk_capture.h`
- `RELEASE_NOTES_v1.5.7.md` (new)
- `RELEASE_NOTES_v1.5.6.md` (removed)
