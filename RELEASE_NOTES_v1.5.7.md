# Release Notes v1.5.7

This release note reflects changes from `v1.5.6` to `v1.5.7`.

## Boot Stability and GATT Reliability

- **Fixed Keyboard profile boot crash**: The Keyboard BLE profile was causing GATT registration to fail at boot because NimBLE rejects HID services with no characteristics. Added a minimal `HID Protocol Mode` characteristic (`0x2A4E`) to make the HID service structurally valid.
- **Added Keyboard→Heart Sensor GATT fallback**: If Keyboard GATT registration fails, the component now automatically falls back to the Heart Sensor profile instead of hard-failing. The fallback is persisted to NVS so subsequent boots avoid the same failure path.
- **Graceful no-GATT continuation**: If all GATT profile registrations fail, the component continues operating without custom GATT services rather than calling `mark_failed()`, preventing unnecessary boot failures.
- **Relaxed non-fatal boot checks**: NVS health-check failures and `ble_store_config_init()` failures are now non-fatal warnings rather than hard stops, improving resilience on flash-stressed hardware.
- **Setup flow hardening**: `setup()` exits early if `setup_ble()` marks the component failed, preventing entity/UI initialization on a broken BLE stack.
- **Explicitly zero GATT handle globals** before registration so handle state is predictable regardless of which profile or fallback path is taken.

## Thread Safety and Concurrency

- **Serialized BLE control operations**: Added a dedicated BLE operation mutex (`ble_op_mutex_`) with RAII guard (`BleOpGuard`) to serialize sensitive NimBLE calls (advertising start/stop, device name updates, MAC rotation, connection termination) across the ESPHome main loop and NimBLE callback contexts.
- **Fixed `sec_retry_done_` race condition**: The security retry flag is now written under mutex *before* calling `ble_gap_security_initiate()`, closing a window where a NimBLE callback could observe the stale `false` value and trigger a double retry.
- **Fixed `irk_last_try_ms_` first-poll bypass**: The IRK poll interval guard was bypassed on the first poll because the timestamp initialized to 0. It is now set to `now_ms()` alongside `enc_ready_` when encryption completes.
- **Fixed `mac_rotation_retries_` and `mac_rotation_ready_time_` unprotected writes**: These members are now reset inside existing `MutexGuard` blocks, consistent with the documented threading model.
- **Made getters thread-safe**: `get_ble_profile()` and `get_ble_name()` now acquire `state_mutex_` before reading shared state.

## Correctness Fixes

- **Fixed duplicate Home Assistant state events on auto-stop**: `publish_and_log_irk()` now checks `is_advertising()` before calling `stop_advertising()`, preventing a double call and double `publish_state(false)` when the disconnect handler has already stopped advertising.
- **Fixed IRK re-publish deduplication bypass**: The reconnect path in `on_connect()` previously called `publish_irk_to_sensors()` directly, bypassing the 60-second rate limit, deduplication cache, and capture counter. It now routes through `publish_and_log_irk()`.
- **Fixed BLE name length silent truncation**: The `name_len` field is now clamped to 29 bytes with a warning log before the `uint8_t` cast.
- **Fixed `commit_err` misleading log**: NVS set and commit errors are now tracked with separate variables so the log accurately reports which operation failed.
- **Advertising switch state now reflects reality**: The advertising switch publishes the actual runtime state after start/stop attempts rather than echoing only the requested state.

## Observability and Diagnostics

- **Diagnostic log on spurious double-disconnect**: `on_disconnect()` now logs at DEBUG level if called while already disconnected, making NimBLE duplicate events visible in serial output.
- **Diagnostic log on L2CAP parameter update acceptance**: Logs at DEBUG level when a peer-requested connection parameter update is accepted.
- **Diagnostic log on Keyboard→Heart Sensor fallback**: Logs handle values after fallback registration to clarify GATT state in diagnostics.
- **Hardened NVS initialization logging**: Added strict return-code checks and logging for `nvs_flash_init()`, erase/re-init paths, and NVS health-check operations.
- **Consistent failure logging**: `ble_store_config_init()`, boot-time `ble_store_clear()`, `ble_svc_gap_device_name_set()`, and all terminate/start/stop paths now log failures consistently.

## Code Quality

- **Removed dead code**: Unused helper `read_peer_bond_by_conn()`, unused constant `IRK_MIN_POLL_INTERVAL_MS`, unused local `adv_success`.
- **Replaced magic GAP event numbers with named constants** for readability and safer maintenance across ESP-IDF/NimBLE SDK variants.

## Files Updated

- `components/irk_capture/irk_capture.cpp`
- `components/irk_capture/irk_capture.h`
- `ESPHome Devices/irk-capture-base.yaml`
- `ESPHome Devices/irk-capture-full.yaml`
- `RELEASE_NOTES_v1.5.7.md`
