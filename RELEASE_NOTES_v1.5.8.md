# Release Notes v1.5.8

This release note reflects changes from `v1.5.7` to `v1.5.8`.

## Thread Safety

- **Added timeout to `BleOpGuard` for ESPHome main loop callers**: `BleOpGuard` now accepts an optional timeout parameter (defaulting to `portMAX_DELAY`). All seven call sites in the ESPHome main loop (`loop()`, `start_advertising()`, `stop_advertising()`, `refresh_mac()`, `update_ble_name()`) now use a 200 ms timeout to prevent the FreeRTOS scheduler from stalling the main loop indefinitely if the NimBLE task holds `ble_op_mutex_` for an extended period. NimBLE callback sites retain `portMAX_DELAY` to avoid dropping IRK capture events. A new `acquired()` method allows callers to detect and handle lock-acquisition failure gracefully.

## Correctness

- **Removed inapplicable IEEE 802 unicast mask from BLE MAC generation**: The `& 0xFE` mask applied to `rnd[0]` (LSB of the BLE address) in both `on_ble_host_synced()` and `refresh_mac()` was an IEEE 802 Ethernet concept that does not apply to BLE static random addresses. Per BLE Core Spec Vol 6, Part B §1.3.2, only the two MSBs of the most significant octet must be set to `11` (enforced by `rnd[5] |= 0xC0`). The `& 0xFE` mask has been removed and comments updated to reference the correct spec section.

## Code Cleanup

- **Removed redundant variable assignment in `retry_security_if_needed()`**: The line `sec_init_time_copy = now` immediately after `sec_init_time_ms_ = now` was a no-op — the subsequent `(now - sec_init_time_copy) < SEC_RETRY_DELAY_MS` check evaluates to `0 < delay`, which is always true on the same iteration. Removed the redundant assignment and added a comment explaining why updating `sec_init_time_copy` is unnecessary.

## Files Updated

- `components/irk_capture/irk_capture.cpp`
- `ESPHome Devices/irk-capture-base.yaml`
- `ESPHome Devices/irk-capture-full.yaml`
- `RELEASE_NOTES_v1.5.8.md`
