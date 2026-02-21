# Release Notes v1.5.8

This release note reflects changes from `v1.5.7` to `v1.5.8`.

## Bug Fix

- **Fixed spurious encryption timeout on first connect in `retry_security_if_needed()`**: When `sec_init_time_ms_` was 0 (first connection since boot/reset), the local snapshot `sec_init_time_copy` was left as 0 after setting `sec_init_time_ms_ = now`. The subsequent timeout check `(now - sec_init_time_copy) > SEC_TIMEOUT_MS` evaluated as `(uptime_ms - 0)`, which immediately exceeded `SEC_TIMEOUT_MS` (20 000 ms) on any device that had been running longer than 20 seconds. This caused every first pairing attempt to be aborted with a spurious "Encryption timeout" disconnect before the peer could respond. Fixed by also updating `sec_init_time_copy = now` after initializing `sec_init_time_ms_`.

## Files Updated

- `components/irk_capture/irk_capture.cpp`
- `ESPHome Devices/irk-capture-base.yaml`
- `ESPHome Devices/irk-capture-full.yaml`
- `RELEASE_NOTES_v1.5.8.md`
