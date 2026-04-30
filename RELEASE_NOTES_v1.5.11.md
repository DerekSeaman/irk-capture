# Release Notes v1.5.11

This release note reflects changes from `v1.5.10` to `v1.5.11`.

## Bug Fix

- **Fixed build error: `log_no_irk_for_peer` not declared in scope**: The static helper introduced in v1.5.10 was defined after its first call site (`handle_gap_disconnect`), causing a compile error. Moved the definition before `handle_gap_disconnect` to resolve the forward-reference issue.

## Files Updated

- `components/irk_capture/irk_capture.cpp`
- `ESPHome Devices/irk-capture-base.yaml`
- `ESPHome Devices/irk-capture-full.yaml`
- `RELEASE_NOTES_v1.5.11.md`
