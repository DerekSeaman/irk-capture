# Release Notes v1.5.13

This release note reflects changes from `v1.5.12` to `v1.5.13`.

## Bug Fix

- **Fixed pairing failure with Galaxy Watch (Wear OS 5) and similar Android/Wear OS devices**: Two related bugs caused a duplicate Security Request PDU to be sent to the peer mid-handshake, triggering SMP error 0x07 "Command Not Supported" on devices that initiate pairing themselves.
  - `BLE_HS_EALREADY` (rc=2) returned from the initial `ble_gap_security_initiate()` call was not treated the same as `BLE_HS_EBUSY` (rc=3). Both mean the peer is already handling security, but only `EBUSY` suppressed the 2-second retry. `EALREADY` now also suppresses the retry.
  - The 2-second retry handler always logged at `WARN` level and sent an unconditional Security Request regardless of whether security was already in progress. It now checks the return code and logs `EALREADY`/`EBUSY` at `DEBUG` level with an explanatory message.
  - Added a specific hint log when status=7 occurs to guide troubleshooting.

## Files Updated

- `components/irk_capture/irk_capture.cpp`
- `ESPHome Devices/irk-capture-base.yaml`
- `ESPHome Devices/irk-capture-full.yaml`
- `RELEASE_NOTES_v1.5.13.md`
