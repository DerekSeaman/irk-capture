# Release Notes v1.5.12

This release note reflects changes from `v1.5.11` to `v1.5.12`.

## Improvement

- **Publish placeholder to Home Assistant for public-address devices**: When a device uses a static public MAC address and has no IRK, the IRK sensor now publishes `"IRK not used"` and the address sensor publishes the fixed MAC address. Previously the sensors would remain stale from a prior capture. This fires once at disconnect time so Home Assistant always reflects the actual outcome of each pairing session.

## Files Updated

- `components/irk_capture/irk_capture.cpp`
- `ESPHome Devices/irk-capture-base.yaml`
- `ESPHome Devices/irk-capture-full.yaml`
- `RELEASE_NOTES_v1.5.12.md`
