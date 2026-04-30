# Release Notes v1.5.10

This release note reflects changes from `v1.5.9` to `v1.5.10`.

## Improvement

- **Improved logging for public-address devices (e.g. Garmin watches)**: When a device uses a static public MAC address and has no IRK to distribute, the firmware now emits a clear INFO message explaining the situation and displays the fixed MAC address to use directly as the stable identifier.

## Files Updated

- `components/irk_capture/irk_capture.cpp`
- `ESPHome Devices/irk-capture-base.yaml`
- `ESPHome Devices/irk-capture-full.yaml`
- `RELEASE_NOTES_v1.5.10.md`
