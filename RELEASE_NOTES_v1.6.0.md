# Release Notes v1.6.0

This release note reflects changes from `v1.5.12` to `v1.6.0`.

## What's New

- **Much simpler installation.** ESPHome Device Builder now generates most of your device config automatically — you just add a small `packages:` block to pull in IRK Capture. No more manually copying WiFi, API, or OTA settings into a template.
- **Optional board-specific extras for Seeed XIAO boards.** Separate companion packages for the C3, C5, C6, and S3 add status LED support, antenna switching, and (on C5) dual-band WiFi support. See the linked blog post for details.

## Removed

- The standalone all-in-one config, the old local-package example, and the example secrets file are gone — the new Device Builder + `packages:` method replaces all of them.

## Changed

- Pairing now uses the same settings as v1.5.12 (Secure Connections enabled). The Galaxy Watch Wear OS 5 workaround from v1.5.13–v1.5.15 has been reverted; those versions were never widely released.

## Files Updated

- `components/irk_capture/irk_capture.cpp`
- `ESPHome Devices/irk-capture-base.yaml`
- `ESPHome Devices/irk-capture-device-remote.yaml`
- `README.md`
- `RELEASE_NOTES_v1.6.0.md`
