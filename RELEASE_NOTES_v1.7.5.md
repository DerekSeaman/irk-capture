# Release Notes v1.7.5

IRK Capture v1.7.5 changes how the firmware is organised, not how it behaves. Captures, the wizard, Status values and Home Assistant entities work as they did in v1.7.4.

## What's new for you

Nothing changes on the device. There are no new entities, options or Status values, and no YAML changes are needed.

## Upgrading

- **Rebuild as usual.** Devices that track `main` pick up this version on their next build.
- **No YAML changes are needed.**

## For developers

- The component's C++ is split by responsibility. `IRKCaptureComponent` is still one class with the same locking; its definition now spans `irk_capture.cpp`, `irk_capture_ble.cpp`, `irk_capture_gap.cpp`, `irk_capture_advertising.cpp`, `irk_capture_identity.cpp`, `irk_capture_session.cpp`, `irk_capture_history.cpp` and `irk_capture_entities.cpp`. Shared includes, constants, helpers and threading notes live in `irk_capture_internal.h`, which lists what each file holds.
- Eleven helpers that moved into the header are now `static inline`, and `persist_ble_profile` has external linkage.
- CI now also validates and compiles the two packages in `ESPHome Devices/` (base and wizard, ESP32-C3), not only the generated CI configuration.
