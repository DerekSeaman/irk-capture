# Release Notes v1.6.2

This release covers the changes from `v1.6.1` to `v1.6.2`.

## Fixes a build failure on recent ESPHome versions

ESPHome 2026.9.0 changed how it assembles Bluetooth support during a build,
which could cause the IRK Capture build to fail with an error mentioning a
missing `ble_gap.h` file. If you hit that error while building or rebuilding
your device, this release fixes it — no changes needed on your end beyond
building again.

Thanks to community member David Coulson for tracking this down and
contributing the fix.
