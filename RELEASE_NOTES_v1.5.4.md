# Release Notes v1.5.4

## Bug Fixes

- **Fixed BLE advertising not starting on boot**: Resolved an issue where BLE advertising would not automatically start when `start_on_boot: true` was configured. The advertising initialization now correctly waits for the NimBLE host to sync before starting.

- **Fixed rc=21 (BLE_HS_EALREADY) error**: Added defensive handling to prevent advertising start failures when the BLE GAP was in a transitional state.

## Documentation

- Various documentation updates including improved installation instructions, ESP32 variant reference images, and browser compatibility notes for serial flashing.
