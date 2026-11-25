# Release Notes v1.7.9

IRK Capture v1.7.9 is a reliability release. There are no new entities, options or Status values.

## What's new for you

- **More dependable Bluetooth.** Refresh BLE Identity and the end of each pairing now retry when Bluetooth is briefly busy, instead of giving up or holding the connection open for up to 90 seconds. Advertising also comes back after Forget All Bonds if Bluetooth was busy at the time.
- **Faster turnaround between devices.** A device that pairs without sharing its key is let go right away, so the next device can connect.
- **No false captures.** A device that reconnects at the same moment another one leaves can no longer be reported as a new capture.
- **Tidier capture labels.** Spaces around a label are removed, and a label made only of spaces is refused.
- **Clearer log messages** when the ESP32 needs a restart to recover.

### Wizard

- **Status and buttons stay current.** No more brief false "Advertising is off", and controls show their new state straight away.
- **The device name box behaves.** It starts empty for each new device, stays cleared when you clear it, shows the name as saved, and keeps what you typed even on a slow network.
- **Steadier navigation.** Double taps, Back and slow replies no longer skip steps or show stale errors.
- **Copying the key works better** over a home network, including on iPhone.
- **Lighter on the ESP32** while the wizard is open, so it competes less with Bluetooth during pairing.
- **Small security improvements** to how the wizard page and sign-in are protected.

### Behind the scenes

- Code comments cleaned up.
- More automated tests and build checks, including ESP32-S3.

## Upgrading

- **ESPHome 2026.9.0 or newer is required** (up from 2026.7.4). Update ESPHome before rebuilding if yours is older.
- **No YAML changes are needed**, whether you use the shipped packages or your own YAML. Existing `on_capture` automations, entities and Home Assistant events work as before.
- **Rebuild as usual.** Devices that track `main` pick up this version on their next build.
- **Scripts that call the wizard API:** no changes. Endpoints and responses are the same.
