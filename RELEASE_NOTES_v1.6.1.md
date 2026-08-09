# Release Notes v1.6.1

This release covers the changes from `v1.6.0` to `v1.6.1`.

Capturing Bluetooth IRKs can be finicky — every phone, watch, and OS version
behaves a little differently, and small timing quirks can make a capture fail
for no obvious reason. This release is all about reliability: making pairing and
capture "just work" more often, and smoothing out a few rough edges along the
way.

## More dependable pairing and capture

The biggest focus this release is making captures succeed more consistently:

- **Re-pairing works cleanly.** If you previously removed the IRK Capture device
  from your phone or watch's Bluetooth settings and want to capture again, it now
  pairs fresh and grabs the key as expected — no reboot gymnastics needed.
- **Smarter about showing keys.** The IRK sensor no longer fills up with
  duplicate entries when a device quietly reconnects in the background, but it
  *will* update if a device's key genuinely changes, and it always shows the key
  right after a successful pairing.
- **Gentler on chatty devices.** Some phones reconnect over and over on their
  own. Instead of reacting to every reconnect, the device now briefly pauses to
  break the loop and lets you know in the logs.
- **Steadier overall.** A lot of behind-the-scenes robustness work makes the
  whole pairing process more stable across the wide range of devices out there.

## The "BLE Advertising" switch behaves the way you'd expect

The switch now works as a lasting on/off setting. It stays on between captures
(pausing only for the moment a device is actively connecting) and, when you turn
it off, it stays off until you turn it back on.

## Home Assistant shows a meaningful version

The device previously could display an out-of-date firmware version in Home
Assistant. It now reports the actual ESPHome version it was built with, so what
you see always reflects reality.

## Easier troubleshooting

- The device now logs its Wi-Fi MAC address right at startup. If it ever has
  trouble joining your network, this makes it much easier to spot problems like a
  router that's filtering or blocking the device.
- Logs are cleaner and quieter during normal use, so the messages that matter
  are easier to find.
