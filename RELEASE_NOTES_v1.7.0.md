# Release Notes v1.7.0

This release covers everything since `v1.6.2`. It is the biggest update to IRK Capture
so far: capturing several phones and watches in one sitting is now much smoother, pairing
is more reliable, and Home Assistant shows you what the device is doing at each step.

## What's new

### A Status sensor that shows where a capture is

A new **Status** sensor tells you what IRK Capture is doing right now: `advertising`
(ready for a device to pair), `pairing`, `capturing`, `captured`, or `no_irk` if the
device paired but didn't share its IRK. You no longer need to watch the logs or several
entities at once to know whether a capture worked.

### Forget All Bonds button

A new **Forget All Bonds** button clears the current session's capture list so you can
start fresh, without rebooting or changing the ESP32's Bluetooth address. For safety it
only clears stored pairings when nothing is connected; if a device is connected, press it
again once it disconnects.

### Capture as many devices as you like in one session

Pairing a fourth device in the same session could fail, because the Bluetooth stack only
has room for three stored pairings. IRK Capture now makes room automatically, so you can
keep capturing without rebooting between devices. IRKs you already captured are not
affected.

## Improvements

- **More reliable pairing.** Some phones and watches could drop the connection partway
  through pairing because a few Bluetooth details the ESP32 offered didn't behave the way
  they expected. Those are fixed, which should mean fewer failed pairing attempts.
- **Clear result when a device doesn't share its IRK.** If a device pairs but doesn't
  provide an IRK, the IRK sensor now shows `Failed: IRK not used` along with the device's
  MAC address, instead of leaving you guessing. The logs suggest using that MAC address
  directly in that case.
- **Your latest capture stays on screen.** A phone you paired earlier can reconnect in the
  background. It no longer replaces the IRK and Device MAC of the device you just captured
  while you're copying them.
- **Advertising recovers on its own.** If Bluetooth advertising fails to start, IRK Capture
  keeps retrying in the background instead of staying silently off. It never reboots or
  loses your captures to do this.
- **Stuck pairings are cleaned up.** A pairing attempt that stalls is now ended cleanly so
  you can simply try again. An IRK captured just before the cleanup is still kept.
- **Safer BLE Profile changes.** If a new profile can't be saved, IRK Capture keeps the
  current profile and logs an error instead of rebooting into the wrong one.
- **BLE Device Name fixes.** Re-entering the current name no longer disconnects anything,
  and Home Assistant no longer shows a name the device rejected.
- **Clearer logs.** Pairing failure reasons are now reported accurately.

## Upgrading

- Perform a clean build when upgrading (see **Upgrading to a New Version** in the README).
- ESPHome 2026.7 or newer is required; this release is tested with ESPHome 2026.9.0.
- Advertising startup: the shared package now lets `irk_capture.start_on_boot` decide
  whether advertising starts at boot, which matches how most people already use it. Only
  if your own YAML sets the BLE Advertising switch's `restore_mode` to `ALWAYS_ON` will
  advertising now always start ON, even with `start_on_boot: false`.

## Thanks to David Coulson

Many of this release's new features came from David Coulson (@davidcoulson), who built them
to support a guided, multi-device capture flow.

- Added the Status sensor and Forget All Bonds button, with session state tracked so a
  front end can follow each capture without parsing logs.
- Added the component's internal groundwork for guided flows: an option to stop advertising
  after a capture, labels for the next captured device, and capture history served over
  HTTP rather than as a Home Assistant entity, which has a 255-character limit.
- Made it possible to point a device at a different component branch or fork via
  `irk_component_url` / `irk_component_ref` substitutions, which is what makes testing the
  `dev` branch possible.
- Moved reboots that could be requested from Bluetooth or web server tasks onto ESPHome's
  main loop, and extended CI to compile-check all of the new entities.
