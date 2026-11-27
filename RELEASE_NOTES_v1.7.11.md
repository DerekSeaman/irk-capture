# Release Notes v1.7.11

IRK Capture v1.7.11 fixes a set of wizard, profile and pairing edge cases. There are no new entities, options or Status values.

## What's new for you

- **Going back in the wizard ends the old pairing.** Pressing Back, Stop or Exit while a device is still pairing now disconnects it and discards its result. Before, a device you had backed out of could finish pairing during your next run. It could then show up as the new device's key, take the label you typed for the next device, or turn advertising off when **Stop Advertising After Capture** was on.
- **Stop in the wizard can be retried.** If the device doesn't confirm a stop, for example during a Wi-Fi drop, the wizard says so and pressing Stop sends it again. A wizard tab that another tab has taken over no longer stops the newer run.
- **A new run starts clean.** Starting pairing straight after a failed attempt no longer shows the previous attempt's "Failed" or "No IRK" status for a few seconds.
- **Advertising you turn back on stays on past the capture limit.** Once `max_captures` was reached (or after the first capture with `continuous_mode: false`), advertising turned off again whenever a device disconnected without being captured, or an already-captured device reconnected and resent its key, including in the middle of a wizard run. Only a new device, a changed key or a deliberate re-pair counts now.
- **The wizard label no longer reuses a label another capture already used.** An untouched label field now follows the device, and Next saves only a label you edited.
- **Changing BLE Profile no longer advertises the new profile before the reboot.** The device keeps advertising its current profile until it restarts. A change made in the wizard now also shows in Home Assistant's **BLE Profile** select straight away.
- **Refresh BLE Identity no longer stalls for up to 90 seconds** when a device connects while the refresh is under way. That device is now disconnected so the refresh can finish.
- **IRK Check wording.** `unverified` now reads "could not be checked against the connection address", in the wizard and the README, instead of naming a fixed address as the only cause.

### Behind the scenes

- A device whose encryption failed or timed out no longer has its stale stored key published when it disconnects.
- A connection accepted exactly as the device's millisecond clock wraps (every 49.7 days) keeps its overall pairing timeout.
- A device you backed out of in the wizard no longer delays the next run's advertising by five seconds.
- Bluetooth no longer pauses to close a stray connection or to start or stop advertising. If another Bluetooth operation is under way, that work runs as soon as it finishes.
- The wizard's first step also waits out a reboot or profile change started from the panel.
- A signed-in wizard tab keeps its own login throttle even while other addresses are guessing passwords.

## Upgrading

- **No YAML changes are needed.**
- **ESPHome 2026.9.0 or newer** is still required. Tested with 2026.11.1 on ESP-IDF 5.5.6 and 6.1.1.
- **Rebuild as usual.** Devices that track `main` pick up this version on their next build.
- **A wizard run you back out of is discarded once the ESP32 receives the Stop.** Back, Stop and Exit send a Stop. After the ESP32 processes it, a device from that run that finishes pairing does not have its key sent to Home Assistant; pair it again to capture it. If the wizard can't confirm the Stop, pressing Back, Stop or Exit again resends it.
- **Stop in the wizard stops only the wizard's own run.** After a failure message, Stop has nothing left to stop. Use the **BLE Advertising** switch or the panel's advertising button to turn advertising off.
- **Scripts that call the wizard API:**
  - A new endpoint, `POST /api/capture/stop`, ends a capture. Send `{"operation_id":"<id>","boot_id":"<id>"}` (quoted, both from the `/api/capture/start` reply) to stop that run only, or `{}` to stop whatever is running. The reply is `{"ok":true,"stopped":true}`, or `"stopped":false` when a newer run has replaced the one you named.
  - `/api/status` adds `connection_generation` to `capture_result` and `capture_start`. To follow a run, match `capture_start` to the `operation_id` from the START reply and wait for its `state` to be `ready`. Then accept a `capture_result` only from the same `boot_id` and only when both its `sequence` and its `connection_generation` are newer than `capture_start`'s, comparing with rollover in mind.
  - `/api/status` adds `history_version`. Send it back as `/api/status?history=<value>` and, while the history is unchanged, the reply has `"history_unchanged":true` instead of `history`. Requests without it get the full history as before.
  - `POST /api/advertising` is unchanged.
