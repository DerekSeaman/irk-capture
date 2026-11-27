# Release Notes v1.7.11

IRK Capture v1.7.11 fixes a set of wizard, profile and pairing edge cases. There are no new entities, options or Status values.

## What's new for you

- **Going back in the wizard ends the old pairing.** Pressing Back, Stop or Exit while a device is still pairing now disconnects it and discards its result. Before, a device you had backed out of could finish pairing during your next run. It could then show up as the new device's key, take the label you typed for the next device, or turn advertising off when **Stop Advertising After Capture** was on.
- **Stop in the wizard can be retried.** If the device doesn't confirm a stop, for example during a Wi-Fi drop, the wizard says so and pressing Stop sends it again. A wizard tab that another tab has taken over no longer stops the newer run.
- **A new run starts clean.** Starting pairing straight after a failed attempt no longer shows the previous attempt's "Failed" or "No IRK" status for a few seconds.
- **Advertising you turn back on stays on past the capture limit.** Once `max_captures` was reached (or after the first capture with `continuous_mode: false`), any later disconnect that captured nothing turned advertising off again, including in the middle of a wizard run.
- **The wizard label no longer reuses a label another capture already used.** An untouched label field now follows the device, and Next saves only a label you edited.
- **Changing BLE Profile no longer advertises the new profile before the reboot.** The device keeps advertising its current profile until it restarts. A change made in the wizard now also shows in Home Assistant's **BLE Profile** select straight away.
- **Refresh BLE Identity no longer stalls for up to 90 seconds** when a device connects while the refresh is under way. That device is now disconnected so the refresh can finish.
- **IRK Check wording.** `unverified` now reads "could not be checked against the connection address", in the wizard and the README, instead of naming a fixed address as the only cause.

### Behind the scenes

- A device whose encryption failed or timed out no longer has its stale stored key published when it disconnects.
- A connection accepted exactly as the device's millisecond clock wraps (every 49.7 days) keeps its overall pairing timeout.
- A device you backed out of in the wizard no longer delays the next run's advertising by five seconds.

## Upgrading

- **No YAML changes are needed.**
- **ESPHome 2026.9.0 or newer** is still required. Tested with 2026.11.1 on ESP-IDF 5.5.6 and 6.1.1.
- **Rebuild as usual.** Devices that track `main` pick up this version on their next build.
- **A wizard run you back out of is discarded.** If a device finishes pairing after you leave the pairing step, its key is not sent to Home Assistant. Pair it again to capture it.
- **Stop in the wizard stops only the wizard's own run.** After a failure message, Stop has nothing left to stop. Use the **BLE Advertising** switch or the panel's advertising button to turn advertising off.
- **Scripts that call the wizard API:**
  - A new endpoint, `POST /api/capture/stop`, ends a capture. Send `{"operation_id":"<id>","boot_id":"<id>"}` (quoted, both from the `/api/capture/start` reply) to stop that run only, or `{}` to stop whatever is running. The reply is `{"ok":true,"stopped":true}`, or `"stopped":false` when a newer run has replaced the one you named.
  - `/api/status` adds `connection_generation` to `capture_result` and `capture_start`. A result belongs to a capture start only if its `connection_generation` is newer than the start's.
  - `POST /api/advertising` is unchanged.
