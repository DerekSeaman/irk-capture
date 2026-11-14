# Release Notes v1.7.4

IRK Capture v1.7.4 makes failures visible. A pairing that goes wrong, a Refresh BLE Identity that doesn't take effect, or a Bluetooth stack that never starts now tells you so, instead of quietly returning to "advertising" or "idle". The wizard is harder to lock yourself out of, and it explains itself better when something is wrong.

## What's new for you

### Status can say "failed"

The **Status** sensor has a new value, `failed`. It appears when a pairing ends without a result, for example when the phone's encryption fails or no IRK arrives within 45 seconds. It stays on screen for about four seconds, like `captured` and `no_irk`. The **IRK** sensor keeps the last good result.

If you have automations that match on Status values, add `failed` to them.

### Failed identity refresh is reported

When **Refresh BLE Identity** can't change the name and address, the wizard now says so. Before, it looked as if it had worked. The message appears once per attempt, and it doesn't come back after you reload the page.

### Bluetooth that won't start is reported

If the Bluetooth host hasn't synchronised after 30 seconds, the device logs an error and Status shows `failed`. Earlier, the device just sat there idle with no explanation. Reboot the device; if the problem keeps coming back, check the Bluetooth configuration in the build.

### Wizard login protects each device on its own network

Five wrong passwords used to lock out every browser for 30 seconds, and anyone on the network could repeat that. Failed logins are now counted per client address, so a bad password only locks out the client that typed it.

### Clearer wizard errors

- A pairing that did not complete shows "Pairing did not complete" with the next step, rather than "Waiting for your device".
- If the device sends a reply the page can't read, the page says so. It no longer reports the device as unreachable.
- A bug in the page shows an error message and is logged to the browser console, instead of looking like a dropped connection.
- If the device's Bluetooth capture failed to start, the controls say so, and the reboot control still works.

### Devices captured after the limit stay in the history

Re-enabling advertising after the limit is reached used to push earlier devices out of the history list. The history now keeps at least 32 devices.

### Safer behaviour under Bluetooth contention

- Disconnecting from the main loop no longer waits indefinitely for the Bluetooth lock. If the lock is busy, the link closes when the pairing timeout runs out. The device stays responsive.
- Repeated attempts to stop advertising back off instead of spamming the log.

## Upgrading

- **Clean build.** Follow [Upgrading to a New Version](README.md#upgrading-to-a-new-version).
- **`max_captures` must be 64 or less.** Higher values now fail the build with a message.
- **Wizard username and password must use printable ASCII.** Browsers encode other characters differently for the login, so such a login could never work. The build now says so.
- **Check automations** that depend on Status values. Add `failed` where you need it.
- **No other YAML changes are needed.**

## For developers

- `mutex_guard.h` is now shared by `irk_capture` and `irk_wizard`.
- Build for all three boards with `esphome compile` against the latest ESPHome before tagging a release.
- The wizard's JSON reader walks the request object, so a key name inside a string value is no longer mistaken for a member. Escapes are decoded, and invalid input is rejected rather than guessed at.
- `GET /api/status` has a new `identity_refresh` object (`operation_id`, `state`), and `capture_start`/`bond_clear` keep their names.
- The status response is sent in chunks. A failed send closes the connection rather than leaving a truncated document.
- Login throttling uses a `ClientThrottles` table keyed on the client address (IPv4, and IPv6 when enabled).
- BLE setup failures after `nimble_port_init()` now deinitialise the stack, and NVS recovery no longer erases the partition. A failed setup stops cleanly instead of leaving the controller allocated.
- Bonds still live in RAM only, and nothing is erased at boot beyond the bond clear that already ran.
