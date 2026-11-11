# Release Notes v1.7.3

IRK Capture v1.7.3 is a maintenance release that follows a full code review. Failed captures now finish sooner, **Refresh BLE Identity** is more dependable, BLE names are handled more carefully, and the wizard is harder to misuse.

## What's fixed

### "No IRK" results finish right away

When a device paired but did not provide an IRK, as some Garmin watches do, the ESP32 kept the connection open for up to 90 seconds. No other device could pair during that time, and the log then reported a pairing timeout that had not really happened. The connection now closes as soon as the result is shown. Advertising pauses for 5 seconds afterwards so the same device cannot reconnect in a loop.

### Refresh BLE Identity retries when Bluetooth is busy

If the Bluetooth controller was briefly busy, a new address was abandoned on the first failure and the old address stayed in use. The ESP32 now retries up to 10 times, waiting a little longer each time.

### BLE Device Name is no longer reset by a bad entry

- An empty name, or one made only of unsupported characters, is now rejected and the current name is kept. Previously the name was reset to "IRK Capture", which also changed the address and cleared pairings.
- Leading spaces no longer use up the 12-character limit.
- A name of exactly 12 characters no longer logs a truncation warning.
- The **BLE Device Name** field in Home Assistant now stops at 13 characters instead of accepting more and trimming it.

### Next Capture Label clears after use

The label was applied to the next captured device but stayed on display, which suggested it would be applied again. The entity now clears once the label is used.

### Unlimited mode keeps more history

With `max_captures: 0`, only 10 devices were kept, so the 11th capture removed the first from the list. Unlimited mode now keeps the 32 most recent devices.

### Wizard

- The wizard page can no longer be embedded inside another web page.
- The page stops polling the ESP32 while its browser tab is hidden and refreshes when you return to it.
- The wizard server restarts itself if it fails to start completely.

## Under the hood

- The ESP32 does less work while it holds its internal lock during pairing, and the wizard reads its status in one step.
- A flash write that ran on every boot is gone.
- Paired device keys are kept in memory only, so none survive a reboot. This was already the behavior and is now set explicitly.
- Internal cleanup and corrected code comments, with no change to how captures work.

## Upgrading

- Perform a clean build; see [Upgrading to a New Version](README.md#upgrading-to-a-new-version).
- Check `ble_name` in your YAML. It may contain only letters, digits, spaces, hyphens and underscores, and cannot start or end with a space. A name with other characters, such as an apostrophe, used to be changed silently and now stops the build with a message.
- If you use the wizard on a custom port, `port` must be 32767 or lower. The default of 8080 is unaffected.
