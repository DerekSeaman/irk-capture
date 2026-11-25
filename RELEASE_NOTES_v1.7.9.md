# Release Notes v1.7.9

IRK Capture v1.7.9 is a reliability release. It makes Refresh BLE Identity ride out a busy Bluetooth stack, frees the connection slot sooner when a device pairs without handing over its key, and tidies several wizard rough edges. There are no new entities, options or Status values.

## What's new for you

- **Refresh BLE Identity rides out a busy moment.** A refresh first closes any connected device. If Bluetooth was briefly too busy to close it, the refresh used to give up. It now waits a second and tries again, up to five times. A second refresh pressed while the first is still waiting gets the same five attempts.
- **A device that pairs without its key is let go at once.** Some devices finish pairing a few seconds late and then turn out not to share their identity key. The "Failed: IRK not used" result was already shown, but the connection stayed open for up to 90 seconds and then reported a pairing timeout that never happened. The ESP32 now closes that connection right away, so the next device can connect.
- **A reconnecting device can't be mistaken for a new capture.** If one device disconnected and an already paired device connected in the same instant, the second device's key could be reported as a fresh capture: firing `on_capture`, using the next-capture label and, with Stop After Capture on, stopping advertising. The ESP32 now checks that the connection is still the one it was reading.
- **Capture labels lose stray spaces.** Spaces before or after a label are now removed, so `"  Kitchen  "` is saved as `Kitchen`, and leading spaces no longer use up the length limit. A label made only of spaces is refused and the current label is kept, the same as a label with no allowed characters. An empty label still clears it. This applies to the Next Capture Label entity and the wizard alike.
- **A clearer log line.** If the ESP32 cannot start the background task it uses for Bluetooth cleanup, the log now spells out what won't work: Forget All Bonds, clearing old pairings during Refresh BLE Identity, and quick disconnects. It also tells you to restart the ESP32.

### Wizard

- **No more false "Advertising is off".** Right after pressing Next: Start Pairing, the wizard could flash "Advertising is off. Go back a step…" for about a second, even though advertising had just started. The wizard now gets the current state from the device every time.
- **Buttons and toggles update straight away.** The panel's Start/Stop Advertising button and the Stop After Capture toggle now show their new state as soon as you press them, instead of a moment later.
- **Each new wizard run starts with an empty name.** After you finished a capture and pressed Done, opening the wizard again kept the previous device's name in the "Whose device is this?" box, ready to be attached to the next device. The box now starts empty. A name you set in the panel still fills it in.
- **A name you clear stays cleared.** Emptying the name box no longer brings the old name back a second later.
- **You see the name as it was saved.** If the device drops characters it doesn't allow, the box now shows the name as it was saved.
- **Tapping Next twice is harmless.** A double tap on Next in the name step saves once. A late reply can no longer pull you back from pairing to the previous step and stop the capture.
- **A refused Start no longer sends a Stop.** If the device turns down Next: Start Pairing, for example after a wrong password or too many attempts, the wizard used to follow up with a request to stop advertising. That extra request could use up another sign-in attempt and replace the real error message. The wizard now sends it only when it can't tell whether the start went through.
- **Copying the key works better over a home network.** The wizard is usually opened over plain `http://`, where browsers limit copying. Copy now works more reliably there. On iPhone, it no longer scrolls the page or briefly brings up the keyboard.

## Upgrading

- **Rebuild as usual.** Devices that track `main` pick up this version on their next build.
- **Using the shipped packages:** no YAML changes are needed.
- **Using your own YAML:** no changes are needed. Existing `on_capture` automations, entities and Home Assistant events work as before.
- **ESPHome 2026.9.0 or newer.** IRK Capture is now tested with ESPHome 2026.9.0 and newer, up from 2026.7.4. If your ESPHome is older, update it before rebuilding. 2026.9.0 is also the first release that accepts the README example's encrypted OTA (`encryption:` under `ota:`).
- **Scripts that call the wizard API:** no changes. Endpoints and responses are the same.
