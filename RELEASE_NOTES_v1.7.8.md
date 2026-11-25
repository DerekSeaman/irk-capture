# Release Notes v1.7.8

IRK Capture v1.7.8 is a reliability release. It makes Refresh BLE Identity more dependable, stops a failed connection attempt from disturbing a capture in progress, and makes the wizard's error messages reach your browser. There are no new entities, options or Status values.

## What's new for you

- **Refresh BLE Identity is more dependable.** Refreshing the identity clears the old pairings and changes the Bluetooth address. That cleanup now runs where the Bluetooth stack expects it, so it can no longer collide with a phone that is connecting at the same moment. If a phone connects while a refresh is under way, the refresh waits for that connection to close. A hiccup while closing that connection no longer cancels your refresh.
- **A failed connection attempt can't disturb a capture.** Sometimes a connection attempt fails but the link stays up for a moment. Encryption on that stray link is now ignored. Before, it could be mistaken for a real pairing, which risked a wrong capture result, a pairing removed by mistake, or advertising held off for a few seconds.
- **Your saved settings are in place before the first connection.** After a restart, the saved Stop After Capture setting is now restored before Bluetooth starts accepting connections. A phone that connected within the first moments after boot could, in rare cases, be handled with the wrong setting.
- **A clearer log line.** If the ESP32 cannot run the cleanup step of Refresh BLE Identity, the log now says so and tells you to restart the ESP32. Before, the problem showed only in the wizard.

### Wizard

- **You see the real error.** If Bluetooth capture failed to start, the wizard is meant to tell you to check the device log and reboot. In some browsers that message was lost, and the page showed only a generic connection failure. The wizard now reads your request first, so the message arrives.
- **Rejected requests are closed cleanly.** A request that is refused for a wrong password, a lockout, a wrong content type, a request that is too large or one that is too slow now ends its connection, and the wizard tells the browser so.
- **A harder-to-guess password check.** The wizard's sign-in check now takes the same time whatever you type, including a password of the wrong length. Before, the time it took could hint at how long the real password was.
- **The Stop After Capture toggle works from the keyboard.** The toggle can now be reached with the Tab key and switched with Space or Enter, and screen readers announce it as an on/off switch. It looks the same as before.

## Upgrading

- **Rebuild as usual.** Devices that track `main` pick up this version on their next build.
- **Using the shipped packages:** no YAML changes are needed.
- **Using your own YAML:** no changes are needed. Existing `on_capture` automations, entities and Home Assistant events work as before.
- **Scripts that call the wizard API:** no changes. Endpoints and responses are the same, except that a rejected request may now close its connection.
