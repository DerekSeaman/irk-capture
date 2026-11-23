# Release Notes v1.7.7

IRK Capture v1.7.7 checks every IRK it captures. After a capture, the ESP32 confirms that the IRK generates the private address the device connected from, and reports the result in a new IRK Check sensor. A capture label you type with no usable characters no longer wipes the label you already set, and the wizard can no longer be held up by a client that sends a request very slowly. The ESP32 now reports its manufacturer as "ESP32" instead of "ESPresense".

## What's new for you

- **Each captured IRK is checked.** Phones and watches that use BLE privacy connect from a private address generated from their IRK. After each capture, the ESP32 checks that the captured IRK generated that address. The result is one of:
  - `verified`: the IRK resolves the address the device connected from.
  - `unverified`: the device connected from an address no IRK generates, such as a fixed address, so there was nothing to check. It also means the check couldn't run. A capture with no IRK also reads `unverified`.
  - `mismatch`: the device connected from a private address the IRK does not resolve. This is logged as a warning.

  The result appears in the new **IRK Check** sensor, in the log, in the wizard's result step and history, and in a new `verification` field of the `esphome.irk_captured` event. See [Checking a captured IRK](README.md#checking-a-captured-irk).
- **A clearer message when no IRK is shared.** The log now says whether the device connected from a fixed address or from a private address that changes over time. It suggests using the MAC instead of an IRK only when the address is fixed.
- **An unusable label no longer clears your label.** Capture labels keep letters, digits, spaces, hyphens, underscores and apostrophes. A label made only of other characters, such as an emoji or punctuation, used to be stripped to nothing and silently erase the label you had already saved. Now it is refused and your saved label stays. In the wizard you see an error saying which characters are allowed. In Home Assistant, the Next Capture Label entity switches back to the saved label. Clearing the field on purpose still removes the label.
- **The manufacturer name is now "ESP32".** Phones can read a manufacturer name from a Bluetooth accessory after pairing. IRK Capture reported "ESPresense", left over from the project it grew out of. It now reports "ESP32" in every profile. Capturing works exactly as before, because pairing never uses this name. A phone that already paired may keep showing the old name until it pairs again.

There is one new entity, **IRK Check**. There are no new options or Status values.

### Wizard

- **A slow request can't hold up the wizard.** The wizard handles one request at a time. A signed-in client that sent a request a byte at a time, just fast enough to avoid timing out, could keep the wizard busy for about 40 minutes. Reading a request now stops after 10 seconds, and the wizard answers with a timeout error. The page itself sends small requests that finish well within that.
- **The IRK check is shown.** The result step and the capture history show each capture's IRK check.

### Documentation

- The README has a new **Home Assistant Notifications** section with a sample automation that sends a notification for each capture, including its IRK check result.
- The README explains the IRK check, and its log sample and screenshot now show the IRK Check sensor.

## Upgrading

- **Rebuild as usual.** Devices that track `main` pick up this version on their next build.
- **Using the shipped `irk-capture-base.yaml` package:** no YAML changes are needed. The IRK Check sensor is included.
- **Using your own YAML:** nothing breaks. To get the IRK Check sensor, add `irk_check:` (with a `name`) to the `irk_capture` text sensor, next to `last_irk` and `last_address`. Existing `on_capture` automations keep working, and they can now use `verification` as well.
- **Scripts that call the wizard API:** `POST /api/label` now answers `400` for a non-empty label with no supported characters, instead of `200` with an empty label. An empty string still clears the label and answers `200`. `GET /api/status` and the capture history now include a `verification` field for each capture.

## For developers

- Added `rpa_check.h` (IRK address check), the `irk_check` sensor and a `verification` argument for `on_capture`. `set_next_capture_label()` now returns `LabelUpdateResult`. The rest is internal refactoring and comment fixes with no change in behavior.
