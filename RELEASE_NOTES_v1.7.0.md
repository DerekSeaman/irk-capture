# Release Notes v1.7.0

IRK Capture v1.7.0 gives you two ways to capture Bluetooth Identity Resolving Keys: the familiar ESPHome device page in Home Assistant and a new, optional web wizard served directly by your ESP32. This release adds guided pairing, labeled capture results, and clearer information about what the ESP32 is doing.

These notes describe the changes from the released version on `main` to v1.7.0.

## Choose the interface that works for you

| | Home Assistant device page | Standalone web wizard |
| :--- | :--- | :--- |
| Where you use it | The ESP32's ESPHome device page in Home Assistant | A web page served by the ESP32, opened in your browser |
| Setup | Included in the main IRK Capture package | Add the optional wizard package and a username and password |
| Capture workflow | Use the device controls, pair your device, and read the IRK sensor | Choose Express Capture or follow the five-step wizard |
| Results | Shows the latest captured IRK and device BLE MAC | Shows labeled captures from the current session, with Copy buttons |
| Home Assistant required for capture | Yes, when using this interface | No; it also works without internet and from the ESP32's fallback Wi-Fi access point |

Both interfaces use the same capture functionality on the ESP32 and produce IRKs you can use with Home Assistant's **Private BLE Device** integration. You can keep using the Home Assistant device page without installing the wizard.

## New: capture from your browser

The optional wizard package adds a standalone web page on port **8080** by default, alongside the standard ESPHome web interface. It offers two ways to capture:

- **Express Capture:** Work from one screen. Select your device type, add an optional label such as “Derek's iPhone,” and pair with the Bluetooth name shown on the page. Your captured IRK appears in the results with a Copy button.
- **IRK Wizard:** Follow five steps to select your device type, add an optional label, clear an old pairing, pair with the ESP32 using device-specific instructions, and copy the captured IRK into Home Assistant.

The page shows the current Bluetooth name, BLE profile, ESP32 BLE MAC, and capture status. Selecting a device type chooses the appropriate profile; if that requires a reboot, the page shows the restart progress and reconnects automatically.

![IRK Capture Tool with Express Capture and Capture Wizard](docs/wizard-1.jpg)

### Keep several captures organized

Give each capture a label and copy its IRK from the session results. This is useful when capturing several devices in one sitting. Session history is temporary: copy the IRKs you need before rebooting or clearing it. Changing the BLE profile also reboots the ESP32.

The optional package also adds **Stop Advertising After Capture**, available on the web page and in Home Assistant. Enable it when you want the ESP32 to stop advertising after a capture.

The web page includes controls to refresh the Bluetooth identity, clear stored pairings, and reboot the ESP32. **Forget All Bonds** in Express Capture and **Clear Pairings** in the guided wizard let you start fresh. Follow any displayed retry instructions if a device is still connected or Bluetooth is busy. You may also need to forget the ESP32 in the device's own Bluetooth settings.

To enable the web interface, follow [Optional: The Capture Wizard Package](README.md#optional-the-capture-wizard-package). Add `wizard_username` and `wizard_password` to your ESPHome Builder Secrets registry. The page uses HTTP, so use it on a network you trust.

## What's new on the Home Assistant device page

### See capture progress

The new **Status** sensor shows whether the ESP32 is idle, advertising, pairing, capturing, or has captured a key. If a device pairs without sharing an IRK, the status reports `no_irk` and the IRK sensor shows `Failed: IRK not used`.

### Clearer control and sensor names

| Previous name | New name | What it means |
| :--- | :--- | :--- |
| **Generate New MAC** | **Refresh BLE Identity** | Gives the ESP32 a new Bluetooth address and a matching new advertising name |
| **Device MAC** | **Device BLE MAC** | The captured device's Bluetooth address |
| **Effective MAC** | **ESP32 BLE MAC** | The ESP32's own advertised Bluetooth address |

### Make the ESP32 easier to recognize

**Refresh BLE Identity** now changes both the Bluetooth address and name without rebooting. Names such as `IRK HR 7F3A` or `IRK KB 7F3A` identify the active profile and end with the last four characters of the ESP32 BLE MAC. This helps when a device remembers an earlier pairing or discovery entry.

You can also set **BLE Device Name** in either profile, including Keyboard, using a custom name of up to 12 characters. Refreshed and custom names last until the next reboot. The default names remain **IRK Capture** for Heart Sensor and **Logitech K380** for Keyboard.

## Upgrading

- Perform a clean build when upgrading; see [Upgrading to a New Version](README.md#upgrading-to-a-new-version).
- The standalone web wizard is optional and is not enabled by upgrading the main package alone. Add it only if you want the browser interface.
- Check dashboards and automations that use the renamed button or MAC sensors. Home Assistant may leave the old entities unavailable; update references to the new entities and remove obsolete entries as needed.
- Existing captured IRKs do not need to be collected again just to use this release.

## Thanks

Thanks to David Coulson (@davidcoulson) for contributions that helped bring guided capture, clearer status, and Bluetooth identity refresh to this release.
