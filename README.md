# IRK Capture for ESPHome

![Linting Status](https://github.com/DerekSeaman/irk-capture/actions/workflows/lint.yml/badge.svg) [![Buy Me a Coffee](https://img.shields.io/badge/Buy%20Me%20a%20Coffee-donate-yellow?logo=buy-me-a-coffee&logoColor=white)](https://buymeacoffee.com/vdereks)

This ESPHome package will capture Apple and Android Bluetooth Identity Resolving Keys (IRK) using an ESP32 running ESPHome. Use the captured IRKs with the [Private BLE Device](https://www.home-assistant.io/integrations/private_ble_device/) integration in Home Assistant for reliable room-level presence detection. I use the [Bermuda BLE Trilateration](https://github.com/agittins/bermuda?tab=readme-ov-file) integration with IRKs for room-level presence detection.

This ESPHome IRK capture package is only designed to capture IRKs and can NOT pull double duty as a Bluetooth proxy. You can either flash this to a spare ESP32 device and keep it in a sock drawer when not being used, or temporarily flash this package to an ESP32 then flash back to your generic Bluetooth proxy ESPHome configuration. IRKs are generally permanent and do not change over time.

## What is a BLE IRK and Why Is It Needed?

Modern Apple and Android devices use **BLE privacy features** that randomize their MAC addresses periodically to prevent tracking. This creates a problem for ESPHome Bluetooth Proxy tracking in Home Assistant - your device appears as a different device every time its MAC address changes. This can happen as often as every 15 minutes.

The **Identity Resolving Key (IRK)** is a cryptographic key exchanged during BLE pairing that allows authorized devices to resolve these random MAC addresses back to the original device. By capturing a device's IRK, you can reliably track it for presence detection even as it randomizes its MAC address.

Capturing IRKs from devices can be very tricky, as the Bluetooth stack can very widely among OS versions and device vendors. Some devices may not play well with this package, or need pairing code tweaks to successfully capture the IRK. I have added a lot of debugging code which could help your favorite vibe coding LLM read the debug logs and provide suggested code changes.

The ESP32 uses a **random static address** for BLE advertising, which is regenerated each time the device boots. This address serves as both the advertised MAC address and the identity address for pairing. The "Generate New MAC" button also changes this address. However, if your phone or watch has previously paired with the ESP32, it may still have cached bond information. To ensure your device sees the ESP32 as completely new, either restart the ESP32 or use "Generate New MAC", and then **forget the pairing** on your phone/watch before attempting to pair again.

## Track Who's in Each Room with ESPHome + Bermuda BLE

For a complete guide for room-level presence detection using Bermuda BLE Trilateration with Home Assistant, check out my post: [Track Who's in Each Room with ESPHome + Bermuda BLE](https://www.derekseaman.com/2025/12/home-assistant-track-whos-in-each-room-with-esphome-bermuda-ble.html)

## What This Package Does

This IRK capture component turns your ESP32 into a BLE peripheral that can emulate different device types to capture IRKs from various platforms. It supports two BLE profiles:

- **Heart Sensor Profile** (for Apple devices, Android watches): Advertises as a heart rate monitor, which Apple devices and many Android watches can discover (with third party app)
- **Keyboard Profile** (for Android phones): Advertises as a "Logitech K380" keyboard, which bypasses Samsung's aggressive BLE filtering on Galaxy phones

When your Apple or Android device pairs with the ESP32:

1. The ESP32 presents itself as the selected BLE device type (heart rate sensor or keyboard)
2. Your device initiates a secure pairing process
3. During pairing, the device shares its IRK with the ESP32
4. The IRK is captured and exposed as a Home Assistant text sensor
5. You can then use this IRK with the Private BLE Device integration for presence tracking

![ESPHome IRK Capture Device](docs/screenshot-1.jpg)

## Requirements

- **ESP32 board** with Bluetooth support (any variant: ESP32, ESP32-C3, ESP32-C6, ESP32-S3, etc.)
- **ESP-IDF framework** (required - this component does NOT support Arduino framework)
- **ESPHome** 2026.7 or newer - Tested with 2026.9.0
- **Home Assistant** (optional, but recommended for using the captured IRK with Private BLE Device integration)
- **ESPHome Device Builder** (optional, but makes managing ESPHome devices in Home Assistant easier)

## Installation Instructions

I’ve written a detailed blog post that covers the installation and usage of ESPHome Device Builder. It shows you how to build a device profile for your ESP32 and capture your device IRKs. You can find it here:
[How-To: Using my ESPHome Bluetooth IRK Capture Package](https://www.derekseaman.com/2026/01/how-to-using-my-bluetooth-irk-capture-package.html)

The super abbreviated installation instructions are as follows:

- Build a new ESPHome device specific to your ESP32 board
- Add the shown **packages:** section at the bottom
- Connect your ESP32 device and flash it

ESPHome Device Builder pulls the package and component from the `main` branch, so clean builds always get the latest version. My blog post includes optional Seeed Studio XIAO S3, C3, C5 and C6 device profile enhancements.

![Device YAML Configuration](docs/YAML-screenshot.jpg)

Here's the same thing as plain, copyable YAML — a generic ESPHome Device
Builder device with the **packages:** section added at the bottom (replace the
placeholder key/passwords with your own device's values):

```yaml
# Board: Seeed XIAO ESP32C3 (Seeed Studio)
# Definition: definitions/boards/seeed-xiao-esp32c3/manifest.yaml

esphome:
  name: test-irk
  friendly_name: test IRK

esp32:
  variant: esp32c3
  flash_size: 4MB
  framework:
    type: esp-idf

logger:

api:
  encryption:
    key: "UkVQTEFDRS1XSVRILVlPVVItT1dOLTMyQi1LRVkhISE="

ota:
  - platform: esphome
    encryption:

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  ap:
    ssid: test Fallback Hotspot
    password: "CHANGE_ME_AP_PASSWORD"

captive_portal:

packages:
  device:
    url: https://github.com/DerekSeaman/irk-capture
    ref: main
    file: ESPHome Devices/irk-capture-base.yaml
    refresh: always
```

You can find the **packages:** content here: [irk-capture-device-remote.yaml](https://github.com/DerekSeaman/irk-capture/blob/main/ESPHome%20Devices/irk-capture-device-remote.yaml)

If you're using a Seeed Studio XIAO ESP32 board, add the board-specific hardware
and diagnostics package as a second `packages:` entry alongside the one above:

- **XIAO ESP32-C3**: [repo](https://github.com/DerekSeaman/ESPHome-Seeed-Xiao-ESP32-c3-Config) — [Seeed XIAO ESP32-c3 IRK.yaml](https://github.com/DerekSeaman/ESPHome-Seeed-Xiao-ESP32-c3-Config/blob/main/examples/Seeed%20XIAO%20ESP32-c3%20IRK.yaml)
- **XIAO ESP32-C5**: [repo](https://github.com/DerekSeaman/ESPHome-Seeed-Xiao-ESP32-c5-Config) — [Seeed XIAO ESP32-c5 IRK.yaml](https://github.com/DerekSeaman/ESPHome-Seeed-Xiao-ESP32-c5-Config/blob/main/examples/Seeed%20XIAO%20ESP32-c5%20IRK.yaml)
- **XIAO ESP32-C6**: [repo](https://github.com/DerekSeaman/ESPHome-Seeed-Xiao-ESP32-C6-Config) — [Seeed XIAO ESP32-c6 IRK.yaml](https://github.com/DerekSeaman/ESPHome-Seeed-Xiao-ESP32-C6-Config/blob/main/examples/Seeed%20XIAO%20ESP32-c6%20IRK.yaml)
- **XIAO ESP32-S3**: [repo](https://github.com/DerekSeaman/ESPHome-Seeed-Xiao-ESP32-s3-Config) — [Seeed XIAO ESP32-s3 IRK.yaml](https://github.com/DerekSeaman/ESPHome-Seeed-Xiao-ESP32-s3-Config/blob/main/examples/Seeed%20XIAO%20ESP32-s3%20IRK.yaml)

### Full Example: Seeed XIAO ESP32-C3

Here's a complete device YAML for the Seeed XIAO ESP32-C3, combining the
generic device config above with **both** `packages:` entries — the main
IRK Capture package and the C3-specific hardware/diagnostics package
(replace the placeholder key/passwords with your own device's values):

```yaml
# Board: Seeed XIAO ESP32C3 (Seeed Studio)
# Definition: definitions/boards/seeed-xiao-esp32c3/manifest.yaml

esphome:
  name: test-irk
  friendly_name: test IRK

esp32:
  variant: esp32c3
  flash_size: 4MB
  framework:
    type: esp-idf

logger:

api:
  encryption:
    key: "UkVQTEFDRS1XSVRILVlPVVItT1dOLTMyQi1LRVkhISE="

ota:
  - platform: esphome
    encryption:

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  ap:
    ssid: test Fallback Hotspot
    password: "CHANGE_ME_AP_PASSWORD"

captive_portal:

packages:
  device:
    url: https://github.com/DerekSeaman/irk-capture
    ref: main
    file: ESPHome Devices/irk-capture-base.yaml
    refresh: always
  c3_hardware:
    url: https://github.com/DerekSeaman/ESPHome-Seeed-Xiao-ESP32-c3-Config
    ref: main
    file: examples/Seeed XIAO ESP32-c3 IRK.yaml
    refresh: always
```

## Usage Instructions

Again, my blog post covers usage in detail. However, the super short version is as follows:

- In Home Assistant go to Settings > ESPHome -> Your ESP32 IRK Capture Device
- Select the appropriate BLE profile (Heart Sensor for Apple devices and Android watches, Keyboard for Android phones)
- Pair your phone or watch with the advertising ESP32 device name
- Watch the Sensors IRK value and it should display the captured IRK
- Paste the captured IRK into the Private BLE Device integration in Home Assistant

## Home Assistant Entities

After flashing and connecting to Home Assistant, the following entities will be available:

| Entity | Type | Description |
| :--- | :--- | :--- |
| **BLE Advertising** | Switch | Keep Bluetooth advertising enabled between connections (starts ON by default) |
| **BLE Device Name** | Text Input | Change the Heart Sensor profile name (default: "IRK Capture"); Keyboard is fixed to "Logitech K380" |
| **BLE Profile** | Select | Choose BLE advertising profile: "Heart Sensor" (Apple) or "Keyboard" (Android). Changing profiles triggers a reboot. |
| **Generate New MAC** | Button | Generate a new random MAC address for the ESP32 |
| **Device MAC** | Text Sensor | Bluetooth MAC address of the last paired device |
| **Effective MAC** | Text Sensor | Current BLE MAC address being advertised by the ESP32 |
| **IRK** | Text Sensor | Latest completed pairing result: a captured IRK or `Failed: IRK not used` |
| **Status** | Text Sensor | Current session state: `idle`, `advertising`, `pairing`, `capturing`, `captured`, or `no_irk` |
| **Forget All Bonds** | Button | Clears this session's capture list so a run can start from a clean slate, and clears stored bonds when no device is connected |
| **Restart Device** | Button | Restart the ESP32 - Clears all pairing information |
| **BSSID** | Text Sensor | Wi-Fi access point BSSID (diagnostic) |
| **Internal Temp** | Sensor | ESP32 internal temperature (diagnostic) |
| **IP** | Text Sensor | Device IP address (diagnostic) |
| **MAC** | Text Sensor | ESP32 Wi-Fi MAC address (diagnostic) |
| **SSID** | Text Sensor | Connected Wi-Fi network name (diagnostic) |
| **Uptime** | Sensor | Device uptime (diagnostic) |
| **Wi-Fi Disconnects (since boot)** | Sensor | Number of Wi-Fi disconnections since boot (diagnostic) |
| **Wi-Fi RSSI** | Sensor | Wi-Fi signal strength in dBm (diagnostic) |

The advertising switch represents the requested state and stays ON while a device is connected.
An explicit switch `restore_mode` controls its startup state; omitting it or using `DISABLED`
delegates startup to `irk_capture.start_on_boot`. The shared package uses `DISABLED`, so
existing `start_on_boot: false` configurations still start with advertising OFF.
If you explicitly set `ALWAYS_ON`, advertising starts ON even when `start_on_boot` is false.

If advertising or random-address setup fails, retry delays increase through 1, 2, 4, and 8 seconds.
After the fifth failure, automatic recovery continues once per minute while the switch stays ON.
An error marks the transition to slow recovery; continuing failures are logged at most once every
five minutes. Successful recovery is also logged. Turning the switch OFF cancels retries;
turning it back ON starts a fresh attempt. Recovery does not reboot or clear captures/bonds.

### Status and building a capture flow on top of it

**Status** reports where a capture attempt is, which is otherwise only inferable by watching
several entities change at once:

| State | Meaning |
| :--- | :--- |
| `idle` | No active capture work or advertising; a completed connection may still be closing |
| `advertising` | Advertising, nothing connected |
| `pairing` | A peer is connected, security is not yet complete |
| `capturing` | Encrypted, reading the identity key from the bond store |
| `captured` | A key was just captured (or the device was deliberately re-paired) |
| `no_irk` | Pairing completed but the peer sent no identity key - see the IRK sensor |

`captured` and `no_irk` are held briefly so a polling UI cannot miss them between updates,
and remain visible while that completed connection is still open. A newer completed result
replaces the previous status. A bonded device reconnecting and republishing the same key does
not re-trigger `captured`.

**Forget All Bonds** clears this session's capture list, and also clears stored bonds when no
device is connected. Generate New MAC already clears the bond store as part of rotating the
address, so reach for this one when you want a clean capture list without changing the address.

For a guided, multi-device flow built on these entities, see the optional wizard package.

## Tested Devices

This ESPHome IRK capture component has been successfully tested with:

- **Apple OS 26 and 27 family:**
  - iPhone 17 Pro
  - Apple Watch Ultra 3
  - iPad Pro M5

- **Android devices:**
  - Samsung Galaxy S25+
  - Samsung Galaxy Watch7
  - Samsung Galaxy Watch (Wear OS 5)
  - Google Pixel 9
  - Jailbroken Amazon Echo Show 5 with LineageOS 18.1

## Troubleshooting Tips

### The Provided IRK does not match any BLE devices that Home Assistant can see

This is most common on some Android devices and happens when you input the captured IRK into the Private BLE Device field. This happens because Home Assistant can’t see the corresponding BLE device that matches the IRK. Some Android devices only broadcast BLE beacons very infrequently. This means Home Assistant may not have recently seen the BLE device, thus it can’t match the IRK. Unfortunately the solution to this is very device and OS specific, and may not be solvable. I suggest Googling your device and see if any settings can be changed to increase the frequency of the BLE advertising.

### ESP32 Device Name Not Appearing in Bluetooth Settings

- **Turn off Bluetooth** on your device
- Press the **"Restart Device"** button to reset the ESP32's BLE stack
- Ensure the **"BLE Advertising"** switch is ON
- Turn your device's Bluetooth back on and connect to the ESP32
- If ESP32 device does not appear on Android, see the section below

### Android Phone can't see ESP32 Device Name

Samsung One UI 7 (Galaxy S25, S24, etc.) aggressively filters BLE devices in Bluetooth settings. To restore visibility:

1. **Enable Developer Options**: Settings → About Phone → Software Information → Tap "Build Number" 7 times
2. **Enable BLE visibility**: Settings → Developer Options → Scroll down and enable **"Show unsupported Bluetooth LE devices in Bluetooth settings"**
3. Return to Bluetooth settings and scan again — the ESP32 device should now appear
4. Tap on the device (e.g., "Logitech K380" when using Keyboard profile) and tap pair. The IRK should appear in the ESP32 logs and ESPHome device page in Home Assistant

### Android Phone still not visible after Developer Options fix

If the Developer Options fix doesn't work, or you're on a non-Samsung Android device with similar filtering, try using the nRF Connect app:

1. Install **nRF Connect** from the Play Store (by Nordic Semiconductor)
2. Open the app and tap "Scan"
3. Look for "Logitech K380" in the device list (when using Keyboard profile)
4. Tap on it to connect
5. The pairing dialog should appear, allowing the bonding process to complete
6. Look for the captured IRK in the ESP32 logs or the ESPHome device page

### GrapheneOS / Hardened Android — "Incorrect PIN or passkey" Error

GrapheneOS (and some other hardened Android builds) enforces **mandatory authenticated pairing** for HID Keyboard devices. Because a Bluetooth keyboard could theoretically inject keystrokes, GrapheneOS requires a PIN or passkey confirmation before completing the bond. IRK Capture uses "Just Works" pairing (no PIN), so GrapheneOS rejects the Keyboard profile pairing and shows "Incorrect PIN or passkey" on the phone.

**Symptoms:**

- Phone shows "Incorrect PIN or passkey" during pairing
- An IRK may appear in the logs and ESPHome device page, but pasting it into the Private BLE Device integration reports "The provided IRK does not match any BLE devices that Home Assistant can see"
- After rotating the ESP32 MAC address and retrying, the connection fails with `ENC_CHANGE status=1035`

The captured IRK is invalid for use in Home Assistant because GrapheneOS considers the bond incomplete and may immediately rotate to a new RPA key state, or the IRK is from a pairing that was aborted before HA's passive scanner could see the device advertising with it.

**Solution: Switch to the Heart Sensor profile.**

GrapheneOS does not enforce MITM-authenticated pairing for heart rate sensors, so Just Works pairing succeeds cleanly and the captured IRK will work correctly.

1. On the ESPHome device page in Home Assistant, change **BLE Profile** to **"Heart Sensor"**
2. Wait ~30 seconds for the ESP32 to reboot and begin advertising
3. On your GrapheneOS phone, open Bluetooth settings and pair to the heart rate sensor device
4. The IRK will be captured and published — use it in the Private BLE Device integration as normal

### Android Watches

Many Android watches aggressively filter BLE devices, and by default neither the keyboard or heart sensor will be shown as a pairable device. However, the app "Gear Tracker II" (no affiliation) overcomes this aggressive BLE filtering and should allow you pair your watch to the ESP32 and extract the IRK.

Watches that require "reverse" pairing (i.e. the watch advertises as a device that needs to be paired with) will NOT work with this package. This package requires your watch pair TO the ESP32, not the other way around.

### IRK Not Captured After Pairing

Some devices, including some Garmin watches, use a fixed BLE address and do not provide an IRK. When a completed pairing stores a bond without an IRK, the Home Assistant IRK sensor shows **`Failed: IRK not used`**, and **Device MAC** identifies that peer. This replaces the previous sensor value so it reflects the latest completed attempt; prior values remain in Home Assistant's recorded history. A later successful capture replaces the failure with the 32-character IRK, including when a previously captured device reconnects.

Missing bond records, incomplete pairing, and invalid key bytes are not reported as "IRK not used." A public identity address alone does not establish whether a device uses BLE privacy.

- After pairing, **forget/unpair the BLE device** from your device's Bluetooth settings
- Turn Bluetooth OFF on your device
- Modify the BLE Device Name on the ESPHome device page
- Turn Bluetooth ON on your device
- Try pairing to the ESP32 again
- If that still fails, power cycle your phone/watch/tablet, power cycle your ESP32, change the BLE Device Name, and try pairing again

### Upgrading to a New Version

When upgrading IRK Capture to a new version, always perform a clean build to ensure all component changes are fully compiled:

1. In ESPHome Device Builder, open your IRK Capture device
2. Click the three-dot menu (⋮) in the lower right and select **"Clean Build Files"**
3. After the clean completes, click **"Install"** to rebuild and flash

Skipping the clean step can result in stale cached object files being linked against the new component source, which may cause unexpected behavior even if the flash appears to succeed.

### ESPHome Log Sample

Below is a sample log showing a successful IRK capture:

```text
[16:15:01.812][D][switch:065]: 'BLE Advertising': Sending state ON
[16:15:01.814][D][irk_capture:1690]: Advertising with profile: Heart Sensor
[16:15:01.861][D][sensor:135]: 'Wi‑Fi RSSI': Sending state -30.00000 dBm with 0 decimals of accuracy
[16:15:16.669][I][irk_capture:866][nimble_host]: Connection established successfully
[16:15:16.669][I][irk_capture:1832][nimble_host]: Conn start: handle=0 enc_ready=0 was_adv=1
[16:15:16.669][I][irk_capture:1835][nimble_host]: Connected; handle=0, initiating security
[16:15:16.669][I][irk_capture:362][nimble_host]: sec: enc=0 bonded=0 auth=0 key_size=0
[16:15:16.669][I][irk_capture:366][nimble_host]: peer ota=4A:1B:2C:3D:4E:5F type=1
[16:15:16.669][I][irk_capture:368][nimble_host]: peer id =A1:B2:C3:D4:E5:F6 type=0
[16:15:16.669][D][irk_capture:372][nimble_host]: conn params: interval=24 latency=0 supervision_timeout=500
[16:15:16.672][D][irk_capture:376][nimble_host]: role=slave our_ota=C0:FF:EE:12:34:56
[16:15:18.724][I][irk_capture:2105]: Retrying security initiate after 2050 ms
[16:15:18.727][D][irk_capture:2107]: Retry security initiate rc=2
[16:15:19.117][D][irk_capture:1458][nimble_host]: Pairing complete: handle=0
[16:15:19.121][D][irk_capture:1195][nimble_host]: Peer identity resolved using IRK
[16:15:19.121][I][irk_capture:988][nimble_host]: ENC_CHANGE status=0 (0x00)
[16:15:19.121][I][irk_capture:1045][nimble_host]: Encryption established; attempting immediate IRK capture
[16:15:19.121][I][irk_capture:522][nimble_host]: Pairing completed; publishing IRK again
[16:15:19.121][I][irk_capture:425][nimble_host]:
[16:15:19.121][I][irk_capture:428][nimble_host]: *** IRK CAPTURED *** (REPAIR)
[16:15:19.121][I][irk_capture:607][nimble_host]: Identity Address: A1:B2:C3:D4:E5:F6
[16:15:19.121][I][irk_capture:608][nimble_host]: IRK: a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6
[16:15:19.121][I][irk_capture:609][nimble_host]: Capture events this session: 2
[16:15:19.121][I][irk_capture:610][nimble_host]: Unique devices this session: 1
[16:15:19.121][I][irk_capture:425][nimble_host]:
[16:15:19.211][I][irk_capture:893][nimble_host]: Disconnect reason=534 (0x216)
[16:15:19.211][I][irk_capture:1932][nimble_host]: Disconnected
```

## Credits

Based on [ESPresense](https://github.com/ESPresense/ESPresense) enrollment functionality.

Original package: [github://KyleTeal/irk-capture/irk-capture-package.yaml@main](https://github.com/KyleTeal/irk-capture)

## License

MIT License - See LICENSE file for details
