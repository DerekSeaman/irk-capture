# IRK Capture for ESPHome

![Linting Status](https://github.com/DerekSeaman/irk-capture/actions/workflows/lint.yml/badge.svg)

This ESPHome package will capture Apple and Android Bluetooth Identity Resolving Keys (IRK) using an ESP32 running ESPHome. Use the captured IRKs with the [Private BLE Device](https://www.home-assistant.io/integrations/private_ble_device/) integration in Home Assistant for reliable room-level presence detection. I use the [Bermuda BLE Trilateration](https://github.com/agittins/bermuda?tab=readme-ov-file) integration with IRKs for room-level presence detection.

> **⚠️ IMPORTANT:** This package **requires the ESP-IDF framework** and does **NOT** support the Arduino framework. The example YAML configurations include the required `framework: type: esp-idf` setting.

This package uses the ESP-IDF framework for broad ESP32 board compatibility. The ESP-IDF framework is required for ESP32-C2, ESP32-C5, ESP32-C6, ESP32-C61, ESP32-H2, and ESP32-P4 variants, as these newer ESP32 variants are not supported by the Arduino framework.

## What is a BLE IRK and Why Is It Needed?

Modern Apple and Android devices use **BLE privacy features** that randomize their MAC addresses periodically to prevent tracking. This creates a problem for ESPHome Bluetooth Proxy tracking in Home Assistant - your device appears as a different device every time its MAC address changes. This can happen as often as every 15 minutes.

The **Identity Resolving Key (IRK)** is a cryptographic key exchanged during BLE pairing that allows authorized devices to resolve these random MAC addresses back to the original device. By capturing a device's IRK, you can reliably track it for presence detection even as it randomizes its MAC address.

Capturing IRKs from devices can be very tricky, as the Bluetooth stack can very widely among OS versions and device vendors. Some devices may not play well with this package, or need pairing code tweaks to successfully capture the IRK. I have added a lot of debugging code which could help your favorite vibe coding LLM read the debug logs and provide suggested code changes.

Bluetooth uses an 'identity address' that is separate from the readily seen Bluetooth MAC address. The identity address is randomized each time the ESP32 device boots up. This means that even if you use the UI option to change the ESP32 BT MAC, your device may not see your ESP32 as new as the identity address is not changed. It is best to always restart your ESP32 to get a new random identity address and MAC. This should appear as an entirely new Bluetooth device to your phone or watch.

## Blog Post: Track Who's in Each Room with ESPHome + Bermuda BLE

For a complete guide for room-level presence detection using Bermuda BLE Trilateration with Home Assistant, check out my post: [Track Who's in Each Room with ESPHome + Bermuda BLE](https://www.derekseaman.com/2025/12/home-assistant-track-whos-in-each-room-with-esphome-bermuda-ble.html)

## What This Package Does

This IRK capture component turns your ESP32 into a BLE peripheral that **advertises as a heart rate monitor**. When your iOS or Android device pairs with it:

1. The ESP32 presents itself as a standard Bluetooth LE heart rate sensor
2. Your device initiates a secure pairing process
3. During pairing, the device shares its IRK with the ESP32
4. The IRK is captured and exposed as a Home Assistant text sensor
5. You can then use this IRK with the Private BLE Device integration for presence tracking

![ESPHome IRK Capture Device](docs/screenshot-1.jpg)

![ESPHome IRK Capture Logs](docs/screenshot-2.jpg)

## Requirements

- **ESP32 board** with Bluetooth support (any variant: ESP32, ESP32-C3, ESP32-C6, ESP32-S3, etc.)
- **ESP-IDF framework** (required - this component does NOT support Arduino framework)
- **ESPHome** 2024.x or newer
- **Home Assistant** (optional, but recommended for using the captured IRK with Private BLE Device integration)
- **ESPHome Device Builder** (Optional, but makes managing ESPHome devices in Home Assistant easier)

## Installation

### Using ESPHome Device Builder (Recommended)

If you're using the ESPHome Device Builder add-on in Home Assistant, follow these steps:

1. **Create the 'common' directory, if not already present:**
   ```
   /config/esphome/
   ├── common/
   │   └── irk-capture-base.yaml
   └── your-device-name.yaml
   ```

2. **Copy the base configuration:**
   - Download [irk-capture-base.yaml](ESPHome%20Devices/irk-capture-base.yaml) from this repository
   - Place it in `/config/esphome/common/` on your Home Assistant installation

3. **Create your device YAML:**
   - Use [irk-capture-device.yaml](ESPHome%20Devices/irk-capture-device.yaml) as a template
   - Create a new dummy device in ESPHome, and take note of the unique API and OTA keys.
   - Replace the following values in the YAML from this repo:

   ```yaml
   substitutions:
     device_name: esphome-irk-capture          # Change: Unique name for your device (lowercase, hyphens only)
     friendly_name: IRK Capture                # Change: Human-readable name shown in Home Assistant
     api_key: "ZmFrZWFwaWtleWZha2VleGFtcGxlZmFrZWtleQ=="  # Change: Generated with ESPHome new device wizard
     ota_password: "ChangeMe!2025"             # Change: Generated with the ESPHome new device wizard
     esp32_variant: esp32c3                    # Change: Your ESP32 variant (esp32, esp32c3, esp32c6, esp32s3, etc.)
     esp32_board: seeed_xiao_esp32c3           # Change: Your board type (see ESPHome board list)
     ble_name: "IRK Capture"                   # Change: BLE advertising name (shown in Bluetooth settings)

   packages:
     device: !include common/irk-capture-base.yaml
   ```

4. **Secrets File** (managed by ESPHome device builder):
   ```yaml
   wifi_ssid: "Your WiFi Network"
   wifi_password: "your_wifi_password"
   wifi_captive: "fallback_password"
   ```

5. **Flash to your ESP32:**
   - In ESPHome Device Builder, click "Install" and choose your connection method
   - IMPORTANT: After the flashing is complete, either power cycle your ESP32 or do a 'Restart Device' from the ESPHome interface. This will randomize both the BLE MAC address and identity address.

### Using Standalone Configuration

If you prefer a single-file configuration without packages:

1. **Download the full configuration:**
   - Use [irk-capture-full.yaml](ESPHome%20Devices/irk-capture-full.yaml)

2. **Modify the substitutions section** (lines 5-11) with your device-specific values

3. **Configure your secrets.yaml** as shown above

4. **Flash to your ESP32**

## Home Assistant Entities

After flashing and connecting to Home Assistant, the following entities will be available:

| Entity | Type | Description |
|--------|------|-------------|
| **IRK** | Text Sensor | The captured IRK in format `xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx` |
| **Device MAC** | Text Sensor | Bluetooth MAC address of the last paired device |
| **BLE Advertising** | Switch | Turn Bluetooth advertising on/off (starts ON by default) |
| **BLE Device Name** | Text Input | Change the advertised Bluetooth name (default: "IRK Capture") |
| **Generate New MAC** | Button | Generate a new random MAC address for the ESP32 |
| **Restart Device** | Button | Restart the ESP32 - Clears all pairing information |
| **BSSID** | Text Sensor | Wi-Fi access point BSSID (diagnostic) |
| **IP** | Text Sensor | Device IP address (diagnostic) |
| **Uptime** | Sensor | Device uptime in hours (diagnostic) |
| **Internal Temp** | Sensor | ESP32 internal temperature (diagnostic) |
| **Wi-Fi Signal** | Sensor | Wi-Fi signal strength (diagnostic) |

## Usage Instructions

### Capturing an IRK

1. **Enable BLE advertising:**
   - In Home Assistant, open ESPHome and find your IRK Capture device
   - Power on your ESP32 board with the IRK Capture build

2. **Open Bluetooth settings on your device**

3. **Look for the advertised device:**
   - Default name: "IRK Capture" (or whatever you set as `ble_name`)
   - It will appear under available devices

4. **Tap on the device name to pair:**
   - If prompted, tap "Pair" or "Connect"
   - No PIN is required for this pairing
   - Depending on the device/OS, pairing may not complete or show as not connected. This can be normal.

5. **View the captured IRK:**
   - **Option 1:** Check the ESP32 logs in ESPHome Device Builder
   - **Option 2:** View the "IRK" text sensor in Home Assistant (on your IRK Capture device page)
   - The IRK will be in format: `xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx`

6. **Forget the pairing (recommended):**
   - After successfully capturing the IRK, go to your device's Bluetooth settings
   - Forget or unpair the "IRK Capture" device (or whatever name you used)
   - This prevents your device from automatically reconnecting and allows the ESP32 to capture IRKs from other devices
   - If you need to capture IRKs from multiple devices, I suggest a 'Restart Device' between each capture to avoid potential issues

### Installing Private BLE Device Integration

1. **Go to Home Assistant:**
   - Navigate to Settings → Devices & Services

2. **Add the Private BLE Device integration:**
   - Click "+ ADD INTEGRATION"
   - Search for "Private BLE Device"
   - Click to add

3. **Enter the captured IRK:**
   - Paste the complete IRK string
   - Example: `a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6`

4. **Complete setup:**
   - Give the device a friendly name (e.g., "Derek's iPhone")
   - The device will now be tracked for presence detection

## Tested Devices

This IRK capture component has been successfully tested with:

- **Apple OS 26 family:**
  - iPhone
  - Apple Watch
  - iPad

- **Android devices:**
  - Jailbroken Amazon Echo Show 5 with LineageOS 18.1

## Troubleshooting Tips

### Device Not Appearing in Bluetooth Settings

- **Turn off Bluetooth** on your device
- Press the **"Restart Device"** button to reset the ESP32's BLE identity information
- Ensure the **"BLE Advertising"** switch is ON
- Turn your device's Bluetooth back on and connect to the ESP32

### IRK Not Captured After Pairing

- After pairing, **forget/unpair the BLE device** from your device's Bluetooth settings
- Turn Bluetooth OFF on your device
- Press the **"Restart Device"** button on the ESPHome device page
- Turn Bluetooth ON on your device
- Try pairing again from scratch
- Power cycle your device, restart your ESP32, and try pairing again

### ESPHome Build Fails

- Clean the build folder and retry
- Ensure you're using ESPHome 2024.x or newer (tested with ESPHome 2025.12)
- Verify your `esp32_variant` and `esp32_board` substitutions match your hardware
- Check that all required secrets are defined in `secrets.yaml`

## Credits

Based on [ESPresense](https://github.com/ESPresense/ESPresense) enrollment functionality.

Original package: [github://KyleTeal/irk-capture/irk-capture-package.yaml@main](https://github.com/KyleTeal/irk-capture)

## License

MIT License - See LICENSE file for details
