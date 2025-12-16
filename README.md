# IRK Capture for ESPHome

Capture iOS and Android BLE Identity Resolving Keys (IRK) using an ESP32 running ESPHome. Use the captured IRKs with [Private BLE Device](https://www.home-assistant.io/integrations/private_ble_device/) integration in Home Assistant for reliable presence detection.

## How It Works

The ESP32 advertises as a Bluetooth heart rate monitor, which iOS and Android will pair with. During the pairing process, the device's IRK is exchanged and captured. This IRK can then be used to track the device even when it uses randomized MAC addresses.

## Requirements

- ESP32 board (any variant with Bluetooth)
- ESPHome 2024.x or newer
- Home Assistant (optional, for integration)

## Installation

1. Copy `irk-capture.yaml` to your ESPHome config directory

2. Create a `secrets.yaml` file in your ESPHome config directory:
   ```yaml
   wifi_ssid: "Your WiFi"
   wifi_password: "your_password"
   ap_pass: "fallback_ap_password"
   encrypt_key: "your-32-byte-base64-key=="
   ota_pass: "ota_password"
   ```

3. Flash to your ESP32:
   ```bash
   esphome run irk-capture.yaml
   ```


## Usage

1. Open your phone's Bluetooth settings
2. Look for the device (default name: "JBL Tune")
3. Tap to pair - accept the pairing request
4. The IRK will be captured and shown in Home Assistant

## Home Assistant Entities

| Entity | Type | Description |
|--------|------|-------------|
| Last Captured IRK | Text Sensor | The captured IRK (format: `irk:xxxx...`) |
| Last Device Address | Text Sensor | MAC address of the paired device |
| BLE Advertising | Switch | Enable/disable Bluetooth advertising |
| Generate New MAC | Button | Generate a new random MAC address |
| BLE Device Name | Text | Change the advertised Bluetooth name |

## Configuration Options

To change the BLE device name, add this to your `irk-capture.yaml`:

```yaml
irk_capture:
  ble_name: "Beats Solo4"  # Name shown in Bluetooth settings
```

To change the ESP32 board type:

```yaml
esp32:
  board: esp32-s3-devkitc-1  # For ESP32-S3
```

## Using Captured IRKs

1. Copy the IRK from the "Last Captured IRK" sensor (e.g., `irk:xxxxxxxxxxxxxxxxxxxxxxxxx`)
2. Go to Home Assistant → Settings → Devices & Services
3. Add the "Private BLE Device" integration
4. Paste the IRK

## Home Assistant Automation

Get notified when an IRK is captured:

```yaml
automation:
  - alias: "IRK Captured Notification"
    trigger:
      - platform: state
        entity_id: sensor.last_captured_irk
    condition:
      - condition: template
        value_template: "{{ trigger.to_state.state not in ['', 'unknown', 'unavailable'] }}"
    action:
      - service: persistent_notification.create
        data:
          title: "🔑 IRK Captured!"
          message: |
            IRK: {{ states('sensor.last_captured_irk') }}
            Address: {{ states('sensor.last_device_address') }}
```


**Build Fails:**
- Clean the build folder and retry 

**Device not visible in Bluetooth settings:**
- Press "Generate New MAC" button
- Make sure advertising switch is ON

**"IRK does not match" in Home Assistant:**
- Keep your phone's Bluetooth active while adding
- The phone must be advertising BLE


## Credits

Based on [ESPresense](https://github.com/ESPresense/ESPresense) enrollment functionality.
