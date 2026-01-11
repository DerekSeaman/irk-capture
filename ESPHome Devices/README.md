# ESPHome Device Configuration Files

This directory contains the ESPHome YAML configuration files for the IRK Capture component.

## Files

### irk-capture-base.yaml

Generic ESP32 IRK Capture Package

This is the base package configuration that provides IRK capture functionality for any ESP32 device. It contains all the core configuration but requires the device-specific YAML to provide substitutions for hardware details.

**Usage:**

- Copy this file to `/config/esphome/common/` on your Home Assistant installation
- Reference it from your device YAML using `packages: !include common/irk-capture-base.yaml`

**Required substitutions:**

- `device_name` - Unique device name (lowercase, hyphens only)
- `friendly_name` - Human-readable name shown in Home Assistant
- `api_key` - ESPHome API encryption key
- `ota_password` - OTA update password
- `esp32_variant` - Your ESP32 variant (esp32, esp32c3, esp32c6, esp32s3, etc.)
- `esp32_board` - Your board type (see ESPHome board list)
- `ble_name` - BLE advertising name shown in Bluetooth settings

---

### irk-capture-device.yaml

Example Device Configuration

This is a minimal example showing how to use the base package with the ESPHome Device Builder with a specific ESP32 device. It demonstrates the package-based approach where device-specific settings are kept separate from the core functionality.

**Usage:**

- Use this as a template for your own device configuration
- Replace all substitution values with your device-specific settings
- Tip: Create a dummy device in ESPHome Device Builder, grab the API/OTA keys, then use this YAML template

---

### irk-capture-full.yaml

Standalone Complete Configuration

This is a single-file configuration that merges the base package and device settings into one file. Use this if you prefer not to use the package system or want a simple all-in-one configuration.

**Usage:**

- Download this file and modify the substitutions section (lines 5-12)
- No need to copy the base package file
- Create a new device in ESPHome Device Builder, and replace all of the YAML with these contents. Use the unique API/OTA keys generated for the device.

---

### secrets.yaml

Example Secrets File

This is an example secrets file showing the required Wi-Fi credentials. ESPHome Device Builder typically manages this file automatically. Make sure these substituations are in the secrets file.

**Required secrets:**

- `wifi_ssid` - Your Wi-Fi network name
- `wifi_password` - Your Wi-Fi password
- `wifi_captive` - Fallback AP password

---

## Which Configuration Should I Use

### Use the Package Approach (base + device) if

- You have multiple ESP32 devices running IRK Capture
- You want to maintain consistency across devices
- You prefer separation between base functionality and device settings
- You're comfortable with the ESPHome package system

### Use the Standalone Configuration (full) if

- You have a single ESP32 device
- You prefer everything in one file
- You want simplicity over modularity
- You're new to ESPHome packages

---

## Directory Structure for Package Approach

When using the package approach, your ESPHome directory should look like this:

```text
/config/esphome/
├── common/
│   └── irk-capture-base.yaml
└── my-irk-capture.yaml (your device file)
```

## Directory Structure for Standalone Approach

When using the standalone approach:

```text
/config/esphome/
└── irk-capture-full.yaml (renamed to your preference)
```
