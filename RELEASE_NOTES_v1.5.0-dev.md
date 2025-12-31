# IRK Capture v1.5.0 Release Notes

## Overview

v1.5.0 is a major update focused on **Android device support** and **reliability improvements**. This release introduces a new BLE profile system that enables IRK capture from Samsung Galaxy phones and other Android devices that previously needed workarounds.

---

## New Features

### BLE Profile Selector

A new dropdown in the ESPHome device page lets you switch between two BLE advertising profiles:

| Profile | Best For | Advertised Name | Service |
| ------- | -------- | --------------- | ------- |
| **Heart Sensor** | Apple devices (iPhone, iPad, Apple Watch) | Your configured name | Heart Rate (0x180D) |
| **Keyboard** | Android devices (Samsung Galaxy, Pixel, etc.) | "Logitech K380" | HID (0x1812) |

**Why this matters:** Samsung One UI 7 and other Android versions aggressively filter BLE devices in Bluetooth settings. The Keyboard profile advertises as a familiar Logitech keyboard, bypassing these filters and making the ESP32 visible for pairing.

**Usage:**

1. Select the profile matching your target device
2. The ESP32 will automatically reboot to apply the new GATT services
3. Wait ~30 seconds for the device to be ready (watch for the Effective MAC sensor to update)

### Effective MAC Sensor

A new text sensor displays the current BLE MAC address being advertised by the ESP32. This helps you:

- Verify MAC rotation is working after pressing "Generate New MAC"
- Confirm the device is ready after a profile change or reboot
- Troubleshoot pairing issues by confirming the advertised address

---

## Improvements

### Thread Safety Overhaul

The entire codebase has been refactored for production-grade thread safety on ESP32 single-core devices:

- Added FreeRTOS mutex protection for all shared state between the NimBLE task and ESPHome main task
- Implemented RAII `MutexGuard` class for exception-safe lock/unlock
- Eliminated race conditions that could cause intermittent pairing failures

### Non-Blocking MAC Rotation

The "Generate New MAC" button now uses an event-driven state machine instead of blocking delays:

- Instant button response (no UI freeze)
- Reliable MAC changes even during active connections
- Automatic retry logic for transient errors

### Bond Table Management

- Bond table is now cleared on every boot, ensuring a fresh pairing session
- Eliminates "phantom" pairing failures from stale bond data
- Improves privacy by not persisting old IRKs across reboots

### Memory Safety

- IRK cache now has a hard cap of 10 entries with FIFO eviction
- Pre-allocated vector capacity prevents heap fragmentation on ESP32-C3
- In-place updates for duplicate IRKs (no memory bloat from reconnecting devices)

### BLE Name Validation

- Runtime sanitization of BLE device names from Home Assistant
- Enforces 12-character limit for Samsung compatibility
- Automatic fallback to "IRK Capture" for invalid input

### NimBLE Stack Optimization

- Increased NimBLE task stack size from 4KB to 5KB for logging safety
- Prevents stack overflow during debug logging in GAP event callbacks

---

## Bug Fixes

- **Fixed race condition in IRK cache** causing potential heap corruption during rapid reconnections
- **Fixed deadlock** when duplicate IRKs triggered advertising stop
- **Fixed zombie connection state** where the UI showed "connected" but no data flowed
- **Fixed heap fragmentation** on extended capture sessions with pre-allocated vectors
- **Fixed pairing timeout handling** with proper cooldown to prevent rapid-fire reconnection loops

---

## Breaking Changes

None. All existing YAML configurations work without modification.

### Behavioral Changes

1. **Bond table cleared on boot** - Devices must re-pair after ESP32 reboot. This is intentional for reliability.
2. **Profile change triggers reboot** - Switching between Heart Sensor and Keyboard profiles requires a reboot to update GATT services (NimBLE limitation).

---

## Tested Devices

| Device | OS | Profile | Status |
| ------ | -- | ------- | ------ |
| iPhone | iOS 26 | Heart Sensor | Working |
| Apple Watch | watchOS 26 | Heart Sensor | Working |
| iPad | iPadOS 26 | Heart Sensor | Working |
| Samsung Galaxy S25+ | One UI 7 | Keyboard | Working |
| Samsung Galaxy Watch8 Classic | Wear OS 6 | Both | Not Working |
| Google Pixel 9 | Android 15 | Keyboard | Working |
| Amazon Echo Show | LineageOS 18.1 | Heart Sensor | Working |

---

## Migration Guide

### From v1.4.x

Add the new optional entities to your configuration (if desired):

```yaml
select:
  - platform: irk_capture
    irk_capture_id: irk
    ble_profile:
      id: ble_profile_select
      name: "BLE Profile"

text_sensor:
  - platform: irk_capture
    irk_capture_id: irk
    effective_mac:
      name: "Effective MAC"
```

Then compile, flash, and power cycle your ESP32.
