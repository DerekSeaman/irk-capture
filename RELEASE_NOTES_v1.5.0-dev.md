# IRK Capture v1.5.0-dev Release Notes

## Overview

v1.5.0-dev is a major stability and robustness release focusing on thread safety, memory management, and improved device compatibility. This release addresses critical concurrency issues and adds several quality-of-life improvements.

---

## What's New

### Thread Safety Overhaul

- Added FreeRTOS mutex protection for all shared state between NimBLE and ESPHome tasks
- Implemented RAII `MutexGuard` class for exception-safe locking
- Eliminates race conditions that could cause crashes or undefined behavior on ESP32

### Non-Blocking MAC Rotation

- Refactored MAC address rotation to use an event-driven state machine
- Eliminates blocking delays that could trigger watchdog resets
- MAC rotation now happens seamlessly without blocking the main loop

### Bond Table Management

- Bond table is now cleared on every boot for a clean slate
- Prevents bond table capacity issues and stale pairing data
- Improves privacy by not persisting old IRKs across reboots

### BLE Advertising Improvements

- Faster advertising interval (100-150ms) for quicker device discovery
- Uses explicit static random address type for better compatibility
- Optimized advertising profile with single Heart Rate service UUID

### Input Validation

- BLE device name is now sanitized and limited to 12 characters
- Only alphanumeric characters, spaces, hyphens, and underscores allowed
- Invalid input falls back to default "IRK Capture" name

### Configuration Defaults

- `continuous_mode` now defaults to `true` (keeps advertising after capture)
- `max_captures` now defaults to `10` (auto-stops after 10 IRKs)

### Single Capture Mode Fix

- Fixed bug where single capture mode (`continuous_mode: false`) didn't stop advertising after first IRK

---

## Bug Fixes

- **Fixed race conditions** in IRK cache deduplication, timer handlers, and connection state
- **Fixed deadlock** in deduplication path when stopping advertising
- **Fixed MAC address byte ordering** for correct static random address generation
- **Fixed infinite loop** in MAC rotation when host not ready
- **Fixed zombie connection state** when NimBLE stack becomes wedged
- **Fixed heap fragmentation** by pre-allocating IRK cache capacity
- **Fixed stack overflow risk** by increasing NimBLE task stack to 5120 bytes

---

## Technical Changes

- Increased `CONFIG_BT_NIMBLE_TASK_STACK_SIZE` from 4096 to 5120 bytes
- Added 500ms settling delay before MAC rotation to ensure BLE stack is idle
- Added 90-second global pairing timeout with 5-second cooldown
- Added zombie connection protection with forced state reset on termination failure
- IRK cache limited to 10 entries with FIFO eviction

---

## Documentation

- Added comprehensive troubleshooting section with DEBUG logging details
- Added Samsung/Android BLE visibility workarounds
- Added sample log output for successful IRK capture
- Clarified ESP-IDF framework requirement
- Updated tested devices list

---

## Breaking Changes

None. All changes are backward compatible with existing YAML configurations.

### Behavioral Changes

1. **Bond table cleared on boot** - Devices must re-pair after ESP32 reboot
2. **Default configuration changed** - Now captures multiple devices by default

---

## Tested Configurations

- ESP32-C3 / ESP32-C6 + ESPHome 2025.12.2 + ESP-IDF framework
- iPhone (iOS 26)
- Apple Watch (watchOS 26)
- iPad (iPadOS 26)
- Samsung Galaxy S25+
- Amazon Echo Show 5 & 8 Gen 1 (LineageOS 18.1)
