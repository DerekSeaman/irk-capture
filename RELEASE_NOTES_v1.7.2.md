# Release Notes v1.7.2

IRK Capture v1.7.2 is a reliability release. It fixes a case where advertising could stay off, keeps the ESP32 BLE MAC sensor current, and stops capture log lines from going missing.

## What's fixed

### Advertising no longer gets stuck off

When a paired device keeps reconnecting, the ESP32 pauses advertising for 5 seconds to break the loop. In a rare case the pause never ended, and advertising stayed off while the **BLE Advertising** switch still showed ON. Advertising now always resumes after the pause.

### ESP32 BLE MAC updates right after Refresh BLE Identity

With advertising off, pressing **Refresh BLE Identity** changed the device name but left the **ESP32 BLE MAC** sensor on the old address until advertising started again. The sensor now updates as soon as the new address is in use.

### Complete capture logs

The last lines of a capture, such as the unique device count, could be missing from the logs in ESPHome Device Builder. The base package now gives the logger a larger buffer, so the full capture is shown.

### Steadier pairing

Two rare timing problems are fixed. The ESP32 could stop looking for an IRK too early, or drop a connection that had only just started.

## Under the hood

The advertising, timer, and address rotation logic is simpler, with about 160 fewer lines of code. Captures behave the same. Advertising restarts slightly sooner after **Refresh BLE Identity**.

## Upgrading

- Perform a clean build; see [Upgrading to a New Version](README.md#upgrading-to-a-new-version).
- No YAML changes are needed. The larger log buffer comes with the base package and uses about 1 KB more memory.
