# Release Notes v1.7.1

IRK Capture v1.7.1 adds Home Assistant events for new captures and improves connection reliability.

## What's new

### Home Assistant events and MQTT (optional)

Each capture now fires an `esphome.irk_captured` event in Home Assistant with the IRK and the device's BLE MAC. Use it to trigger automations, such as a notification or publishing new IRKs to MQTT for other systems.

To receive the event, enable **Allow the device to perform Home Assistant actions** for your IRK Capture device. See [Optional: Home Assistant Events and MQTT Publishing](README.md#optional-home-assistant-events-and-mqtt-publishing) for setup, an example MQTT automation, and how to verify it's working.

### More reliable connections

If a connection attempt fails but leaves the Bluetooth link open, the ESP32 now closes it so it can advertise again.

### Clearer logs

Log messages describe pairing and disconnect events more accurately, which makes troubleshooting easier.

## Upgrading

- Perform a clean build; see [Upgrading to a New Version](README.md#upgrading-to-a-new-version).
- No YAML changes are needed. The event is included in the base package.

## Thanks

Thanks to [stephen-zlon](https://github.com/stephen-zlon) for suggesting capture events.
