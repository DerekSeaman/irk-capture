# Release Notes v1.7.1

This release covers the changes since `v1.7.0`. It makes the ESP32 much easier to find when
your phone has seen it before, and lets you rename it in either BLE profile.

## What's new

### Refresh BLE Identity replaces Generate New MAC

The **Generate New MAC** button is now **Refresh BLE Identity**. Each press gives the ESP32 a
new MAC address *and* a matching new name, without rebooting:

- Heart Sensor: `IRK HR 7F3A`
- Keyboard: `IRK KB 7F3A`

The last four characters match the end of the **Effective MAC** sensor, so you can tell at a
glance which entry on your phone is the ESP32. The name also shows which profile is active.

Why both? iPhones hide an accessory whose *name* they have already seen, even on a brand new
MAC address. So after a previous pairing, a new MAC alone could leave the ESP32 appearing for a
second and then vanishing. Changing both makes it look like a new accessory every time.

The generated name lasts until the next reboot. Restarting the ESP32 (or changing the BLE
profile, which restarts it) brings back the default names: "IRK Capture" for Heart Sensor and
"Logitech K380" for Keyboard. Out of the box, nothing changes for Samsung Galaxy phones.

### Rename the Keyboard profile from Home Assistant

**BLE Device Name** now works in the Keyboard profile too. Type a name, press Enter, and the
ESP32 starts advertising it right away. Like the Heart Sensor profile, a custom name lasts
until the next reboot, which restores "Logitech K380".

## Tips

- Every new MAC address can show up as its own entry on your phone for a minute or two, so
  you may briefly see several ESP32 names. Pair with the one shown in **BLE Device Name**, or
  leave and reopen Bluetooth settings and only the current one comes back.
- On your phone/watch, turn off Bluetooth, then turn it on to see the ESP32 faster.

## Upgrading

- Perform a clean build when upgrading (see **Upgrading to a New Version** in the README).
- Because the button was renamed, Home Assistant adds a new **Refresh BLE Identity** button and
  shows the old **Generate New MAC** entity as unavailable. You can delete the old one. If an
  automation or dashboard used the old button, point it at the new one.
- If your own YAML configures the button, the new key is `refresh_identity`. The old `new_mac`
  key still works.

## Thanks to David Coulson

Refresh BLE Identity comes from David Coulson (@davidcoulson), who tracked down why iPhones
were hiding the ESP32 and built the name-plus-MAC refresh. His change derives the new name from
the MAC address only after the ESP32 has actually switched to it, so the name and the Effective
MAC sensor always agree, and adds a CI test for the name format.
