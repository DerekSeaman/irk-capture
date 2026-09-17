# Unreleased changes

## Capturing more than three devices in one session

Pairing a fourth device in a single session could fail outright. The Bluetooth
stack keeps room for only three stored pairings, and once it was full it turned
new pairing requests away instead of making space — even with capture slots
still free. The device now retires the oldest stored pairing automatically, so
you can keep capturing without rebooting between devices. Already-captured IRKs
are unaffected.

## Advertising startup and recovery

The BLE Advertising switch now honors `restore_mode`. An explicit restore mode
takes precedence over `irk_capture.start_on_boot`; omitting it or selecting
`DISABLED` delegates startup to `start_on_boot`.

The shared package changes its restore mode from the previously ineffective
`ALWAYS_ON` to `DISABLED`. This preserves existing startup behavior, including
`start_on_boot: false`. If your own YAML explicitly selects `ALWAYS_ON`, that
setting now starts advertising ON even when `start_on_boot` is false. Select
`DISABLED` to continue controlling startup with `start_on_boot`, or `ALWAYS_OFF`
to always start OFF.

Advertising and random-address initialization failures now retry after 1, 2, 4,
and 8 seconds, then once per minute. The switch retains the requested ON state.
Slow recovery is announced once, continuing failure logs are limited to one
every five minutes, and successful recovery is logged. Turning the switch OFF
cancels recovery; turning it back ON starts a fresh attempt. This recovery does
not reboot the device or clear its capture session.

## Capture reliability

- A completed pairing that provides no IRK now updates the IRK sensor to
  `Failed: IRK not used` with the matching Device MAC, before timeout cleanup.
  This replaces the previous result; Home Assistant's recorded history retains
  earlier values. Missing bonds and incomplete pairing are not classified as
  unused IRKs. A subsequent successful reconnect restores the captured IRK.
- A full fallback-timer queue replaces its oldest pending check instead of
  rebooting and losing the session.
- Submitting the current BLE name leaves the connection and bonds intact.
  Failed name updates can be retried.
- MAC rotation synchronizes its shared state and discards stale work after a
  host reset. Waiting for a connection no longer floods warnings or skips
  connection maintenance.
- Bond clearing during rotation runs outside the component mutexes. Previous
  and effective MAC addresses remain available in the logs.
- A failed connection-descriptor lookup after encryption is logged; the
  existing disconnect fallback remains available.
- A pending MAC change no longer gets stuck when a connection closes without
  notifying the device, which previously left advertising off with the switch
  still on.
- A late IRK result can no longer disconnect a different device that connected
  in the meantime.
- The Heart Sensor profile's heart-rate notifications now actually run; they
  were silently disabled, which could make some phones and watches drop the
  connection mid-pairing.
- Device information and the pairing-trigger characteristic can now be read on
  an encrypted connection. They previously refused every read, which some
  devices treat as a faulty accessory.
- Pairing failure reasons are reported correctly in the logs. The previous
  decoding mislabelled them, and the recovery step for a stale-key failure
  never ran.
- Changing the BLE profile no longer reboots into the old profile when the new
  one cannot be saved; the selection is restored and an error is logged instead.
- Home Assistant no longer shows a new BLE name that the device rejected.
- Encryption and overall pairing timeouts now share one disconnect retry budget
  and cleanup path. Once a pairing has taken too long and recovery starts, it
  always runs to completion and closes the connection, even if the device
  finishes pairing a moment later. An IRK captured in that window is still kept
  and published; only the stalled attempt is ended, and the device can simply be
  paired again.
- When a pairing completes without an IRK, the logs now also suggest the peer's
  MAC address as a fallback to use directly, phrased conditionally because a
  missing IRK does not on its own prove the device uses a fixed address.
