# Unreleased changes

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
