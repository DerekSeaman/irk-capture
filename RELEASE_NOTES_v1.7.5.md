# Release Notes v1.7.5

IRK Capture v1.7.5 reorganises the firmware and makes the device log more useful for diagnosing pairing problems. Captures and Home Assistant entities work as they did in v1.7.4, and Status gains one new value.

## What's new for you

- **New Status value: `reconnected`.** When a device that is already paired reconnects (for example an iPhone you left paired, tapping the ESP32 in its Bluetooth list), Status now shows `reconnected` instead of `idle` until the connection closes. Its key is read again, but it doesn't count as a new capture. The wizard shows it too. If a Home Assistant automation compares Status against a fixed list of values, add `reconnected` to it.

There are no new entities or options, and no YAML changes are needed. The device log now explains each pairing in more detail:

- **Pairing timing.** The `ENC_CHANGE` line says how long encryption took after the connection opened, and the disconnect line says how long the connection lasted and whether it was encrypted. A short connection that never encrypted points to a different problem than a slow one.
- **Security after pairing.** A new `sec after pairing` line shows the link's state once encryption is up (encrypted, bonded, key size). The existing `sec` line is logged at connect, before pairing, so it always shows zeros.
- **Readable values.** Settings that were logged as bare numbers now include their meaning: the pairing IO capability (`NoInputNoOutput`), the keys exchanged (`ENC|ID`), connection interval and supervision timeout in milliseconds, and why a subscription changed (`peer wrote CCCD`, `connection ended`).
- **Characteristic names.** Subscription lines name the characteristic, for example `attr=8 (Service Changed)`.
- **Clearer security retry.** The line logged two seconds into pairing used to read `Retrying security initiate`, even when pairing was simply still running. It now says whether a retry happened, and logs at debug level when it was skipped because pairing was already under way.
- **Bond eviction.** When the bond store is full, the log names the device whose bond was removed to make room. This used to happen silently, and a device whose bond disappeared would re-pair without explanation.
- **Every bonded reconnect is logged.** A paired device that reconnects within 60 seconds of its key last being published isn't published again, and that used to leave nothing in the debug log. It now logs `Bonded reconnect (2/5): IRK published 7000 ms ago; not republishing`, so the reconnect count no longer appears to jump.
- **Connection updates.** When a phone changes the connection parameters mid-connection, the new values are logged. These events used to appear only as `Unhandled GAP event`.

## Upgrading

- **Rebuild as usual.** Devices that track `main` pick up this version on their next build.
- **No YAML changes are needed.**

## For developers

- The component's C++ is split by responsibility. `IRKCaptureComponent` is still one class with the same locking; its definition now spans `irk_capture.cpp`, `irk_capture_ble.cpp`, `irk_capture_gap.cpp`, `irk_capture_advertising.cpp`, `irk_capture_identity.cpp`, `irk_capture_session.cpp`, `irk_capture_history.cpp` and `irk_capture_entities.cpp`. Shared includes, constants, helpers and threading notes live in `irk_capture_internal.h`, which lists what each file holds.
- Eleven helpers that moved into the header are now `static inline`, and `persist_ble_profile` has external linkage.
- New logging helpers in `irk_capture_internal.h`: `io_cap_str`, `key_dist_str`, `log_conn_params` and `log_sec_state`. `log_sm_config` takes an optional label and is shared by the boot and per-connection lines.
- The bond store status callback is now `log_and_evict_bond` in `irk_capture_ble.cpp`. It still delegates to NimBLE's `ble_store_util_status_rr` and compares the bonded-peer list before and after to name the evicted peer.
- `BLE_GAP_EVENT_CONN_UPDATE` now has its own case in the GAP dispatcher.
- CI now also validates and compiles the two packages in `ESPHome Devices/` (base and wizard, ESP32-C3), not only the generated CI configuration.
