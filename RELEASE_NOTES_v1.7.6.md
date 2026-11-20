# Release Notes v1.7.6

IRK Capture v1.7.6 makes clearing pairings complete and fixes several problems found in a code review. Refresh BLE Identity and Forget All Bonds now fully remove each paired device, and both report clearly when cleanup is incomplete. Status reports `failed` when pairing times out. The wizard login is harder to brute-force, and the wizard rejects malformed commands and credentials that are too long to work.

## What's new for you

- **Clearing pairings is complete.** Refresh BLE Identity and Forget All Bonds used to delete the stored pairings but leave each device's identity key in the list Bluetooth uses to recognise private addresses. A cleared phone was still recognised the moment it reconnected (the log showed its real identity in the `peer id` line) until the ESP32 rebooted. Each paired device is now unpaired one by one, which removes it from both places, and then the bond store is cleared as before.
- **Refresh BLE Identity is more reliable.** It now waits until advertising has actually stopped before clearing pairings and changing the address, and retries the stop if it doesn't take. Before, one failed stop could make the address change fail.
- **Incomplete cleanup is reported, with the right advice.** If Refresh BLE Identity changes the address but can't remove every pairing, it no longer reports success. The wizard says the address changed but some pairing information could not be removed, and suggests pressing Forget All Bonds. Forget All Bonds then says which step failed:
  - "Bond storage could not be cleared; retry." Stored pairings are kept in flash, so a restart doesn't remove them.
  - "Some devices stay recognised until the ESP32 restarts." The pairings are gone, and a restart clears the leftover keys.
- **Pairing timeouts show `failed`.** When a device connects but doesn't finish pairing (no encryption after 20 seconds, or no result after 90 seconds), Status now shows `failed` for a few seconds before returning to `advertising`. It used to go straight back to `advertising`, which made a stalled pairing look like nothing had happened. A timeout never replaces a key the connection already captured.
- **Disconnecting after a capture can't hit the wrong device.** After a capture, the ESP32 disconnects the device it captured from. That disconnect now runs on the Bluetooth task, right after checking it's still the same connection. Before, a device that reconnected within a fraction of a second could be disconnected instead.

There are no new entities, options or Status values.

### Wizard

- **The login lockout can't be bypassed.** Someone guessing passwords from five or more network addresses could reset the failed-login count on every attempt and never be locked out. A recent failed login now stays counted until that address has been idle for the full lockout period. While an attack from many addresses is under way, a new address shares the overflow limit with them, so you might be locked out briefly too.
- **Malformed commands are rejected.** The wizard used to act on a command whose JSON was truncated or invalid after the part it read. It now checks the whole command first. This only affects scripts that send the wizard commands directly. The page itself always sends valid JSON.
- **Credentials that can't work are caught when you build.** Username and password together are limited to 185 characters, because a longer pair produces a login header the ESP32 rejects, so you could never sign in. The username also can't contain `:`, which HTTP Basic authentication uses as its separator. Both now cause a configuration error instead of firmware you can't log in to.

## Upgrading

- **Rebuild as usual.** Devices that track `main` pick up this version on their next build.
- **No YAML changes are needed,** unless your wizard username contains `:` or your username and password together exceed 185 characters. The build then stops with an error that says which, and you can change them in `secrets.yaml`.
- **Home Assistant automations:** Status can now show `failed` after a pairing timeout. If an automation treats `failed` as an encryption error only, it now fires for timeouts too.

## For developers

- `unpair_and_clear_bonds()` in `irk_capture_history.cpp` lists the bonded peers with `ble_store_util_bonded_peers()`, calls `ble_gap_unpair()` on each (which removes the peer from the resolving list and the store), then calls `ble_store_clear()` for any remaining records. It returns a `BondCleanupResult` (in `irk_capture_internal.h`) with `store_rc` and `unpair_rc` kept separate, because the two failures need different advice. `unpair_rc` holds the first listing or unpair error, even when the final store clear succeeds. `handle_mac_rotation_()` and `handle_forget_bonds_()` use the helper in place of `ble_store_clear()`. The boot-time clear keeps `ble_store_clear()`, because the resolving list is empty at boot.
- `IdentityRefreshState` gains `PARTIAL`, which appears in the wizard's status JSON as `identity_refresh.state: "partial"`. It is set when the address changed but `BondCleanupResult::ok()` was false.
- `handle_mac_rotation_()` no longer assumes the stop issued by `request_mac_rotation_()` worked. A new `mac_rotation_prepared_` flag keeps it retrying `stop_advertising()` until `advertising_` is false, sharing the rotation's retry budget, before it clears bonds. `loop()` retries a stop only when the advertising intent is off, and a rotation keeps it on, so the rotation has to own this retry.
- `terminate_bounded_()` now takes the connection generation and only records the request. `handle_pending_termination_()` runs on the NimBLE task via the existing worker event, revalidates `connected_`, `conn_handle_` and `connection_generation_`, and then calls `ble_gap_terminate()`. Connect and disconnect callbacks run on the same task, so a reused handle can't slip in between the check and the call. If `ble_op_mutex_` is busy, it logs a warning and leaves the pairing timeout to close the link, instead of waiting on the host task.
- `handle_connection_timeout_()` now calls `report_capture_failure_()`. That function also checks that the generation is still current, and it records `failed_generation_` so each connection reports at most once.
- `ClientThrottles` no longer evicts a slot with recent failures. A slot's failures are forgotten only after it has been idle for `AuthThrottle::LOCKOUT_MS`.
- `json_skip_value()` is now a real validator, covering objects, arrays, strings, literals and numbers, with nesting limited to 16. `json_find_value_start()` validates the whole body, rejects trailing data, and rejects a command key that appears twice.
- `validate_auth_config()` in `irk_wizard/__init__.py` checks the Base64 header length (`6 + 4 * ceil((user + 1 + password) / 3)` ≤ 256) and rejects `:` in the username.
- Helpers used by a single file moved out of `irk_capture_internal.h`: `io_cap_str`, `key_dist_str` and `log_sm_config` into `irk_capture_ble.cpp`, and `log_conn_params` and `log_sec_state` into `irk_capture_gap.cpp`. `log_sm_config` is now an ordinary function declared in the header. The GATT UUIDs moved into the new `irk_capture_gatt.h`, which only the GATT registration, advertising and GAP files include.
- Not changed: the single-peer deletions after an encryption failure, a pairing timeout and a repeat-pairing request still use `ble_store_util_delete_peer()`, which doesn't remove a controller resolving-list entry. A repeat pairing must keep its link open, so it can't use `ble_gap_unpair()`. A peer that pairs again replaces its own entry, so these leave at most one stale entry per device that never comes back, until reboot.
