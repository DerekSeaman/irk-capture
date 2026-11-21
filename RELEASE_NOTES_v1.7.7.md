# Release Notes v1.7.7

IRK Capture v1.7.7 is a small maintenance release. A capture label you type with no usable characters no longer wipes the label you already set, and the wizard can no longer be held up by a client that sends a request very slowly. The rest is internal tidying with no change in behavior.

## What's new for you

- **An unusable label no longer clears your label.** Capture labels keep letters, digits, spaces, hyphens, underscores and apostrophes. A label made only of other characters, such as an emoji or punctuation, used to be stripped to nothing and silently erase the label you had already saved. Now it is refused and your saved label stays. In the wizard you see an error saying which characters are allowed. In Home Assistant, the Next Capture Label entity switches back to the saved label. Clearing the field on purpose still removes the label.

There are no new entities, options or Status values.

### Wizard

- **A slow request can't hold up the wizard.** The wizard handles one request at a time. A signed-in client that sent a request a byte at a time, just fast enough to avoid timing out, could keep the wizard busy for about 40 minutes. Reading a request now stops after 10 seconds, and the wizard answers with a timeout error. The page itself sends small requests that finish well within that.

## Upgrading

- **Rebuild as usual.** Devices that track `main` pick up this version on their next build.
- **No YAML changes are needed.**
- **Scripts that call the wizard API:** `POST /api/label` now answers `400` for a non-empty label with no supported characters, instead of `200` with an empty label. An empty string still clears the label and answers `200`.

## For developers

- `set_next_capture_label()` returns `LabelUpdateResult { accepted, stored_label }`. A non-empty value that sanitizes to empty is refused: the stored label is unchanged, returned as `stored_label`, and republished so the text entity reverts. `/api/label` answers 400 when `accepted` is false.
- `read_request_body()` in `irk_wizard.cpp` has an overall deadline (`MAX_BODY_READ_MS`, 10 s) in addition to its three receive timeouts. One receive call can still be in progress when the deadline passes, so the worst case is about 15 s.
- `should_publish_irk()` is now `record_observation_locked_()` and returns its `PublishDecision` directly instead of a `bool` plus an out parameter.
- `capture_limit_reached_locked_()` holds the single-capture and `max_captures` rule that the capture path and the disconnect handler both use. `stop_after_capture_` is still checked separately.
- `handle_gap_enc_change()` handles the bond read with a `switch` over every `BondRead` value and no `default`, so `-Wswitch` flags any value added later.
- `flush_pending_publishes_()` takes everything staged as one `PendingPublishBatch` from `take_pending_publishes_()`, then publishes in the same order through `publish_result_entities_()` and `publish_status_if_changed_()`.
- Comment fixes: the reconnect-suppression guard and the `capture_triggers_` locking note now describe the current code, and `handle_mac_rotation_()` has a phase transition table.
