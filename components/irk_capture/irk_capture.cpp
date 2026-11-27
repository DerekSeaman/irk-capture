// Component lifecycle (setup, loop, dump_config), staged entity publishing,
// the label and stop-after-capture controls, and session status. The rest of
// IRKCaptureComponent is in the irk_capture_*.cpp files beside this one; see
// irk_capture_internal.h for the file map and what the files share.

#include "irk_capture_internal.h"

// ESP32-only implementation - requires Bluetooth hardware
#ifdef USE_ESP32

namespace esphome {
namespace irk_capture {

static constexpr char VERSION[] = "1.7.11";

//======================== Component lifecycle ========================

void IRKCaptureComponent::setup() {
  ESP_LOGI(TAG, "IRK Capture v%s starting", VERSION);

  // Sanitize initial name from YAML. The config validator already rejects
  // anything this would change, so the fallback only guards other callers.
  ble_name_ = sanitize_ble_name(ble_name_);
  if (ble_name_.empty()) ble_name_ = DEFAULT_BLE_NAME;

  // Stable for this boot, including NimBLE host recovery. A browser can detect
  // an MCU reboot even when its next capture has the same sequence number.
  // Set before anything can fail: the wizard's reboot and profile recovery
  // commands need it, and refuse a zero boot identity.
  esp_fill_random(&capture_result_.boot_id, sizeof(capture_result_.boot_id));
  if (capture_result_.boot_id == 0) capture_result_.boot_id = 1;

  state_mutex_ = xSemaphoreCreateMutex();
  if (!state_mutex_) {
    ESP_LOGE(TAG, "CRITICAL: Failed to create state mutex - thread safety compromised!");
    this->mark_failed(LOG_STR("state mutex allocation failed"));
    publish_setup_failure_();
    return;
  }
  ble_op_mutex_ = xSemaphoreCreateMutex();
  if (!ble_op_mutex_) {
    ESP_LOGE(TAG,
             "CRITICAL: Failed to create BLE op mutex - cannot serialize BLE "
             "host calls");
    this->mark_failed(LOG_STR("BLE operation mutex allocation failed"));
    publish_setup_failure_();
    return;
  }

  // Load persisted BLE profile from NVS (before BLE stack init so GATT is
  // correct)
  // "Not found" is the normal state before a profile has ever been saved. Any
  // other error means a saved profile could not be read, and the device would
  // otherwise come up in the default profile without saying why.
  nvs_handle_t nvs_handle;
  const esp_err_t open_err = nvs_open("irk_capture", NVS_READONLY, &nvs_handle);
  if (open_err != ESP_OK) {
    if (open_err != ESP_ERR_NVS_NOT_FOUND) {
      ESP_LOGW(TAG, "Saved BLE profile could not be opened (%s); using the default",
               esp_err_to_name(open_err));
    }
  } else {
    uint8_t profile_val = 0;
    const esp_err_t read_err = nvs_get_u8(nvs_handle, "ble_profile", &profile_val);
    if (read_err != ESP_OK) {
      if (read_err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Saved BLE profile could not be read (%s); using the default",
                 esp_err_to_name(read_err));
      }
    } else {
      if (profile_val <= static_cast<uint8_t>(BLEProfile::KEYBOARD)) {
        ble_profile_ = static_cast<BLEProfile>(profile_val);
        ESP_LOGI(TAG, "Loaded persisted BLE profile: %s",
                 ble_profile_ == BLEProfile::KEYBOARD ? "Keyboard" : "Heart Sensor");
      } else {
        ESP_LOGW(TAG, "Invalid persisted profile value %u, using default", profile_val);
      }
    }
    nvs_close(nvs_handle);
  }

  // Clear in-memory IRK cache for fresh session (complements ble_store_clear()
  // which is called later during BLE stack initialization)
  irk_cache_.clear();
  irk_cache_.reserve(cache_limit_());
  history_changed_locked_();
  capture_events_ = 0;
  unique_devices_ = 0;
  connection_generation_ = 0;
  connection_origin_ = CaptureOrigin::UNKNOWN;
  last_result_generation_ = 0;
  capture_status_ = CaptureStatus::NONE;
  status_result_hold_until_ = 0;
  next_capture_label_.clear();

  // Resolve startup intent before starting the NimBLE task. DISABLED (the
  // platform default) preserves start_on_boot; explicit restore modes win.
  if (advertising_switch_) {
    auto restored = advertising_switch_->get_initial_state_with_restore_mode();
    if (restored.has_value()) {
      advertising_requested_ = restored.value() != advertising_switch_->is_inverted();
    }
  }

  if (stop_after_capture_switch_) {
    auto restored = stop_after_capture_switch_->get_initial_state_with_restore_mode();
    if (restored.has_value()) {
      stop_after_capture_ = restored.value() != stop_after_capture_switch_->is_inverted();
    }
  }

  // Start the host wait before setup_ble(). The host can sync while setup_ble()
  // is still running, and on_ble_host_synced() clears this value then. Setting
  // it afterwards would leave a synced host marked as never synced, and Status
  // would read "failed" 30 s later.
  {
    MutexGuard lock(state_mutex_);
    host_wait_since_ = deadline_after(now_ms(), 0);  // Cleared when the host syncs.
  }
  this->setup_ble();
  if (this->is_failed()) {
    publish_setup_failure_();
    return;
  }

  // on_ble_host_synced() applies the resolved intent when NimBLE becomes ready.

  if (ble_name_text_) {
    std::string name;
    {
      MutexGuard lock(state_mutex_);
      name = advertised_name_();
    }
    ble_name_text_->publish_state(name);
  }
  if (ble_profile_select_) {
    ble_profile_select_->publish_state(ble_profile_ == BLEProfile::KEYBOARD ? "Keyboard"
                                                                            : "Heart Sensor");
  }
  if (advertising_switch_) {
    advertising_switch_->publish_state(is_advertising_requested());
  }
  if (stop_after_capture_switch_) {
    stop_after_capture_switch_->publish_state(get_stop_after_capture());
  }
  if (status_sensor_) {
    last_status_value_ = "idle";
    status_sensor_->publish_state(last_status_value_);
  }
}

void IRKCaptureComponent::publish_setup_failure_() {
  // Main task, from setup() only. loop() never runs on a failed component, so
  // nothing else would publish Status and it would stay unknown.
  if (!status_sensor_) return;
  last_status_value_ = "failed";
  status_sensor_->publish_state(last_status_value_);
}

void IRKCaptureComponent::dump_config() {
  std::string name_copy;
  bool adv_state;
  BLEProfile current_profile;
  {
    MutexGuard lock(state_mutex_);
    name_copy = advertised_name_();
    adv_state = advertising_;
    current_profile = ble_profile_;
  }

  const char* effective_name = name_copy.c_str();
  const char* profile_name =
      (current_profile == BLEProfile::KEYBOARD) ? "Keyboard" : "Heart Sensor";

  // Single consolidated log line avoids UART buffer overflow without vTaskDelay
  // hacks
  ESP_LOGCONFIG(TAG, "IRK Capture v%s: profile=%s name='%s' adv=%s", VERSION, profile_name,
                effective_name, adv_state ? "YES" : "NO");
}

void IRKCaptureComponent::loop() {
  const uint32_t now = now_ms();
  if (now - last_loop_ < TimingConfig::LOOP_MIN_INTERVAL_MS) return;
  last_loop_ = now;

  // Drain entity values and their matching Status from one shared snapshot.
  flush_pending_publishes_();

  report_host_stall_(now);

  // Some NPL ports drop full-queue events; others block the producer. Retry
  // through the worker so neither behavior can stall the ESPHome main task.
  notify_bond_clear_();

  // Timers for IRK checks
  run_peer_timer_(post_disc_timers_, now, "DISC_DELAYED");
  run_peer_timer_(late_enc_timers_, now, "ENC_LATE");

  // Rotation must not starve security timeouts or IRK polling on a connection
  // that arrived while the radio was stopping.
  handle_mac_rotation_(now);
  uint16_t conn_handle_copy;

  // Reconcile unwanted advertising and scheduled advertising restarts.
  bool should_restart_adv = false;
  bool should_stop_unwanted_adv = false;
  {
    MutexGuard lock(state_mutex_);
    // After a lock timeout, wait for the retry deadline instead of blocking
    // and logging on every tick.
    should_stop_unwanted_adv =
        (advertising_ || adv_stop_retry_time_ != 0) && !advertising_requested_ &&
        (adv_stop_retry_time_ == 0 || deadline_reached(now, adv_stop_retry_time_));
    // A failed restart may have left the old advertisement active. Its retry
    // must still run so start_advertising() can stop and replace that payload.
    if (advertising_requested_ && !connected_ && mac_rotation_state_ == MacRotationState::IDLE &&
        adv_restart_time_ != 0 && deadline_reached(now, adv_restart_time_)) {
      adv_restart_time_ = 0;
      should_restart_adv = true;
    }
  }
  if (should_stop_unwanted_adv) {
    stop_advertising();
  }
  if (should_restart_adv) {
    ESP_LOGI(TAG, "Auto-restarting advertising after scheduled delay");
    start_advertising();
  }

  // Pairing robustness (single retry)
  retry_security_if_needed(now);

  // Encryption and overall pairing deadlines share termination, retry, and
  // cleanup. Once recovery starts, leave the link to that handler until closed.
  // Earlier loop work can let NimBLE accept a new connection; re-sample now_ms().
  if (handle_connection_timeout_(now_ms())) return;

  bool is_connected;
  {
    MutexGuard lock(state_mutex_);
    is_connected = connected_;
    conn_handle_copy = conn_handle_;
  }
  if (!is_connected || conn_handle_copy == BLE_HS_CONN_HANDLE_NONE) return;

  // HR notify (from connect, to keep the link active) and IRK polling (post-ENC)
  notify_hr_if_due(now);
  poll_irk_if_due(now);
}

void IRKCaptureComponent::publish_irk_to_sensors(const std::string& irk_hex,
                                                 const std::string& addr_str,
                                                 uint32_t connection_generation, bool capture_event,
                                                 bool stop_after_result, IrkCheck check) {
  // Stage only; the ESPHome main loop() performs the actual publish_state().
  // Callers may run in the NimBLE task, where publish_state() is unsafe.
  MutexGuard lock(state_mutex_);
  if (capture_cancelled_locked_(connection_generation)) return;
  // A valid IRK for the live connection ends its capture work whether or not it
  // becomes the displayed result, so Status stops reporting "capturing".
  if (connected_ && connection_generation != 0 && connection_generation == connection_generation_) {
    irk_gave_up_ = true;
  }
  // An older delayed result must not overwrite the outcome of a newer pairing.
  if (connection_generation != 0 && last_result_generation_ != 0 &&
      static_cast<int32_t>(connection_generation - last_result_generation_) < 0)
    return;
  // Commit automatic OFF with its accepted result. A new wizard begin can
  // then snapshot that result and request ON without an older callback's
  // logging tail switching the new attempt back off. loop() stops an idle
  // radio; disconnect callbacks also honor this intent before restarting.
  if (stop_after_result) {
    if (advertising_requested_) {
      advertising_start_attempts_ = 0;
      advertising_failure_log_time_ = 0;
    }
    advertising_requested_ = false;
    adv_restart_time_ = 0;
    pending_adv_pub_ = true;
  }
  // Stage status and IRK under the same ordering check. Only a genuine capture
  // replaces what is on display. A bonded reconnect of an earlier device may
  // restore its IRK over a no-IRK failure (or an empty sensor), but must never
  // replace another device's result while the user is copying it.
  if (capture_event) {
    if (connection_generation == 0 || capture_result_generation_ != connection_generation) {
      capture_result_.sequence++;
      if (capture_result_.sequence == 0) capture_result_.sequence = 1;
      capture_result_.irk = irk_hex;
      capture_result_.device_mac = addr_str;
      capture_result_.verification = check;
      capture_result_.connection_generation = connection_generation;
      capture_result_generation_ = connection_generation;
      if (!capture_triggers_.empty()) {
        if (pending_capture_events_.size() < CAPTURE_EVENT_CAPACITY) {
          pending_capture_events_.push_back({ irk_hex, addr_str, check });
        } else if (pending_capture_event_drops_ != UINT32_MAX) {
          // Never overwrite an event already awaiting delivery. Keep the latest
          // result/history available and report this exceptional loss on loop().
          ++pending_capture_event_drops_;
        }
      }
    }
    capture_status_ = CaptureStatus::CAPTURED;
    status_result_hold_until_ = deadline_after(now_ms(), TimingConfig::STATUS_RESULT_HOLD_MS);
  } else if (capture_status_ == CaptureStatus::NO_IRK || last_result_generation_ == 0) {
    capture_status_ = CaptureStatus::NONE;
    status_result_hold_until_ = 0;
  } else {
    return;
  }
  if (connection_generation != 0) last_result_generation_ = connection_generation;
  pending_irk_hex_ = irk_hex;
  pending_irk_addr_ = addr_str;
  pending_irk_check_ = check;
  pending_irk_pub_ = true;
}

void IRKCaptureComponent::publish_effective_mac() {
  if (!effective_mac_sensor_) return;

  uint8_t mac[6];
  if (get_own_addr(mac, nullptr)) {
    const std::string mac_str = mac_to_str(mac);
    // Stage only; drained on the main task in loop().
    {
      MutexGuard lock(state_mutex_);
      pending_effmac_ = mac_str;
      pending_effmac_pub_ = true;
    }
    ESP_LOGD(TAG, "Staged effective MAC: %s", mac_str.c_str());
  } else {
    // The sensor keeps its previous value, which may no longer be the address in use.
    ESP_LOGW(TAG, "Own BLE address could not be read; ESP32 BLE MAC sensor not updated");
  }
}

void IRKCaptureComponent::request_reboot_() {
  // App.safe_reboot() tears down every component, so it has to run on the main
  // task, and callers include the NimBLE host task and the wizard's HTTP
  // handler. A zero-delay timeout goes through the scheduler's thread-safe
  // defer queue, like defer(). defer() itself is not used because the
  // scheduler drops a failed component's callbacks, and a profile change after
  // a failed setup still has to reboot; the self-keyed overload always fires.
  App.scheduler.set_timeout(this, 0, []() { App.safe_reboot(); });
}

void IRKCaptureComponent::publish_result_entities_(const std::string& irk, const std::string& mac,
                                                   IrkCheck check) {
  if (irk_sensor_) irk_sensor_->publish_state(irk);
  if (address_sensor_) address_sensor_->publish_state(mac);
  if (irk_check_sensor_) irk_check_sensor_->publish_state(irk_check_name(check));
}

void IRKCaptureComponent::publish_status_if_changed_(const char* status) {
  if (status_sensor_ == nullptr || last_status_value_ == status) return;
  last_status_value_ = status;
  status_sensor_->publish_state(status);
}

IRKCaptureComponent::PendingPublishBatch IRKCaptureComponent::take_pending_publishes_() {
  PendingPublishBatch b;
  MutexGuard lock(state_mutex_);
  b.capture_events.swap(pending_capture_events_);
  b.dropped_capture_events = pending_capture_event_drops_;
  pending_capture_event_drops_ = 0;
  b.adv_pub = pending_adv_pub_;
  b.adv_val = advertising_requested_;
  pending_adv_pub_ = false;
  b.irk_pub = pending_irk_pub_;
  pending_irk_pub_ = false;
  b.irk_hex.swap(pending_irk_hex_);
  b.irk_addr.swap(pending_irk_addr_);
  b.irk_check = pending_irk_check_;
  b.effmac_pub = pending_effmac_pub_;
  pending_effmac_pub_ = false;
  b.effmac.swap(pending_effmac_);
  b.stop_after_pub = pending_stop_after_capture_pub_;
  b.stop_after_val = stop_after_capture_;
  pending_stop_after_capture_pub_ = false;
  b.label_pub = pending_label_pub_;
  pending_label_pub_ = false;
  b.label_val.swap(pending_label_val_);
  b.name_pub = pending_ble_name_pub_;
  pending_ble_name_pub_ = false;
  if (b.name_pub) b.name_val = advertised_name_();
  // A BLE callback may stage another result as soon as this lock is released.
  // Its terminal Status must wait for the next drain alongside its IRK/MAC.
  if (status_sensor_) b.status = status_value_locked_(now_ms());
  return b;
}

void IRKCaptureComponent::flush_pending_publishes_() {
  // Runs on the ESPHome main task. Take staged values under the mutex, then
  // publish outside it (publish_state can be slow and must not hold the lock).
  const PendingPublishBatch b = take_pending_publishes_();
  if (b.adv_pub && advertising_switch_) advertising_switch_->publish_state(b.adv_val);
  if (b.effmac_pub && effective_mac_sensor_) effective_mac_sensor_->publish_state(b.effmac);
  if (b.stop_after_pub && stop_after_capture_switch_)
    stop_after_capture_switch_->publish_state(b.stop_after_val);
  if (b.label_pub && next_capture_label_text_) next_capture_label_text_->publish_state(b.label_val);
  // Home Assistant would otherwise keep showing the name the device stopped
  // advertising the moment the identity was refreshed.
  if (b.name_pub && ble_name_text_) ble_name_text_->publish_state(b.name_val);
  // Drain this snapshot in order. A callback may stage more work, which stays
  // queued for the next flush rather than extending this batch indefinitely.
  // Each event's sensors and terminal Status are visible before its automation.
  for (const auto& event : b.capture_events) {
    publish_result_entities_(event.irk, event.mac, event.verification);
    publish_status_if_changed_("captured");
    const std::string verification = irk_check_name(event.verification);
    for (auto* trigger : capture_triggers_) trigger->trigger(event.irk, event.mac, verification);
  }
  // A newer no-IRK result or a capture whose event overflowed can differ from
  // the last queued event. Restore the matching latest snapshot after delivery.
  const bool latest_already_published = !b.capture_events.empty() &&
                                        b.capture_events.back().irk == b.irk_hex &&
                                        b.capture_events.back().mac == b.irk_addr &&
                                        b.capture_events.back().verification == b.irk_check;
  if (b.irk_pub && !latest_already_published)
    publish_result_entities_(b.irk_hex, b.irk_addr, b.irk_check);
  // Publish last so Status observers can read the corresponding result values.
  if (b.status != nullptr) publish_status_if_changed_(b.status);
  if (b.dropped_capture_events != 0) {
    ESP_LOGE(TAG,
             "on_capture queue full: %" PRIu32
             " event(s) could not be delivered; "
             "check capture history for results",
             b.dropped_capture_events);
  }
}

//======================== Session controls and status ========================

void IRKCaptureComponent::set_stop_after_capture(bool enabled) {
  MutexGuard lock(state_mutex_);
  stop_after_capture_ = enabled;
  pending_stop_after_capture_pub_ = true;
}

bool IRKCaptureComponent::get_stop_after_capture() {
  MutexGuard lock(state_mutex_);
  return stop_after_capture_;
}

std::string IRKCaptureComponent::get_next_capture_label() {
  MutexGuard lock(state_mutex_);
  return next_capture_label_;
}

LabelUpdateResult IRKCaptureComponent::set_next_capture_label(const std::string& value) {
  // Labels aren't advertised over the air, so they get a longer budget than
  // the 12-byte BLE-name cap, and an apostrophe for names like "Dave's iPhone".
  // Everything kept is safe unescaped inside build_history_json()'s strings.
  std::string sanitized;
  sanitized.reserve(value.size());
  for (char c : value) {
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == ' ' ||
        c == '-' || c == '_' || c == '\'') {
      sanitized += c;
    }
  }
  // Trim surrounding spaces before truncating so leading spaces do not eat into
  // the length budget and space-only text is rejected instead of stored.
  const size_t start = sanitized.find_first_not_of(' ');
  if (start == std::string::npos) {
    sanitized.clear();
  } else {
    sanitized = sanitized.substr(start, sanitized.find_last_not_of(' ') - start + 1);
    if (sanitized.length() > CAPTURE_LABEL_MAX_LEN) {
      sanitized.resize(CAPTURE_LABEL_MAX_LEN);
      sanitized.erase(sanitized.find_last_not_of(' ') + 1);
    }
  }
  if (sanitized.empty() && !value.empty()) {
    // Only an empty value clears the label. Text with no supported characters
    // (an emoji, say) is refused instead of silently wiping the saved label.
    // Republish the kept label so a Home Assistant text entity snaps back.
    LabelUpdateResult result;
    {
      MutexGuard lock(state_mutex_);
      result.stored_label = next_capture_label_;
      pending_label_val_ = next_capture_label_;
      pending_label_pub_ = true;
    }
    ESP_LOGW(TAG, "Next capture label rejected: no supported characters");
    return result;
  }
  {
    MutexGuard lock(state_mutex_);
    next_capture_label_ = sanitized;
    pending_label_val_ = sanitized;
    pending_label_pub_ = true;
  }
  ESP_LOGD(TAG, "Next capture label set to '%s'", sanitized.c_str());
  return { true, sanitized };
}

SessionState IRKCaptureComponent::get_session_state(uint32_t known_history_version) {
  // One lock for everything a status page shows. Reading the same values
  // through the individual getters takes state_mutex_ once per value, several
  // times a second, in competition with the NimBLE task during pairing.
  SessionState state;
  std::vector<IRKCacheEntry> cache_copy;
  {
    MutexGuard lock(state_mutex_);
    state.history_version = history_version_;
    // An unchanged history needs neither the copy under the lock nor the
    // formatting after it; the caller keeps the JSON it already has.
    state.history_changed = known_history_version != history_version_;
    if (state.history_changed) cache_copy = irk_cache_;
    state.next_capture_label = next_capture_label_;
    state.advertised_name = advertised_name_();
    state.profile = selected_profile_();
    state.advertising_requested = advertising_requested_;
    state.stop_after_capture = stop_after_capture_;
    state.capture_result = capture_result_;
    state.bond_clear = bond_clear_result_;
    state.identity_refresh = identity_refresh_result_;
  }
  if (state.history_changed)
    state.history_json = format_history_json_(cache_copy);
  else
    state.history_json.clear();
  return state;
}

LiveControls IRKCaptureComponent::get_live_controls() {
  MutexGuard lock(state_mutex_);
  LiveControls controls;
  controls.next_capture_label = next_capture_label_;
  controls.profile = selected_profile_();
  controls.advertising_requested = advertising_requested_;
  controls.stop_after_capture = stop_after_capture_;
  return controls;
}

const char* IRKCaptureComponent::status_value_locked_(uint32_t now) {
  // Caller holds state_mutex_ while also snapshotting the pending IRK/MAC.
  // Retire expired deadlines permanently: signed wrap-safe comparisons only
  // order timestamps within half the clock period (about 25 days).
  if (status_result_hold_until_ != 0 && deadline_reached(now, status_result_hold_until_)) {
    status_result_hold_until_ = 0;
  }
  const bool current_result =
      connection_generation_ != 0 && last_result_generation_ == connection_generation_;
  if (capture_status_ != CaptureStatus::NONE &&
      (status_result_hold_until_ != 0 || (connected_ && current_result))) {
    switch (capture_status_) {
      case CaptureStatus::CAPTURED:
        return "captured";
      case CaptureStatus::NO_IRK:
        return "no_irk";
      default:
        return "failed";
    }
  }
  // A host that has stayed unsynced is a fault, not an idle radio.
  if (host_wait_since_ != 0 &&
      elapsed_ms(now, host_wait_since_) > TimingConfig::HOST_SYNC_TIMEOUT_MS) {
    return "failed";
  }
  if (connected_ && enc_ready_) {
    // irk_gave_up_ also covers polling timeouts; it is not evidence of a
    // no-IRK outcome. Completed reconnects/timeouts are simply waiting to close.
    if (!current_result && !irk_gave_up_) return "capturing";
    // A device that was already bonded reconnected and its key was read again.
    // Saying so explains a connection that produced no new capture.
    return connection_origin_ == CaptureOrigin::BONDED ? "reconnected" : "idle";
  }
  if (connected_) return "pairing";
  return advertising_ ? "advertising" : "idle";
}

}  // namespace irk_capture
}  // namespace esphome

#endif  // USE_ESP32
