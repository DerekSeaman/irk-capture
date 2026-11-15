// The device's BLE identity: name sanitising and updates, MAC rotation, and
// the BLE profile and its persistence.

#include "irk_capture_internal.h"

// ESP32-only implementation - requires Bluetooth hardware
#ifdef USE_ESP32

namespace esphome {
namespace irk_capture {

//======================== BLE name sanitization ========================

std::string IRKCaptureComponent::sanitize_ble_name(const std::string& name) {
  // Keeps the characters that are safe to advertise (alphanumeric, space,
  // hyphen, underscore), trims surrounding spaces, then limits the length.
  // Returns an empty string when nothing usable remains; the caller decides
  // whether that means a default name or a rejected change.
  std::string sanitized;
  sanitized.reserve(name.size());
  size_t rejected = 0;
  for (char c : name) {
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == ' ' ||
        c == '-' || c == '_') {
      sanitized += c;
    } else {
      rejected++;
    }
  }
  if (rejected != 0) {
    ESP_LOGW(TAG, "Removed %zu unsupported character(s) from the BLE name", rejected);
  }

  // Trim before truncating so leading spaces do not use up the length limit.
  const size_t start = sanitized.find_first_not_of(' ');
  if (start == std::string::npos) {
    ESP_LOGW(TAG, "BLE name has no usable characters");
    return "";
  }
  sanitized = sanitized.substr(start, sanitized.find_last_not_of(' ') - start + 1);

  // 12 bytes for Samsung S24/S25 compatibility with single-UUID advertising.
  if (sanitized.length() > BLE_NAME_MAX_LEN) {
    ESP_LOGW(TAG, "BLE name truncated to %zu bytes (Samsung compatibility)", BLE_NAME_MAX_LEN);
    sanitized.resize(BLE_NAME_MAX_LEN);
    // The cut can land just after a space.
    sanitized.erase(sanitized.find_last_not_of(' ') + 1);
  }

  if (sanitized != name) {
    ESP_LOGI(TAG, "BLE name sanitized: '%s' -> '%s'", name.c_str(), sanitized.c_str());
  }
  return sanitized;
}

BLEProfile IRKCaptureComponent::get_ble_profile() {
  MutexGuard lock(state_mutex_);
  return ble_profile_;
}

std::string IRKCaptureComponent::get_ble_name() {
  MutexGuard lock(state_mutex_);
  return advertised_name_();
}

std::string IRKCaptureComponent::advertised_name_() {
  // Keyboard poses as a "Logitech K380" to get past Samsung's BLE filtering,
  // unless a name was entered in Home Assistant since the last reboot.
  if (ble_profile_ == BLEProfile::KEYBOARD && !keyboard_name_custom_) return KEYBOARD_DEFAULT_NAME;
  return ble_name_;
}

void IRKCaptureComponent::commit_generated_name_(const uint8_t* mac) {
  // Caller holds state_mutex_ and has validated the address operation's host
  // and rotation generations. Never derive a suffix for an uncommitted address.
  if (!identity_refresh_pending_ && !generated_name_active_) return;
  identity_refresh_pending_ = false;
  generated_name_active_ = true;
  const bool keyboard = ble_profile_ == BLEProfile::KEYBOARD;
  ble_name_ = identity_name(keyboard ? "KB" : "HR", mac[1], mac[0]);
  if (keyboard) keyboard_name_custom_ = true;
  pending_ble_name_pub_ = true;
}

//======================== MAC refresh (event-driven, non-blocking)
//========================

bool IRKCaptureComponent::rotation_current_locked_(uint32_t generation,
                                                   bool require_idle_link) const {
  // Caller holds state_mutex_. False once a reset, a newer request or (when
  // required) a new connection has overtaken the rotation that was sampled.
  return generation == mac_rotation_generation_ && host_synced_ &&
         (!require_idle_link || !connected_) &&
         mac_rotation_state_ == MacRotationState::READY_TO_ROTATE;
}

void IRKCaptureComponent::abort_rotation_locked_() {
  // Caller holds state_mutex_. Ends the rotation with the address unchanged.
  mac_rotation_state_ = MacRotationState::IDLE;
  mac_rotation_retries_ = 0;
  mac_rotation_ready_time_ = 0;
  // The address never changed, so there is no new suffix to name. Leaving the
  // request armed would rename the device as a side effect of the next plain
  // rotation, such as the one a name change from Home Assistant triggers.
  identity_refresh_pending_ = false;
  identity_refresh_result_.state = IdentityRefreshState::FAILED;
}

void IRKCaptureComponent::handle_mac_rotation_(uint32_t now) {
  uint32_t generation;
  uint32_t ready_time;
  uint8_t retries;
  uint8_t mac[6];
  uint16_t waiting_handle = BLE_HS_CONN_HANDLE_NONE;
  {
    MutexGuard lock(state_mutex_);
    if (!host_synced_ || bond_clear_pending_ ||
        mac_rotation_state_ != MacRotationState::READY_TO_ROTATE)
      return;
    generation = mac_rotation_generation_;
    if (conn_handle_ != BLE_HS_CONN_HANDLE_NONE) {
      // Log once for this connection, then wait for on_disconnect() to make
      // the rotation ready again. The rest of loop() still services the link.
      waiting_handle = conn_handle_;
      mac_rotation_state_ = MacRotationState::REQUESTED;
    }
    ready_time = mac_rotation_ready_time_;
    retries = mac_rotation_retries_;
    std::memcpy(mac, pending_mac_, sizeof(mac));
  }
  if (waiting_handle != BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGW(TAG, "MAC rotation: waiting for connection handle=%u to close", waiting_handle);
    return;
  }
  if (ready_time != 0 && !deadline_reached(now, ready_time)) return;

  if (ready_time == 0 && retries == 0) {
    {
      MutexGuard lock(state_mutex_);
      if (!rotation_current_locked_(generation)) return;
      adv_restart_time_ = 0;
    }
    // The store clear and logging must not hold either component mutex: a
    // slow operation must not make a NimBLE callback wait on ble_op_mutex_.
    log_mac("Previous");
    ESP_LOGI(TAG, "Clearing bonds and waiting %" PRIu32 " ms before MAC rotation",
             TimingConfig::MAC_ROTATION_SETTLE_DELAY_MS);
    int clear_rc = ble_store_clear();
    if (clear_rc != 0) {
      // The rotation still goes ahead: a new address is useful on its own. Old
      // pairings staying behind must not pass as a clean refresh, though.
      ESP_LOGE(TAG,
               "Stored pairings could not be cleared before MAC rotation rc=%d; "
               "press Forget All Bonds after the rotation completes",
               clear_rc);
    }
    {
      MutexGuard lock(state_mutex_);
      if (!rotation_current_locked_(generation)) return;
      mac_rotation_ready_time_ =
          deadline_after(now_ms(), TimingConfig::MAC_ROTATION_SETTLE_DELAY_MS);
    }
    return;
  }

  int rc;
  {
    BleOpGuard ble_lock(ble_op_mutex_, pdMS_TO_TICKS(TimingConfig::BLE_OP_LOCK_WAIT_MS));
    if (!ble_lock.acquired()) return;  // Retry on a later loop without log flooding.
    {
      MutexGuard lock(state_mutex_);
      if (!rotation_current_locked_(generation)) return;
    }
    rc = ble_hs_id_set_rnd(mac);
  }

  // NimBLE returns EINVAL only when it rejects the address bytes themselves, so
  // the same address can never succeed: draw another before retrying. Every
  // other failure comes from the controller or host being busy and may clear.
  const bool replace_address = rc == BLE_HS_EINVAL;
  uint8_t replacement[6];
  if (replace_address) {
    make_static_random_addr(replacement);
  }

  bool aborted = false;
  uint32_t retry_delay = 0;
  {
    MutexGuard lock(state_mutex_);
    // A reset or replacement request during the BLE call invalidates its
    // result. Never resurrect the old rotation or overwrite the new counter.
    if (!rotation_current_locked_(generation, false)) return;
    if (rc == 0) {
      random_address_ready_ = true;
      enc_ready_ = false;
      enc_time_ = 0;
      mac_rotation_state_ = MacRotationState::IDLE;
      mac_rotation_retries_ = 0;
      mac_rotation_ready_time_ = 0;
      identity_refresh_result_.state = IdentityRefreshState::SUCCEEDED;
      commit_generated_name_(mac);
    } else {
      retries = ++mac_rotation_retries_;
      aborted = retries >= TimingConfig::MAC_ROTATION_MAX_RETRIES;
      if (aborted) {
        abort_rotation_locked_();
      } else {
        // Back off instead of spending the whole budget within a few loop
        // ticks, which is shorter than any transient condition lasts.
        const uint8_t doublings = std::min<uint8_t>(retries - 1, 8);
        retry_delay = std::min(TimingConfig::MAC_ROTATION_RETRY_MS << doublings,
                               TimingConfig::MAC_ROTATION_RETRY_MAX_MS);
        mac_rotation_ready_time_ = deadline_after(now_ms(), retry_delay);
        if (replace_address) std::memcpy(pending_mac_, replacement, sizeof(pending_mac_));
      }
    }
  }
  if (rc == 0) {
    log_mac("Effective");
    ESP_LOGI(TAG, "MAC rotation complete: %s", mac_to_str(mac).c_str());
    // start_advertising() publishes the address only when it starts the radio.
    // With advertising off, the sensor would keep showing the previous address.
    publish_effective_mac();
  } else if (aborted) {
    ESP_LOGE(TAG, "MAC rotation aborted rc=%d after %u attempts", rc, retries);
  } else {
    ESP_LOGW(TAG, "MAC rotation rc=%d, retry %u/%u in %" PRIu32 " ms%s", rc, retries,
             TimingConfig::MAC_ROTATION_MAX_RETRIES, retry_delay,
             replace_address ? " with a new address" : "");
    return;
  }
  start_advertising();  // With the new address, or the old one after an abort.
}

void IRKCaptureComponent::refresh_identity() {
  request_mac_rotation_(true);
}

bool IRKCaptureComponent::request_mac_rotation_(bool refresh_name, const NameChange* name_change) {
  const std::string* accepted_name = name_change ? &name_change->name : nullptr;
  ESP_LOGI(TAG, "MAC rotation requested (non-blocking event-driven)");

  // Pre-generate the new MAC address before taking mutex (esp_fill_random is
  // slow)
  uint8_t temp_mac[6];
  make_static_random_addr(temp_mac);
  ESP_LOGD(TAG, "Pre-generated MAC: %s", mac_to_str(temp_mac).c_str());

  // THREAD-SAFE: Atomically update state machine and pending MAC buffer
  bool should_stop_adv;
  uint16_t conn_handle_copy;
  uint32_t rotation_generation;
  auto commit_request = [&]() {
    MutexGuard lock(state_mutex_);

    // A GAP name call can span a reset or a newer rotation request. Reject its
    // stale completion without cancelling or renaming that newer operation.
    if (name_change && (name_change->host_generation != host_generation_ ||
                        name_change->rotation_generation != mac_rotation_generation_ ||
                        name_change->name_generation != ble_name_generation_)) {
      pending_ble_name_pub_ = true;
      return false;
    }

    // Set rotation state and commit pre-generated MAC to shared buffer
    if (++identity_refresh_result_.operation_id == 0) ++identity_refresh_result_.operation_id;
    identity_refresh_result_.state = IdentityRefreshState::PENDING;
    mac_rotation_generation_++;
    rotation_generation = mac_rotation_generation_;
    // Naming intent belongs to this rotation generation. In particular, a
    // newer accepted manual name supersedes both an older refresh request and
    // any generated name waiting to publish on the main task.
    identity_refresh_pending_ = refresh_name;
    if (accepted_name) {
      ble_name_generation_++;
      ble_name_ = *accepted_name;
      generated_name_active_ = false;
      if (ble_profile_ == BLEProfile::KEYBOARD) {
        keyboard_name_custom_ = *accepted_name != KEYBOARD_DEFAULT_NAME;
      }
      pending_ble_name_pub_ = true;
    }
    mac_rotation_state_ = MacRotationState::REQUESTED;
    mac_rotation_ready_time_ = 0;
    mac_rotation_retries_ = 0;  // Reset retry counter for new rotation attempt
    std::memcpy(pending_mac_, temp_mac, sizeof(pending_mac_));

    // Snapshot connection state for disconnect logic
    should_stop_adv = advertising_;
    conn_handle_copy = conn_handle_;
    return true;
  };

  if (accepted_name) {
    // Keep the GAP write and acceptance serialized with other name writes. A
    // newer no-op name or refresh may supersede us during the stack call; in
    // that case restore the accepted GAP value before releasing the BLE lock.
    BleOpGuard ble_lock(ble_op_mutex_, pdMS_TO_TICKS(TimingConfig::BLE_OP_LOCK_WAIT_MS));
    if (!ble_lock.acquired()) {
      ESP_LOGW(TAG, "update_ble_name: ble_op_mutex timeout, name update skipped");
      MutexGuard lock(state_mutex_);
      pending_ble_name_pub_ = true;
      return false;
    }
    const int rc = ble_svc_gap_device_name_set(accepted_name->c_str());
    if (rc != 0) {
      ESP_LOGE(TAG, "ble_svc_gap_device_name_set failed rc=%d", rc);
      MutexGuard lock(state_mutex_);
      pending_ble_name_pub_ = true;
      return false;
    }
    if (!commit_request()) {
      std::string current_name;
      {
        MutexGuard lock(state_mutex_);
        current_name = advertised_name_();
      }
      const int restore_rc = ble_svc_gap_device_name_set(current_name.c_str());
      if (restore_rc != 0) {
        ESP_LOGW(TAG, "Restoring accepted GAP name after a superseded write failed rc=%d",
                 restore_rc);
      }
      return false;
    }
  } else if (!commit_request()) {
    return false;
  }  // Release both mutexes before stopping advertising or terminating a link.

  // Stop advertising (non-blocking)
  if (should_stop_adv) {
    stop_advertising();
  }

  // Terminate any active connection (non-blocking)
  if (conn_handle_copy != BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGI(TAG, "Terminating connection for MAC rotation");
    int rc = 0;
    bool terminate_started = false;
    {
      BleOpGuard ble_lock(ble_op_mutex_, pdMS_TO_TICKS(TimingConfig::BLE_OP_LOCK_WAIT_MS));
      if (ble_lock.acquired()) {
        rc = ble_gap_terminate(conn_handle_copy, BLE_ERR_REM_USER_CONN_TERM);
        // EALREADY means the asynchronous termination is already in progress;
        // keep the rotation request alive so on_disconnect() can advance it.
        terminate_started = (rc == 0 || rc == BLE_HS_EALREADY);
      } else {
        ESP_LOGW(TAG, "MAC rotation terminate: ble_op_mutex timeout");
      }
    }
    if (terminate_started) {
      // on_disconnect() will advance the state machine to READY_TO_ROTATE
    } else {
      // Neither the mutex nor the terminate succeeded, so no disconnect callback
      // will fire and the state machine would hang in REQUESTED forever (loop()
      // only handles READY_TO_ROTATE). Abort the rotation
      // and restore advertising instead of getting stuck.
      if (rc != 0) {
        ESP_LOGW(TAG, "ble_gap_terminate during MAC rotation rc=%d", rc);
      }
      ESP_LOGW(TAG, "MAC rotation aborted (could not disconnect); restoring prior state");
      {
        MutexGuard lock(state_mutex_);
        if (rotation_generation != mac_rotation_generation_) return true;
        abort_rotation_locked_();
      }
      start_advertising();
    }
  } else {
    // No connection - safe to rotate immediately
    ESP_LOGD(TAG, "No active connection, ready to rotate MAC");
    MutexGuard lock(state_mutex_);
    if (rotation_generation != mac_rotation_generation_) return true;
    mac_rotation_state_ = MacRotationState::READY_TO_ROTATE;
    // loop() will handle the actual rotation
  }
  return true;
}

//======================== BLE name update ========================

bool IRKCaptureComponent::update_ble_name(const std::string& name) {
  // The known Keyboard default uses 13 bytes and already fits its scan response.
  // Preserve that exact value so it can be saved unchanged or restored after a
  // custom name. Other names retain the existing Samsung-compatible limit.
  bool keyboard_default;
  {
    MutexGuard lock(state_mutex_);
    keyboard_default = ble_profile_ == BLEProfile::KEYBOARD && name == KEYBOARD_DEFAULT_NAME;
  }
  const std::string accepted_name = keyboard_default ? name : sanitize_ble_name(name);
  if (accepted_name.empty()) {
    // Nothing usable was entered. Substituting a default here would rename the
    // device, rotate its address and clear its bonds for what was a typo, so
    // keep the current name and put it back on display.
    MutexGuard lock(state_mutex_);
    pending_ble_name_pub_ = true;
    return false;
  }
  NameChange change { accepted_name, 0, 0, 0 };
  {
    MutexGuard lock(state_mutex_);
    if (advertised_name_() == accepted_name) {
      // Accepting the current value still supersedes an older generated-name
      // request. It needs no GAP write, disconnect, bond clear, or new rotation.
      ble_name_generation_++;
      identity_refresh_pending_ = false;
      generated_name_active_ = false;
      if (keyboard_default) keyboard_name_custom_ = false;
      pending_ble_name_pub_ = true;
      return true;
    }
    change.host_generation = host_generation_;
    change.rotation_generation = mac_rotation_generation_;
    change.name_generation = ble_name_generation_;
  }

  // The helper commits only after a successful GAP write and restores the
  // accepted GAP value if this write was superseded while the stack ran.
  return request_mac_rotation_(false, &change);
}

// Saves the profile for the next boot. Returns false, after logging why, if it was not stored.
bool persist_ble_profile(BLEProfile profile) {
  nvs_handle_t nvs_handle;
  const esp_err_t open_err = nvs_open("irk_capture", NVS_READWRITE, &nvs_handle);
  if (open_err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to open NVS to persist the BLE profile (%s)", esp_err_to_name(open_err));
    return false;
  }
  const char* step = "set";
  esp_err_t err = nvs_set_u8(nvs_handle, "ble_profile", static_cast<uint8_t>(profile));
  if (err == ESP_OK) {
    step = "commit";
    err = nvs_commit(nvs_handle);
  }
  nvs_close(nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to persist the BLE profile (%s: %s)", step, esp_err_to_name(err));
  }
  return err == ESP_OK;
}

void IRKCaptureComponent::set_ble_profile(BLEProfile profile) {
  BLEProfile old_profile;
  std::string current_name;
  {
    MutexGuard lock(state_mutex_);
    old_profile = ble_profile_;
    ble_profile_ = profile;
    current_name = ble_name_;
  }

  const char* profile_name = (profile == BLEProfile::HEART_SENSOR) ? "Heart Sensor" : "Keyboard";
  ESP_LOGI(TAG, "BLE profile changed to: %s", profile_name);

  // Only trigger changes if profile actually changed
  if (old_profile != profile) {
    // Persist profile to NVS before restart
    const bool persisted = persist_ble_profile(profile);

    // setup() reloads the profile from NVS on boot, so rebooting without a
    // successful write would land right back on the old profile while also
    // discarding the capture session. Roll the selection back instead and let
    // the user retry.
    if (!persisted) {
      {
        MutexGuard lock(state_mutex_);
        ble_profile_ = old_profile;
      }
      const char* old_name =
          (old_profile == BLEProfile::HEART_SENSOR) ? "Heart Sensor" : "Keyboard";
      ESP_LOGE(TAG, "BLE profile change not saved; staying on %s and skipping reboot", old_name);
      if (ble_profile_select_) ble_profile_select_->publish_state(old_name);
      return;
    }

    // Update the displayed BLE name based on profile
    if (profile == BLEProfile::KEYBOARD) {
      // Keyboard profile uses fixed name "Logitech K380"
      if (ble_name_text_) {
        ble_name_text_->publish_state(KEYBOARD_DEFAULT_NAME);
      }
    } else {
      // Heart Sensor profile restores the configured name
      if (ble_name_text_) {
        ble_name_text_->publish_state(current_name);
      }
    }

    // GATT database cannot be dynamically changed in NimBLE - restart required
    // Use ESPHome's safe_reboot to avoid watchdog timeout from blocking
    // vTaskDelay
    ESP_LOGW(TAG,
             "Profile change requires restart to update GATT database - "
             "scheduling safe reboot...");
    // This function publishes entities directly, so it must run on the main
    // task; the wizard defers its call here for that reason. The reboot is
    // still deferred so the caller finishes first: the select publishes its
    // new state, and a wizard request has already been answered.
    request_reboot_();
  }
}

}  // namespace irk_capture
}  // namespace esphome

#endif  // USE_ESP32
