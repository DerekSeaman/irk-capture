// Advertising: building the payload, start, stop, retry and back-off, and
// the capture-start request.

#include "irk_capture_internal.h"
#include "irk_capture_gatt.h"

// ESP32-only implementation - requires Bluetooth hardware
#ifdef USE_ESP32

namespace esphome {
namespace irk_capture {

//======================== Advertising ========================

// Fills the advertising payload and, for the Keyboard profile, the scan
// response that carries the name. Both structs point into `name`, which must
// stay alive until they have been handed to NimBLE.
static void build_adv_fields(BLEProfile profile, const std::string& name,
                             struct ble_hs_adv_fields& fields,
                             struct ble_hs_adv_fields& rsp_fields) {
  const bool use_scan_response = profile == BLEProfile::KEYBOARD;
  // The Keyboard default has 13 bytes; custom names normally have at most 12.
  // Guard the conversion even if configuration or a future caller bypasses
  // normalization. A legacy packet carries 31 bytes and a name field has a
  // 2-byte header, so a scan response fits 29 bytes of name. The Heart Sensor
  // advertisement also carries flags (3), appearance (4) and one 16-bit UUID
  // (4), which leaves 18.
  const size_t max_name_len = use_scan_response ? 29 : 18;
  size_t raw_len = name.size();
  if (raw_len > max_name_len) {
    ESP_LOGW(TAG, "BLE name too long (%zu bytes), truncating to %zu", raw_len, max_name_len);
    raw_len = max_name_len;
  }
  const uint8_t name_len = (uint8_t) raw_len;
  const bool name_complete = raw_len == name.size();
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.appearance_is_present = 1;
  fields.num_uuids16 = 1;
  fields.uuids16_is_complete = 1;
  if (use_scan_response) {
    fields.appearance = 0x03C1;  // Keyboard
    fields.uuids16 = const_cast<ble_uuid16_t*>(&UUID_SVC_HID_BLE);
    // Put the name in the scan response's separate 31-byte budget.
    rsp_fields.name = (uint8_t*) name.c_str();
    rsp_fields.name_len = name_len;
    rsp_fields.name_is_complete = name_complete;
  } else {
    fields.appearance = APPEARANCE_HEART_RATE_SENSOR;
    fields.uuids16 = const_cast<ble_uuid16_t*>(&UUID_SVC_HR);
    fields.name = (uint8_t*) name.c_str();
    fields.name_len = name_len;
    fields.name_is_complete = name_complete;
  }
}

bool IRKCaptureComponent::adv_start_current_locked_(uint32_t host_generation,
                                                    uint32_t rotation_generation) const {
  // Caller holds state_mutex_. True while a start begun under these generations
  // is still wanted and nothing else owns the radio.
  return host_generation == host_generation_ && rotation_generation == mac_rotation_generation_ &&
         host_synced_ && advertising_requested_ && !connected_ && !bond_clear_pending_ &&
         mac_rotation_state_ == MacRotationState::IDLE;
}

void IRKCaptureComponent::start_advertising(bool host_task) {
  if (!host_synced_) {
    ESP_LOGW(TAG, "Host not synced; cannot advertise");
    return;
  }

  // Skip if advertising is OFF, a peer is connected, or maintenance is pending.
  BLEProfile current_profile;
  std::string name_copy;
  bool requested;
  bool connected;
  bool rotating;
  uint32_t host_generation;
  uint32_t rotation_generation;
  {
    MutexGuard lock(state_mutex_);
    requested = advertising_requested_;
    connected = connected_;
    rotating = mac_rotation_state_ != MacRotationState::IDLE || bond_clear_pending_;
    host_generation = host_generation_;
    rotation_generation = mac_rotation_generation_;
  }
  if (!requested) {
    ESP_LOGD(TAG, "Advertising start skipped: user intent is OFF");
    return;
  }
  if (connected || rotating) {
    ESP_LOGD(TAG, "Advertising start deferred until connection/BLE maintenance completes");
    return;
  }

  const char* profile_name = "";
  int rc = 0;
  const char* operation = "start";
  bool address_initialized = false;
  bool recovered = false;
  {
    BleOpGuard ble_lock(ble_op_mutex_,
                        host_task ? 0 : pdMS_TO_TICKS(TimingConfig::BLE_OP_LOCK_WAIT_MS));
    if (!ble_lock.acquired()) {
      if (host_task) {
        // The host task never waits. Hand the start to loop(), keeping any
        // deadline already set: a reconnect hold-off must still run out.
        {
          MutexGuard lock(state_mutex_);
          if (adv_start_current_locked_(host_generation, rotation_generation) &&
              adv_restart_time_ == 0) {
            adv_restart_time_ = deadline_after(now_ms(), 0);
          }
        }
        ESP_LOGD(TAG, "Advertising start handed to the main loop: BLE operation lock busy");
        return;
      }
      handle_advertising_failure_(BLE_HS_EBUSY, host_generation, "BLE operation lock");
      return;
    }
    bool address_ready;
    {
      MutexGuard lock(state_mutex_);
      if (!adv_start_current_locked_(host_generation, rotation_generation)) return;
      address_ready = random_address_ready_;
    }
    if (!address_ready) {
      uint8_t rnd[6];
      make_static_random_addr(rnd);
      operation = "random address setup";
      rc = ble_hs_id_set_rnd(rnd);
      {
        MutexGuard lock(state_mutex_);
        if (host_generation != host_generation_ ||
            rotation_generation != mac_rotation_generation_ || !host_synced_)
          return;
        random_address_ready_ = rc == 0;
        if (rc == 0) commit_generated_name_(rnd);
      }
      address_initialized = rc == 0;
    }

    // An address replacement during host recovery may have changed a generated
    // name. Snapshot it only after that commit, then keep this string alive
    // until both advertising payloads and the GAP name have been submitted.
    {
      MutexGuard lock(state_mutex_);
      if (!adv_start_current_locked_(host_generation, rotation_generation)) return;
      current_profile = ble_profile_;
      name_copy = advertised_name_();
    }
    struct ble_hs_adv_fields fields {};
    struct ble_hs_adv_fields rsp_fields {};
    const bool use_scan_response = current_profile == BLEProfile::KEYBOARD;
    profile_name = use_scan_response ? "Keyboard" : "Heart Sensor";
    build_adv_fields(current_profile, name_copy, fields, rsp_fields);

    if (rc == 0) {
      // Defensive stop - ensure clean GAP state before starting.
      int stop_rc = ble_gap_adv_stop();
      if (stop_rc != 0 && stop_rc != BLE_HS_EALREADY && stop_rc != BLE_HS_EINVAL) {
        ESP_LOGD(TAG, "ble_gap_adv_stop before start rc=%d", stop_rc);
      }
      if (ble_gap_adv_active()) {
        // A failed stop can leave the previous advertisement running. Do not
        // rewrite its payload or treat a subsequent start error as radio OFF.
        operation = "stop before restart";
        rc = stop_rc != 0 ? stop_rc : BLE_HS_EBUSY;
      } else {
        operation = "GAP name update";
        rc = ble_svc_gap_device_name_set(name_copy.c_str());
      }
    }

    if (rc == 0) {
      operation = "advertising data setup";
      rc = ble_gap_adv_set_fields(&fields);
    }

    if (rc == 0 && use_scan_response) {
      operation = "scan response setup";
      rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    }

    if (rc == 0) {
      ble_gap_adv_params advp {};
      advp.conn_mode = BLE_GAP_CONN_MODE_UND;
      advp.disc_mode = BLE_GAP_DISC_MODE_GEN;

      // Faster advertising interval for quicker discovery in Samsung/Android
      // Settings UI. Units are 0.625ms: 0x00A0 = 100ms, 0x00F0 = 150ms (default
      // NimBLE is ~1.28s)
      advp.itvl_min = 0x00A0;
      advp.itvl_max = 0x00F0;

      // Use explicit RANDOM address type: our static random address, set
      // above on the first start and by handle_mac_rotation_() afterwards.
      // Avoids Samsung One UI 7 "Maximum Restrictions" filtering RPA
      // addresses as tracking risks
      constexpr uint8_t own_addr_type = BLE_OWN_ADDR_RANDOM;
      operation = "start";
      rc = ble_gap_adv_start(own_addr_type, nullptr, BLE_HS_FOREVER, &advp,
                             IRKCaptureComponent::gap_event_handler, this);
    }
    const bool actual_advertising = ble_gap_adv_active() != 0;
    bool stale_rotation;
    {
      MutexGuard lock(state_mutex_);
      if (host_generation != host_generation_ || !host_synced_) return;
      stale_rotation = rotation_generation != mac_rotation_generation_;
      if (!stale_rotation) {
        advertising_ = actual_advertising;
        if (rc == 0) {
          recovered = advertising_start_attempts_ != 0;
          advertising_start_attempts_ = 0;
          advertising_failure_log_time_ = 0;
          adv_restart_time_ = 0;
          adv_hold_off_stop_ = 0;  // This start supersedes any hold-off.
          pending_adv_pub_ = true;
        }
      }
    }
    if (stale_rotation) {
      // A newer rotation can observe advertising_ before this start finishes.
      // Stop the obsolete start while still owning the BLE-operation lock;
      // otherwise it could leave the controller active throughout rotation.
      if (actual_advertising) {
        const int stale_stop_rc = ble_gap_adv_stop();
        if (stale_stop_rc != 0 && stale_stop_rc != BLE_HS_EALREADY) {
          ESP_LOGW(TAG, "Stopping a superseded advertising start failed rc=%d", stale_stop_rc);
        }
      }
      const bool still_active = ble_gap_adv_active() != 0;
      {
        MutexGuard lock(state_mutex_);
        if (host_generation == host_generation_ && host_synced_) advertising_ = still_active;
      }
      return;
    }
  }
  // Log outside both component locks, including when address setup succeeds
  // but a later step fails. This keeps MAC diagnostics available during recovery.
  if (address_initialized) log_mac("Effective");
  if (rc != 0) {
    handle_advertising_failure_(rc, host_generation, operation);
  } else {
    if (recovered) ESP_LOGI(TAG, "Advertising recovered; normal operation resumed");
    ESP_LOGI(TAG, "Advertising with profile: %s", profile_name);
    publish_effective_mac();
  }
}

void IRKCaptureComponent::handle_advertising_failure_(int rc, uint32_t host_generation,
                                                      const char* operation) {
  uint8_t attempts;
  uint32_t delay_ms;
  bool entering_slow_recovery;
  bool log_failure;
  const uint32_t now = now_ms();
  {
    MutexGuard lock(state_mutex_);
    if (!adv_start_current_locked_(host_generation, mac_rotation_generation_)) return;
    entering_slow_recovery = advertising_start_attempts_ == TimingConfig::ADV_FAST_ATTEMPTS - 1;
    // Saturate the counter: indefinite slow recovery must never wrap it back
    // into a burst of fast retries or overflow the exponential shift below.
    if (advertising_start_attempts_ < TimingConfig::ADV_FAST_ATTEMPTS) {
      advertising_start_attempts_++;
    }
    attempts = advertising_start_attempts_;
    delay_ms = attempts >= TimingConfig::ADV_FAST_ATTEMPTS
                   ? TimingConfig::ADV_SLOW_RETRY_MS
                   : TimingConfig::ADV_RETRY_MS << (attempts - 1);
    adv_restart_time_ = deadline_after(now, delay_ms);
    log_failure = attempts < TimingConfig::ADV_FAST_ATTEMPTS || entering_slow_recovery ||
                  now - advertising_failure_log_time_ >= TimingConfig::ADV_FAILURE_LOG_INTERVAL_MS;
    if (log_failure) advertising_failure_log_time_ = now;
    // The switch remains the user's requested state throughout recovery.
    pending_adv_pub_ = true;
  }
  if (entering_slow_recovery) {
    ESP_LOGE(TAG,
             "Advertising %s failed rc=%d after %u attempts; "
             "switch remains ON, retrying every %" PRIu32 " seconds",
             operation, rc, attempts, TimingConfig::ADV_SLOW_RETRY_MS / 1000);
  } else if (log_failure) {
    ESP_LOGW(TAG, "Advertising %s failed rc=%d; retry in %" PRIu32 " ms", operation, rc, delay_ms);
  }
}

bool IRKCaptureComponent::stop_advertising(bool host_task, uint32_t hold_off) {
  int rc;
  bool still_active;
  bool was_active = false;
  bool lock_timeout = false;
  {
    BleOpGuard ble_lock(ble_op_mutex_,
                        host_task ? 0 : pdMS_TO_TICKS(TimingConfig::BLE_OP_LOCK_WAIT_MS));
    if (!ble_lock.acquired()) {
      lock_timeout = true;
    } else {
      uint32_t host_generation;
      {
        MutexGuard lock(state_mutex_);
        // Recheck under the BLE lock: a start, OFF or reset since the hold-off
        // was set means this stop would turn off advertising nobody holds off.
        if (hold_off != 0 && !hold_off_pending_locked_(hold_off)) {
          if (adv_hold_off_stop_ == hold_off) adv_hold_off_stop_ = 0;
          return false;
        }
        host_generation = host_generation_;
      }
      was_active = ble_gap_adv_active() != 0;
      rc = was_active ? ble_gap_adv_stop() : BLE_HS_EALREADY;
      still_active = ble_gap_adv_active() != 0;
      // Keep BLE control serialized through the state commit. Releasing the
      // BLE lock first permits a newer start to finish before this stale stop
      // writes false, making subsequent OFF requests skip the live radio.
      MutexGuard lock(state_mutex_);
      if (host_generation != host_generation_) return false;
      advertising_ = still_active;
      adv_stop_retry_time_ =
          still_active ? deadline_after(now_ms(), TimingConfig::ADV_RETRY_MS) : 0;
      if (!still_active) adv_hold_off_stop_ = 0;
    }
  }
  if (lock_timeout) {
    bool first;
    {
      MutexGuard lock(state_mutex_);
      if (hold_off != 0) {
        // Hand the stop to loop() only for the hold-off that asked for it.
        if (!hold_off_pending_locked_(hold_off)) return false;
        adv_hold_off_stop_ = hold_off;
      }
      first = adv_stop_retry_time_ == 0;
      // The host task never waits, so loop() takes it over on its next pass.
      adv_stop_retry_time_ = deadline_after(now_ms(), host_task ? 0 : TimingConfig::ADV_RETRY_MS);
    }
    if (first) {
      ESP_LOGW(TAG,
               "Advertising stop deferred: BLE operation lock busy; retrying every %" PRIu32 " ms",
               TimingConfig::ADV_RETRY_MS);
    }
    return false;
  }
  if (rc != 0 && rc != BLE_HS_EALREADY && rc != BLE_HS_EINVAL) {
    ESP_LOGW(TAG, "ble_gap_adv_stop rc=%d", rc);
  }

  if (still_active) {
    ESP_LOGW(TAG, "Advertising stop requested but NimBLE still reports it active");
  } else if (was_active) {
    ESP_LOGI(TAG, "Advertising stopped");
  }
  return !still_active;
}

void IRKCaptureComponent::set_advertising_intent_locked_(bool requested) {
  if (advertising_requested_ != requested) {
    advertising_start_attempts_ = 0;
    advertising_failure_log_time_ = 0;
  }
  advertising_requested_ = requested;
  if (requested)
    capture_stop_requested_ = false;
  else
    adv_restart_time_ = 0;  // An explicit OFF cancels all automatic restart paths.
  // The switch stays ON while a peer is connected even though the radio is idle.
  pending_adv_pub_ = true;
}

void IRKCaptureComponent::set_advertising_requested(bool requested) {
  bool actual;
  bool connected;
  {
    MutexGuard lock(state_mutex_);
    set_advertising_intent_locked_(requested);
    actual = advertising_;
    connected = connected_;
  }

  if (!requested) {
    // OFF verifies GAP state even if a previous failure left the cache stale.
    // A lock timeout arms a retry independently of the cached advertising flag.
    if (actual || host_synced_) stop_advertising();
  } else if (!connected) {
    start_advertising();
  }
}

CaptureResult IRKCaptureComponent::get_capture_result() {
  MutexGuard lock(state_mutex_);
  return capture_result_;
}

CaptureResult IRKCaptureComponent::begin_capture() {
  CaptureResult baseline;
  {
    MutexGuard lock(state_mutex_);
    baseline = capture_result_;
    // Only a connection made after this start counts for the wizard.
    baseline.connection_generation = connection_generation_;
    // An earlier attempt's outcome, held so it stays visible for a few
    // seconds, would otherwise open this attempt with its "failed" or
    // "no_irk". The wizard tells the user to go back and retry, which is
    // often quicker than the hold.
    capture_status_ = CaptureStatus::NONE;
    status_result_hold_until_ = 0;
    set_advertising_intent_locked_(true);
  }
  // A capture between this transaction and the BLE call can request OFF.
  // start_advertising() rechecks intent so one-shot auto-stop still wins.
  start_advertising();
  return baseline;
}

void IRKCaptureComponent::cancel_capture() {
  uint16_t handle;
  uint32_t generation;
  {
    MutexGuard lock(state_mutex_);
    set_advertising_intent_locked_(false);
    capture_stop_requested_ = true;
    handle = conn_handle_;
    generation = connection_generation_;
    // Disconnect fallback reads can still find a stored key. Invalidate those
    // results before the HTTP reply lets the browser save the next label.
    cancelled_capture_generation_ = generation;
  }
  if (handle != BLE_HS_CONN_HANDLE_NONE)
    terminate_bounded_(handle, generation, "after wizard cancellation");
}

bool IRKCaptureComponent::is_advertising() {
  MutexGuard lock(state_mutex_);
  return advertising_;
}

bool IRKCaptureComponent::is_advertising_requested() {
  MutexGuard lock(state_mutex_);
  return advertising_requested_;
}

}  // namespace irk_capture
}  // namespace esphome

#endif  // USE_ESP32
