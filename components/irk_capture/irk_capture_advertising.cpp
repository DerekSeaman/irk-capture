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

void IRKCaptureComponent::start_advertising() {
  if (!host_synced_) {
    ESP_LOGW(TAG, "Host not synced; cannot advertise");
    return;
  }

  // Enforce the single-connection invariant before touching the controller.
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
    BleOpGuard ble_lock(ble_op_mutex_, pdMS_TO_TICKS(TimingConfig::BLE_OP_LOCK_WAIT_MS));
    if (!ble_lock.acquired()) {
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
      operation = "GAP name update";
      rc = ble_svc_gap_device_name_set(name_copy.c_str());
    }

    if (rc == 0) {
      operation = "advertising data setup";
      rc = ble_gap_adv_set_fields(&fields);
    }

    // Set scan response data if needed (Keyboard profile)
    if (rc == 0 && use_scan_response) {
      operation = "scan response setup";
      rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    }

    if (rc == 0) {
      ble_gap_adv_params advp {};
      advp.conn_mode = BLE_GAP_CONN_MODE_UND;
      advp.disc_mode = BLE_GAP_DISC_MODE_GEN;

      // Faster advertising interval for quicker discovery in Samsung/Android
      // Settings UI Units are 0.625ms: 0x00A0 = 100ms, 0x00F0 = 150ms (default
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
    bool stale_rotation;
    {
      MutexGuard lock(state_mutex_);
      if (host_generation != host_generation_ || !host_synced_) return;
      stale_rotation = rotation_generation != mac_rotation_generation_;
      if (!stale_rotation) {
        advertising_ = (rc == 0);
        if (rc == 0) {
          recovered = advertising_start_attempts_ != 0;
          advertising_start_attempts_ = 0;
          advertising_failure_log_time_ = 0;
          adv_restart_time_ = 0;
          pending_adv_pub_ = true;
        }
      }
    }
    if (stale_rotation) {
      // A newer rotation can observe advertising_ before this start finishes.
      // Stop the obsolete start while still owning the BLE-operation lock;
      // otherwise it could leave the controller active throughout rotation.
      if (rc == 0) {
        const int stale_stop_rc = ble_gap_adv_stop();
        if (stale_stop_rc != 0 && stale_stop_rc != BLE_HS_EALREADY) {
          ESP_LOGW(TAG, "Stopping a superseded advertising start failed rc=%d", stale_stop_rc);
        }
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

void IRKCaptureComponent::stop_advertising() {
  int rc;
  bool still_active;
  bool lock_timeout = false;
  {
    BleOpGuard ble_lock(ble_op_mutex_, pdMS_TO_TICKS(TimingConfig::BLE_OP_LOCK_WAIT_MS));
    if (!ble_lock.acquired()) {
      lock_timeout = true;
    } else {
      uint32_t host_generation;
      {
        MutexGuard lock(state_mutex_);
        host_generation = host_generation_;
      }
      rc = ble_gap_adv_stop();
      still_active = ble_gap_adv_active() != 0;
      // Keep BLE control serialized through the state commit. Releasing the
      // BLE lock first permits a newer start to finish before this stale stop
      // writes false, making subsequent OFF requests skip the live radio.
      MutexGuard lock(state_mutex_);
      if (host_generation != host_generation_) return;
      advertising_ = still_active;
      adv_stop_retry_time_ = 0;
    }
  }
  if (lock_timeout) {
    bool first;
    {
      MutexGuard lock(state_mutex_);
      first = adv_stop_retry_time_ == 0;
      adv_stop_retry_time_ = deadline_after(now_ms(), TimingConfig::ADV_RETRY_MS);
    }
    if (first) {
      ESP_LOGW(TAG,
               "Advertising stop deferred: BLE operation lock busy; retrying every %" PRIu32 " ms",
               TimingConfig::ADV_RETRY_MS);
    }
    return;
  }
  if (rc != 0 && rc != BLE_HS_EALREADY && rc != BLE_HS_EINVAL) {
    ESP_LOGW(TAG, "ble_gap_adv_stop rc=%d", rc);
  }

  if (still_active) {
    ESP_LOGW(TAG, "Advertising stop requested but NimBLE still reports it active");
  } else {
    ESP_LOGI(TAG, "Advertising stopped");
  }
}

void IRKCaptureComponent::set_advertising_requested(bool requested) {
  bool actual;
  bool connected;
  {
    MutexGuard lock(state_mutex_);
    if (advertising_requested_ != requested) {
      advertising_start_attempts_ = 0;
      advertising_failure_log_time_ = 0;
    }
    advertising_requested_ = requested;
    if (!requested) {
      // An explicit OFF cancels all automatic restart paths.
      adv_restart_time_ = 0;
    }
    actual = advertising_;
    connected = connected_;
    // The switch is an authoritative desired-state control and therefore stays
    // ON while a peer is connected even though advertising is temporarily idle.
    pending_adv_pub_ = true;
  }

  if (!requested) {
    if (actual) stop_advertising();
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
    if (!advertising_requested_) {
      advertising_start_attempts_ = 0;
      advertising_failure_log_time_ = 0;
    }
    advertising_requested_ = true;
    pending_adv_pub_ = true;
  }
  // A capture between this transaction and the BLE call can request OFF.
  // start_advertising() rechecks intent so one-shot auto-stop still wins.
  start_advertising();
  return baseline;
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
