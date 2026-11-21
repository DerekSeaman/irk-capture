// GAP event handlers and their dispatcher, and the connect/disconnect
// bookkeeping they call. All of it runs on the NimBLE task.

#include "irk_capture_internal.h"
#include "irk_capture_gatt.h"

// ESP32-only implementation - requires Bluetooth hardware
#ifdef USE_ESP32

namespace esphome {
namespace irk_capture {

// Connection parameters with their units: the interval counts 1.25 ms steps
// and the supervision timeout 10 ms steps; latency is a count of events.
static inline void log_conn_params(const char* label, uint16_t itvl, uint16_t latency,
                                   uint16_t supervision_timeout) {
  const uint32_t itvl_centi_ms = (uint32_t) itvl * 125;
  ESP_LOGD(TAG,
           "%s: interval=%u (%" PRIu32 ".%02" PRIu32
           " ms) latency=%u supervision_timeout=%u (%" PRIu32 " ms)",
           label, itvl, itvl_centi_ms / 100, itvl_centi_ms % 100, latency, supervision_timeout,
           (uint32_t) supervision_timeout * 10);
}

static inline void log_sec_state(const char* label, const struct ble_gap_conn_desc& d) {
  ESP_LOGI(TAG, "%s: enc=%d bonded=%d auth=%d key_size=%u", label, d.sec_state.encrypted,
           d.sec_state.bonded, d.sec_state.authenticated, d.sec_state.key_size);
}

// NimBLE does not report a bare SMP reason code in enc_change.status. It encodes
// the failing side into the value: BLE_HS_SM_US_ERR(reason) == 0x400 + reason for
// a local rejection, BLE_HS_SM_PEER_ERR(reason) == 0x500 + reason for a peer one.
// Comparing raw reason numbers (1..15) against those encoded values never matches,
// so decode the family and reason before describing or acting on a failure.
struct SmFailure {
  bool is_sm { false };
  bool from_peer { false };
  uint8_t reason { 0 };
};

static SmFailure decode_sm_failure(int status) {
  SmFailure out;
  if (status >= BLE_HS_ERR_SM_PEER_BASE && status < BLE_HS_ERR_HW_BASE) {
    out = { true, true, (uint8_t) (status - BLE_HS_ERR_SM_PEER_BASE) };
  } else if (status >= BLE_HS_ERR_SM_US_BASE && status < BLE_HS_ERR_SM_PEER_BASE) {
    out = { true, false, (uint8_t) (status - BLE_HS_ERR_SM_US_BASE) };
  }
  return out;
}

static const char* sm_reason_str(uint8_t reason) {
  switch (reason) {
    case BLE_SM_ERR_PASSKEY:
      return "Passkey Entry Failed";
    case BLE_SM_ERR_OOB:
      return "OOB Not Available";
    case BLE_SM_ERR_AUTHREQ:
      return "Authentication Requirements";
    case BLE_SM_ERR_CONFIRM_MISMATCH:
      return "Confirm Value Failed";
    case BLE_SM_ERR_PAIR_NOT_SUPP:
      return "Pairing Not Supported";
    case BLE_SM_ERR_ENC_KEY_SZ:
      return "Encryption Key Size";
    case BLE_SM_ERR_CMD_NOT_SUPP:
      return "Command Not Supported";
    case BLE_SM_ERR_UNSPECIFIED:
      return "Unspecified Reason";
    case BLE_SM_ERR_REPEATED:
      return "Repeated Attempts";
    case BLE_SM_ERR_INVAL:
      return "Invalid Parameters";
    case BLE_SM_ERR_DHKEY:
      return "DHKey Check Failed";
    case BLE_SM_ERR_NUMCMP:
      return "Numeric Comparison Failed";
    case BLE_SM_ERR_ALREADY:
      return "BR/EDR Pairing in Progress";
    case BLE_SM_ERR_CROSS_TRANS:
      return "Cross-transport Key Derivation";
    case BLE_SM_ERR_KEY_REJ:
      return "Key Rejected";
    default:
      return "Unknown";
  }
}

// GAP event handlers (forward decls)
class IRKCaptureComponent;
int handle_gap_connect(IRKCaptureComponent* self, struct ble_gap_event* ev);
int handle_gap_disconnect(IRKCaptureComponent* self, struct ble_gap_event* ev);
int handle_gap_enc_change(IRKCaptureComponent* self, struct ble_gap_event* ev);
int handle_gap_repeat_pairing(IRKCaptureComponent* self, struct ble_gap_event* ev);

static void log_conn_desc(uint16_t conn_handle) {
  struct ble_gap_conn_desc d;
  if (ble_gap_conn_find(conn_handle, &d) == 0) {
    // Security state
    log_sec_state("sec", d);

    // Addresses
    ESP_LOGI(TAG, "peer ota=%s type=%d", addr_to_str(d.peer_ota_addr).c_str(),
             d.peer_ota_addr.type);
    ESP_LOGI(TAG, "peer id =%s type=%d", addr_to_str(d.peer_id_addr).c_str(), d.peer_id_addr.type);

    // Connection parameters (helpful for Android watch debugging)
    // supervision_timeout is the link supervision timeout per BLE spec
    log_conn_params("conn params", d.conn_itvl, d.conn_latency, d.supervision_timeout);

    // Role and features
    ESP_LOGD(TAG, "role=%s our_ota=%s", d.role == BLE_GAP_ROLE_MASTER ? "master" : "slave",
             addr_to_str(d.our_ota_addr).c_str());
  }
}

//======================== GAP event handlers (extracted)
//========================
/*
GAP Event Handler Return Value Semantics:
-    Return 0: Event handled successfully, continue normal NimBLE processing
-    Return BLE_GAP_REPEAT_PAIRING_RETRY: Delete peer bond and retry pairing
(only valid for REPEAT_PAIRING event)
-    Return BLE_GAP_REPEAT_PAIRING_IGNORE: Ignore repeat pairing request (only
valid for REPEAT_PAIRING event)
-    Other non-zero values: Error occurred, but NimBLE will continue (logged but
not fatal)

THREADING: These handlers run in NimBLE task context, NOT ESPHome main task.
Avoid blocking operations and heavy computation in these functions.
*/

/**
 * @brief Handles BLE_GAP_EVENT_CONNECT - called when peer connects or
 * connection fails
 * @param self Pointer to IRKCaptureComponent instance
 * @param ev GAP event structure containing connection details
 * @return 0 (always - connection success/failure is logged only)
 *
 * On successful connection, initiates security/pairing via on_connect().
 * On failure, restarts advertising to allow retry.
 */
int handle_gap_connect(IRKCaptureComponent* self, struct ble_gap_event* ev) {
  if (ev->connect.status == 0) {
    ESP_LOGI(TAG, "Connection established successfully");
    self->on_connect(ev->connect.conn_handle);
  } else {
    ESP_LOGW(TAG, "Connection failed: status=%d (0x%02X)", ev->connect.status, ev->connect.status);

    // NimBLE can report a failed CONNECT for a link that is still up (for
    // example when the controller cannot read the peer's features). Nothing
    // tracks that link, and with one connection allowed it would block
    // advertising until the peer dropped it, so close it here. Its disconnect
    // is ignored as a non-active handle; the advertising retry below picks up
    // once the slot is free. EAGAIN comes from NimBLE's own broken-connection
    // path, where the link is already down, so it needs nothing.
    struct ble_gap_conn_desc desc {};
    if (ev->connect.status != BLE_HS_EAGAIN &&
        ble_gap_conn_find(ev->connect.conn_handle, &desc) == 0) {
      int term_rc;
      {
        BleOpGuard ble_lock(self->ble_op_mutex_);
        term_rc = ble_gap_terminate(ev->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
      }
      if (term_rc == 0 || term_rc == BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "Failed connection handle=%u is still up; disconnecting it",
                 ev->connect.conn_handle);
      } else {
        ESP_LOGW(TAG, "Failed connection handle=%u is still up and could not be closed rc=%d",
                 ev->connect.conn_handle, term_rc);
      }
    }

    // Thread-safe advertising state reset
    {
      MutexGuard lock(self->state_mutex_);
      self->advertising_ = false;
    }

    // Restart advertising (will set flag with mutex internally)
    self->start_advertising();
  }
  return 0;
}

static const char* subscribe_reason_str(uint8_t reason) {
  switch (reason) {
    case BLE_GAP_SUBSCRIBE_REASON_WRITE:
      return "peer wrote CCCD";
    case BLE_GAP_SUBSCRIBE_REASON_TERM:
      return "connection ended";
    case BLE_GAP_SUBSCRIBE_REASON_RESTORE:
      return "restored from bond";
    default:
      return "unknown";
  }
}

// Names the characteristic behind a subscription's value handle. Handles are
// looked up on demand: they depend on the profile's GATT table and are only
// final after ble_gatts_start(), and subscriptions are rare enough that the
// lookups cost nothing that matters.
static const char* gatt_attr_name(uint16_t attr_handle) {
  static const ble_uuid16_t kSvcGatt = BLE_UUID16_INIT(0x1801);
  static const ble_uuid16_t kChrServiceChanged = BLE_UUID16_INIT(0x2A05);
  static const struct {
    const ble_uuid_t* svc;
    const ble_uuid_t* chr;
    const char* name;
  } kChrs[] = {
    { &kSvcGatt.u, &kChrServiceChanged.u, "Service Changed" },
    { &UUID_SVC_HR.u, &UUID_CHR_HR_MEAS.u, "Heart Rate Measurement" },
    { &UUID_SVC_BAS.u, &UUID_CHR_BATT_LVL.u, "Battery Level" },
    { &UUID_SVC_DEVINFO.u, &UUID_CHR_MANUF.u, "Manufacturer Name" },
    { &UUID_SVC_DEVINFO.u, &UUID_CHR_MODEL.u, "Model Number" },
    { &UUID_SVC_HID_BLE.u, &UUID_CHR_HID_PROTO.u, "HID Protocol Mode" },
    { &UUID_SVC_PROT.u, &UUID_CHR_PROT.u, "Protected" },
  };
  for (const auto& c : kChrs) {
    uint16_t val_handle = 0;
    if (ble_gatts_find_chr(c.svc, c.chr, nullptr, &val_handle) == 0 && val_handle == attr_handle) {
      return c.name;
    }
  }
  return "unknown";
}

// NimBLE reports a disconnect reason as the HCI code plus 0x200. Names the
// reasons this component meets; anything else is shown by number only.
static const char* disconnect_reason_str(int reason) {
  if (reason < 0x200) return "";
  switch (reason - 0x200) {
    case 0x08:
      return "connection timeout";
    case 0x13:
      return "remote user terminated";
    case 0x16:
      return "terminated by local host";
    case 0x22:
      return "LL response timeout";
    case 0x3B:
      return "unacceptable connection parameters";
    case 0x3D:
      return "MIC failure";
    case 0x3E:
      return "connection failed to establish";
    default:
      return "";
  }
}

/**
 * @brief Handles BLE_GAP_EVENT_DISCONNECT - called when peer disconnects
 * @param self Pointer to IRKCaptureComponent instance
 * @param ev GAP event structure containing disconnect reason
 * @return 0 (always)
 *
 * If the connection has not produced a result yet, reads the peer's bond once
 * more and schedules a delayed read (+800ms) when that is inconclusive. Then
 * restarts advertising, unless the capture limit was reached, the user turned
 * advertising off, or a restart is already scheduled after a hold-off.
 */
int handle_gap_disconnect(IRKCaptureComponent* self, struct ble_gap_event* ev) {
  // Use the connection descriptor embedded in the disconnect event directly
  // (ble_gap_conn_find may fail after disconnect since NimBLE removes the
  // descriptor). The post-disconnect timer's peer is set by
  // schedule_post_disconnect_check() below, so no separate cache is needed here.
  const struct ble_gap_conn_desc& d = ev->disconnect.conn;
  uint32_t connection_generation = 0;
  bool already_observed = false;
  bool pairing_completed = false;
  CaptureOrigin origin = CaptureOrigin::UNKNOWN;
  uint32_t connected_since = 0;
  {
    MutexGuard lock(self->state_mutex_);
    if (self->connected_ && self->conn_handle_ == d.conn_handle) {
      connection_generation = self->connection_generation_;
      connected_since = self->pairing_start_time_;
      pairing_completed = self->enc_ready_;
      origin = self->connection_origin_;
      const std::string peer_addr = addr_to_str(d.peer_id_addr);
      for (const auto& entry : self->irk_cache_) {
        if (entry.mac_addr == peer_addr && entry.addr_type == d.peer_id_addr.type &&
            entry.last_observed_generation == connection_generation) {
          already_observed = true;
          break;
        }
      }
    }
  }
  // Logged after the state read so it can say how long the link was up; a
  // short link that never encrypted points at a different fault than a timeout.
  const char* reason_text = disconnect_reason_str(ev->disconnect.reason);
  if (connected_since != 0) {
    ESP_LOGI(TAG, "Disconnect reason=%d (0x%02X)%s%s after %" PRIu32 " ms connected (encrypted=%d)",
             ev->disconnect.reason, ev->disconnect.reason, reason_text[0] != '\0' ? ": " : "",
             reason_text, elapsed_ms(now_ms(), connected_since), (int) pairing_completed);
  } else {
    ESP_LOGI(TAG, "Disconnect reason=%d (0x%02X)%s%s", ev->disconnect.reason, ev->disconnect.reason,
             reason_text[0] != '\0' ? ": " : "", reason_text);
  }

  if (!self->on_disconnect(d.conn_handle)) {
    ESP_LOGW(TAG, "Ignoring disconnect for non-active handle=%u", d.conn_handle);
    return 0;
  }

  // ENC_CHANGE normally captures first. Only use the disconnect store read as
  // a fallback when this connection has not produced a valid IRK yet.
  bool needs_delayed_check = !already_observed;
  if (!already_observed) {
    struct ble_store_value_sec bond {};
    int rc = 0;
    const BondRead read = self->read_peer_irk_(d.peer_id_addr, bond, &rc);
    if (read == BondRead::NO_BOND) {
      ESP_LOGD(TAG, "No bond for peer (ENOENT)");
    } else if (read == BondRead::READ_ERROR) {
      ESP_LOGW(TAG, "ble_store_read_peer_sec rc=%d", rc);
    } else if (read == BondRead::IRK) {
      std::string irk_hex = to_hex_rev(bond.irk, sizeof(bond.irk));
      publish_and_log_irk(self, d.peer_id_addr, irk_hex, "DISC_IMMEDIATE", connection_generation,
                          origin);
      needs_delayed_check = false;
    } else {
      log_no_irk_for_peer(d.peer_id_addr);
      // A public identity alone cannot establish whether a device uses privacy.
      // Only a completed pairing with a stored bond lacking an IRK is conclusive.
      if (read == BondRead::NO_IRK && pairing_completed) {
        self->publish_no_irk_(d.peer_id_addr, connection_generation, pairing_completed, origin);
        needs_delayed_check = false;
      }
    }
  }

  if (needs_delayed_check) {
    self->schedule_post_disconnect_check(d.peer_id_addr, connection_generation, pairing_completed,
                                         origin);
  }

  // Thread-safe advertising state update
  {
    MutexGuard lock(self->state_mutex_);
    self->advertising_ = false;
  }

  // Check if we should stop advertising
  // THREAD-SAFE: Snapshot capture counts and restart schedule under mutex
  bool should_stop_adv = false;
  bool requested;
  bool restart_scheduled;
  {
    MutexGuard lock(self->state_mutex_);
    should_stop_adv = self->capture_limit_reached_locked_();
    requested = self->advertising_requested_;
    restart_scheduled = self->adv_restart_time_ != 0;
  }

  if (should_stop_adv) {
    // Don't restart - max captures reached
    ESP_LOGI(TAG, "Unique-device limit reached - stopping advertising");
    self->set_advertising_requested(false);
  } else if (!requested) {
    ESP_LOGD(TAG, "Advertising remains off by user request");
  } else if (restart_scheduled) {
    ESP_LOGI(TAG, "Advertising restart already scheduled after a hold-off");
  } else {
    if (self->continuous_mode_) {
      ESP_LOGI(TAG, "Continuous mode: restarting advertising for next device");
    }
    self->start_advertising();
  }
  return 0;
}

/**
 * @brief Handles BLE_GAP_EVENT_ENC_CHANGE - called when encryption/pairing
 * completes or fails
 * @param self Pointer to IRKCaptureComponent instance
 * @param ev GAP event structure containing encryption status
 * @return 0 (always)
 *
 * On success (status=0): reads the peer's bond at once. A valid IRK is
 * published and the link closed; a bond without an IRK is reported as "no IRK"
 * and the link closed; otherwise a delayed read is scheduled (+5s).
 * On failure: logs the decoded reason, deletes only this peer's bond and closes
 * the link so the peer pairs afresh. It does not retry security; the single
 * retry in retry_security_if_needed() is time-based.
 */
int handle_gap_enc_change(IRKCaptureComponent* self, struct ble_gap_event* ev) {
  uint32_t connection_generation = 0;
  CaptureOrigin origin = CaptureOrigin::UNKNOWN;
  uint32_t connected_since = 0;
  {
    MutexGuard lock(self->state_mutex_);
    if (self->connected_ && self->conn_handle_ == ev->enc_change.conn_handle) {
      connection_generation = self->connection_generation_;
      origin = self->connection_origin_;
      connected_since = self->pairing_start_time_;
    }
  }
  // Time from connect to the encryption result, for tuning the pairing
  // timeouts against slow peers.
  if (connected_since != 0) {
    ESP_LOGI(TAG, "ENC_CHANGE status=%d (0x%02X) %" PRIu32 " ms after connect",
             ev->enc_change.status, ev->enc_change.status, elapsed_ms(now_ms(), connected_since));
  } else {
    ESP_LOGI(TAG, "ENC_CHANGE status=%d (0x%02X)", ev->enc_change.status, ev->enc_change.status);
  }

  // Log encryption failure reasons for debugging (especially Android pairing).
  if (ev->enc_change.status != 0) {
    const SmFailure failure = decode_sm_failure(ev->enc_change.status);
    if (failure.is_sm) {
      ESP_LOGW(TAG, "ENC_CHANGE failed: %s; rejected by %s (reason=0x%02X status=%d)",
               sm_reason_str(failure.reason), failure.from_peer ? "peer" : "us", failure.reason,
               ev->enc_change.status);
    } else {
      ESP_LOGW(TAG, "ENC_CHANGE failed: status=%d (not a Security Manager failure)",
               ev->enc_change.status);
    }
  }

  if (ev->enc_change.status == 0) {
    ESP_LOGI(TAG, "Encryption established; attempting immediate IRK capture");
    {
      MutexGuard lock(self->state_mutex_);
      // Encryption on a link this component is not tracking (a failed CONNECT
      // whose link stayed up) must not mark the active connection as secured.
      if (self->connected_ && self->conn_handle_ == ev->enc_change.conn_handle) {
        self->enc_ready_ = true;
        self->enc_time_ = now_ms();
        // Initialize irk_last_try_ms_ to now so the ENC_TRY_INTERVAL_MS
        // guard in poll_irk_if_due() is correctly enforced on the first poll
        // attempt. Without this, (now - 0) is huge and the interval check is
        // bypassed immediately.
        self->irk_last_try_ms_ = now_ms();
      }
    }

    // Immediate store read using identity address
    struct ble_gap_conn_desc d {};
    const int desc_rc = ble_gap_conn_find(ev->enc_change.conn_handle, &d);
    if (desc_rc == 0) {
      // The connect-time sec line is always all zeros; this one shows what
      // pairing actually produced.
      log_sec_state("sec after pairing", d);
      struct ble_store_value_sec bond {};
      int rc = 0;
      // No default: a new BondRead value must choose its handling here.
      switch (self->read_peer_irk_(d.peer_id_addr, bond, &rc)) {
        case BondRead::NO_BOND:
          ESP_LOGD(TAG, "No bond for peer yet (ENOENT); scheduling late check");
          self->schedule_late_enc_check(d.peer_id_addr, connection_generation, origin);
          break;
        case BondRead::READ_ERROR:
          ESP_LOGW(TAG, "ble_store_read_peer_sec rc=%d; scheduling late check", rc);
          self->schedule_late_enc_check(d.peer_id_addr, connection_generation, origin);
          break;
        case BondRead::INVALID_IRK:
          ESP_LOGW(TAG, "IRK failed validation (all-zero or all-FF); scheduling late check");
          self->schedule_late_enc_check(d.peer_id_addr, connection_generation, origin);
          break;
        case BondRead::IRK: {
          std::string irk_hex = to_hex_rev(bond.irk, sizeof(bond.irk));
          publish_and_log_irk(self, d.peer_id_addr, irk_hex, "ENC_CHANGE", connection_generation,
                              origin);
          // Tested working behavior: terminate immediately after successful ENC +
          // IRK capture
          int term_rc;
          {
            BleOpGuard ble_lock(self->ble_op_mutex_);
            term_rc = ble_gap_terminate(ev->enc_change.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
          }
          log_terminate_result(term_rc, "after ENC capture");
          break;
        }
        case BondRead::NO_IRK:
          // NimBLE has completed key distribution before reporting ENC_CHANGE
          // success. Preserve this outcome before timeout cleanup can delete it.
          self->publish_no_irk_(d.peer_id_addr, connection_generation, true, origin);
          self->close_after_no_irk_(ev->enc_change.conn_handle, connection_generation,
                                    "after ENC no-IRK result");
          break;
      }
    } else {
      ESP_LOGW(TAG,
               "ENC_CHANGE succeeded but conn desc lookup failed rc=%d; "
               "disconnect fallback will check the peer's bond",
               desc_rc);
    }
  } else {
    // Encryption failed - delete ONLY this peer's bond (not all bonds) and
    // terminate. Clearing the entire store on any failure lets a single
    // misbehaving/hostile peer wipe bond state for every other device, so we
    // scope the cleanup to the failing peer to force it to pair fresh.
    struct ble_gap_conn_desc d {};
    if (ble_gap_conn_find(ev->enc_change.conn_handle, &d) == 0) {
      ESP_LOGW(TAG, "ENC_CHANGE failed status=%d; clearing bond for %s", ev->enc_change.status,
               addr_to_str(d.peer_id_addr).c_str());
      const int delete_rc = ble_store_util_delete_peer(&d.peer_id_addr);
      if (delete_rc != 0 && delete_rc != BLE_HS_ENOENT) {
        // A bond that stays makes this peer fail encryption on every reconnect.
        ESP_LOGW(TAG, "Bond for %s could not be cleared rc=%d; forget this device on the peer",
                 addr_to_str(d.peer_id_addr).c_str(), delete_rc);
      }
    } else {
      ESP_LOGW(TAG, "ENC_CHANGE failed status=%d; conn desc not found, no bond cleared",
               ev->enc_change.status);
    }
    self->report_capture_failure_(connection_generation);
    int term_rc;
    {
      BleOpGuard ble_lock(self->ble_op_mutex_);
      term_rc = ble_gap_terminate(ev->enc_change.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    log_terminate_result(term_rc, "after ENC failure");

    // On a DHKey check failure, hold off the next advertising restart for a
    // few seconds (local behaviour only; it does not reset the peer). Match the
    // reason from either side: the encoded status differs (0x40B local, 0x50B
    // peer).
    const SmFailure failure = decode_sm_failure(ev->enc_change.status);
    if (failure.is_sm && failure.reason == BLE_SM_ERR_DHKEY) {
      ESP_LOGW(TAG, "DHKey check failed; delaying the next advertising restart");
      MutexGuard lock(self->state_mutex_);
      self->adv_restart_time_ =
          deadline_after(now_ms(), TimingConfig::ADV_SUPPRESS_RESTART_DELAY_MS);
    }
  }
  return 0;
}

/**
 * @brief Handles BLE_GAP_EVENT_REPEAT_PAIRING - peer attempting to pair when
 * already bonded
 * @param self Pointer to IRKCaptureComponent instance; the connection is
 * marked as an explicit re-pair so its IRK is always published
 * @param ev GAP event structure containing repeat pairing details
 * @return BLE_GAP_REPEAT_PAIRING_RETRY after a successful stale-bond deletion;
 * BLE_GAP_REPEAT_PAIRING_IGNORE if recovery cannot be prepared safely
 *
 * This event occurs when a peer device has forgotten our bond but we still have
 * theirs stored. We delete our stale bond data and return RETRY to allow NimBLE
 * to complete fresh pairing. This is essential for recovering from out-of-sync
 * bond states between devices.
 */
int handle_gap_repeat_pairing(IRKCaptureComponent* self, struct ble_gap_event* ev) {
  // Portable: get the current connection descriptor from the handle
  struct ble_gap_conn_desc d {};
  int rc = ble_gap_conn_find(ev->repeat_pairing.conn_handle, &d);
  if (rc == 0) {
    const ble_addr_t* peer = &d.peer_id_addr;
    ESP_LOGW(TAG, "Repeat pairing from %s (clearing stale peer bond)", addr_to_str(*peer).c_str());
    // NimBLE requires the application to delete the old bond before returning
    // RETRY. Returning RETRY after a failed delete would just re-enter the same
    // repeat-pairing check and can strand the peer in a timeout loop.
    int delete_rc = ble_store_util_delete_peer(peer);
    if (delete_rc == 0 || delete_rc == BLE_HS_ENOENT) {
      // Remember that this connection represents an explicit re-pair. Its IRK
      // must be published even when unchanged and inside the normal rate limit.
      MutexGuard lock(self->state_mutex_);
      if (self->connected_ && self->conn_handle_ == ev->repeat_pairing.conn_handle) {
        self->connection_origin_ = CaptureOrigin::REPAIR;
      }
      return BLE_GAP_REPEAT_PAIRING_RETRY;
    }
    ESP_LOGE(TAG, "Repeat pairing: stale bond delete failed rc=%d", delete_rc);
  } else {
    ESP_LOGW(TAG, "Repeat pairing: conn desc not found rc=%d", rc);
  }
  return BLE_GAP_REPEAT_PAIRING_IGNORE;
}

//======================== GAP event handler (dispatcher)
//========================

int IRKCaptureComponent::gap_event_handler(struct ble_gap_event* ev, void* arg) {
  auto* self = static_cast<IRKCaptureComponent*>(arg);
  switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
      return handle_gap_connect(self, ev);

    case BLE_GAP_EVENT_DISCONNECT:
      return handle_gap_disconnect(self, ev);

    case BLE_GAP_EVENT_ENC_CHANGE:
      return handle_gap_enc_change(self, ev);

    case BLE_GAP_EVENT_REPEAT_PAIRING:
      return handle_gap_repeat_pairing(self, ev);

    case BLE_GAP_EVENT_CONN_UPDATE: {
      // Peers that renegotiate the connection mid-pairing are a common source
      // of supervision timeouts, so record the parameters they settled on.
      if (ev->conn_update.status != 0) {
        ESP_LOGW(TAG, "Connection update failed: handle=%u status=%d (0x%X)",
                 ev->conn_update.conn_handle, ev->conn_update.status, ev->conn_update.status);
        return 0;
      }
      struct ble_gap_conn_desc d {};
      if (ble_gap_conn_find(ev->conn_update.conn_handle, &d) == 0) {
        log_conn_params("Connection updated", d.conn_itvl, d.conn_latency, d.supervision_timeout);
      } else {
        ESP_LOGD(TAG, "Connection updated: handle=%u", ev->conn_update.conn_handle);
      }
      return 0;
    }

    case BLE_GAP_EVENT_MTU:
      ESP_LOGI(TAG, "MTU updated: %u", ev->mtu.value);
      return 0;

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
      const char* action_desc = "Unknown";
      switch (ev->passkey.params.action) {
        case BLE_SM_IOACT_NONE:
          action_desc = "None";
          break;
        case BLE_SM_IOACT_OOB:
          action_desc = "OOB";
          break;
        case BLE_SM_IOACT_INPUT:
          action_desc = "Input (peer displays passkey)";
          break;
        case BLE_SM_IOACT_DISP:
          action_desc = "Display (we should show passkey)";
          break;
        case BLE_SM_IOACT_NUMCMP:
          action_desc = "Numeric Comparison";
          break;
      }
      ESP_LOGI(TAG, "PASSKEY_ACTION: %s (action=%d)", action_desc, ev->passkey.params.action);

      // NO_INPUT_OUTPUT pairing never asks to display a passkey, and there is
      // no passkey to show (numcmp is only defined for numeric comparison).
      if (ev->passkey.params.action == BLE_SM_IOACT_DISP) {
        ESP_LOGW(TAG,
                 "UNEXPECTED: passkey display requested; not supported with Just Works "
                 "pairing");
      }

      // Just log and return - main branch behavior
      return 0;
    }

    case BLE_GAP_EVENT_NOTIFY_RX:
      return 0;

    case BLE_GAP_EVENT_NOTIFY_TX:
      return 0;

#ifdef BLE_GAP_EVENT_L2CAP_UPDATE_REQ
    case BLE_GAP_EVENT_L2CAP_UPDATE_REQ:
      // Returning 0 accepts the requested parameters. NimBLE initializes
      // self_params from peer_params before invoking this callback.
      ESP_LOGD(TAG, "L2CAP connection parameter update requested (accepted)");
      return 0;
#endif

#ifdef BLE_GAP_EVENT_SUBSCRIBE
    case BLE_GAP_EVENT_SUBSCRIBE:
      ESP_LOGD(TAG,
               "Subscription changed: handle=%u attr=%u (%s) notify=%u indicate=%u "
               "reason=%u (%s)",
               ev->subscribe.conn_handle, ev->subscribe.attr_handle,
               gatt_attr_name(ev->subscribe.attr_handle), ev->subscribe.cur_notify,
               ev->subscribe.cur_indicate, ev->subscribe.reason,
               subscribe_reason_str(ev->subscribe.reason));
      return 0;
#endif

#ifdef BLE_GAP_EVENT_IDENTITY_RESOLVED
    case BLE_GAP_EVENT_IDENTITY_RESOLVED:
      // NimBLE emits this when it stores the peer's identity address during key
      // distribution. It does not mean an IRK resolved the over-the-air address.
      ESP_LOGD(TAG, "Peer identity address received: handle=%u", ev->identity_resolved.conn_handle);
      return 0;
#endif

#ifdef BLE_GAP_EVENT_PARING_COMPLETE
    case BLE_GAP_EVENT_PARING_COMPLETE:
      // NimBLE emits this before persisting keys and before ENC_CHANGE, which
      // remains the single owner of IRK capture, failure cleanup, and
      // termination. The status is a raw SMP reason, not an overall result: it
      // can be zero even when encryption then fails with a controller error, so
      // this only reports the SMP side and ENC_CHANGE has the verdict.
      if (ev->pairing_complete.status == 0) {
        ESP_LOGD(TAG,
                 "Pairing procedure ended: handle=%u (no SMP error; result follows in "
                 "ENC_CHANGE)",
                 ev->pairing_complete.conn_handle);
      } else {
        ESP_LOGW(TAG, "Pairing failed: handle=%u SMP reason=%d (%s)",
                 ev->pairing_complete.conn_handle, ev->pairing_complete.status,
                 sm_reason_str(static_cast<uint8_t>(ev->pairing_complete.status)));
      }
      return 0;
#endif

#ifdef BLE_GAP_EVENT_DATA_LEN_CHG
    case BLE_GAP_EVENT_DATA_LEN_CHG:
      // Routine controller negotiation; useful only for deep diagnostics.
      ESP_LOGV(TAG, "Data length changed: handle=%u tx=%u/%uus rx=%u/%uus",
               ev->data_len_chg.conn_handle, ev->data_len_chg.max_tx_octets,
               ev->data_len_chg.max_tx_time, ev->data_len_chg.max_rx_octets,
               ev->data_len_chg.max_rx_time);
      return 0;
#endif

#ifdef BLE_GAP_EVENT_LINK_ESTAB
    case BLE_GAP_EVENT_LINK_ESTAB:
      // ESP-IDF emits this after CONNECT once link-layer synchronization is
      // final. A successful event is routine. A failure is only logged here; a
      // failed CONNECT whose link is still up is closed in handle_gap_connect().
      if (ev->link_estab.status == 0) {
        ESP_LOGV(TAG, "Link established: handle=%u", ev->link_estab.conn_handle);
      } else {
        ESP_LOGW(TAG, "Link establishment failed: handle=%u status=%d (0x%X)",
                 ev->link_estab.conn_handle, ev->link_estab.status, ev->link_estab.status);
      }
      return 0;
#endif

#ifdef BLE_GAP_EVENT_PHY_UPDATE_COMPLETE
    case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
      // PHY layer updated (normal)
      return 0;
#endif

#ifdef BLE_GAP_EVENT_AUTHORIZE
    case BLE_GAP_EVENT_AUTHORIZE:
      // Authorization event (allow by returning 0)
      return 0;
#endif

#ifdef BLE_GAP_EVENT_SUBRATE_CHANGE
    case BLE_GAP_EVENT_SUBRATE_CHANGE:
      // BLE 5.2+ subrate change (normal)
      return 0;
#endif

#ifdef BLE_GAP_EVENT_VS_HCI
    case BLE_GAP_EVENT_VS_HCI:
      // Vendor-specific HCI event (can ignore)
      return 0;
#endif

#ifdef BLE_GAP_EVENT_TERM_FAILURE
    case BLE_GAP_EVENT_TERM_FAILURE:
      // A disconnect the controller accepted did not complete, so the link is
      // still up. Connection state is left alone; the connection timeout keeps
      // retrying the disconnect and reboots if it never succeeds.
      ESP_LOGW(TAG, "Disconnect failed: handle=%u status=%d (0x%X)", ev->term_failure.conn_handle,
               ev->term_failure.status, ev->term_failure.status);
      return 0;
#endif

    default:
      // More verbose default logging to aid future SDK changes
      ESP_LOGD(TAG, "Unhandled GAP event type=%d", ev->type);
      return 0;
  }
}

//======================== GAP helpers ========================

void IRKCaptureComponent::on_connect(uint16_t conn_handle) {
  // Thread-safe connection and pairing state update
  bool was_advertising = false;
  bool reject_extra_connection = false;
  uint16_t existing_handle = BLE_HS_CONN_HANDLE_NONE;
  uint32_t generation = 0;
  {
    MutexGuard lock(state_mutex_);
    if (connected_ && conn_handle_ != conn_handle) {
      reject_extra_connection = true;
      existing_handle = conn_handle_;
    } else {
      reset_connection_state_();
      conn_handle_ = conn_handle;
      connected_ = true;
      connection_generation_++;
      // Reserve zero for calls that have no connection context.
      if (connection_generation_ == 0) {
        connection_generation_++;
      }
      generation = connection_generation_;
      was_advertising = advertising_;
      advertising_ = false;
      pairing_start_time_ = now_ms();
    }
  }

  if (reject_extra_connection) {
    ESP_LOGE(TAG, "Rejecting unexpected second BLE connection handle=%u (active=%u)", conn_handle,
             existing_handle);
    bool terminate_failed;
    {
      BleOpGuard ble_lock(ble_op_mutex_);
      int rc = ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
      terminate_failed = rc != 0 && rc != BLE_HS_EALREADY;
      if (terminate_failed) {
        ESP_LOGE(TAG, "Failed to terminate unexpected second connection rc=%d; rebooting", rc);
      }
    }
    if (terminate_failed) {
      // on_connect() runs on the NimBLE host task, and rebooting from here also
      // did it while ble_op_mutex_ was still held. The reboot now runs on the
      // main loop, after this handler has returned and released the guard.
      request_reboot_();
    }
    return;
  }

  // Compact summary to increase chance at least one key line survives under log
  // pressure
  // enc_ready_ was just set to false under mutex above
  ESP_LOGI(TAG, "Conn start: handle=%u enc_ready=0 was_adv=%d", conn_handle, (int) was_advertising);

  ESP_LOGI(TAG, "Connected; handle=%u, initiating security", conn_handle);
  log_conn_desc(conn_handle);
  log_sm_config();

  // Do not infer the peer's bond knowledge from sec_state.bonded here. On a
  // newly connected, unencrypted link NimBLE normally reports bonded=0 even
  // when our store contains the peer's old bond. Short-circuiting on that flag
  // used to terminate the link before a peer that forgot us could send its
  // fresh pairing request. Always enter normal security negotiation; if the
  // peer requests pairing while our old bond exists, REPEAT_PAIRING deletes
  // that stale peer bond and lets NimBLE retry cleanly.

  // Record connections that begin without a local bond. If this identity was
  // already captured earlier in the same boot (for example after a failed
  // encryption removed its bond), the eventual successful pairing must still
  // publish the IRK rather than looking like a routine bonded reconnect.
  struct ble_gap_conn_desc desc {};
  if (ble_gap_conn_find(conn_handle, &desc) == 0) {
    struct ble_store_key_sec key {};
    struct ble_store_value_sec bond {};
    key.peer_addr = desc.peer_id_addr;
    const int bond_rc = ble_store_read_peer_sec(&key, &bond);
    MutexGuard lock(state_mutex_);
    if (connected_ && conn_handle_ == conn_handle && connection_generation_ == generation &&
        connection_origin_ != CaptureOrigin::REPAIR) {
      // A failed lookup cannot establish that this is a bonded reconnect.
      connection_origin_ = bond_rc == 0               ? CaptureOrigin::BONDED
                           : bond_rc == BLE_HS_ENOENT ? CaptureOrigin::FRESH
                                                      : CaptureOrigin::UNKNOWN;
    }
  }

  // Proactively initiate pairing; peer should show pairing dialog now
  // THREAD-SAFE: use the conn_handle parameter (already stored to conn_handle_
  // under mutex at the top of on_connect)
  int rc = ble_gap_security_initiate(conn_handle);
  if (rc == 0) {
    ESP_LOGD(TAG, "Security initiate: started");
  } else if (rc == BLE_HS_EALREADY) {
    // A security procedure is already running on this link. NimBLE returns
    // this whichever side started it, so it says nothing about the peer; the
    // 2-second retry would only get EALREADY again.
    ESP_LOGD(TAG, "Security procedure already in progress; skipping retry");
    MutexGuard lock(state_mutex_);
    sec_retry_done_ = true;
  } else if (rc != 0) {
    ESP_LOGW(TAG, "ble_gap_security_initiate rc=%d", rc);
  }
}

bool IRKCaptureComponent::on_disconnect(uint16_t conn_handle) {
  {
    MutexGuard lock(state_mutex_);
    if (!connected_ || conn_handle_ != conn_handle) return false;
    finish_disconnect_(now_ms());
  }
  ESP_LOGD(TAG, "Connection closed; state reset and pending operations advanced");
  ESP_LOGI(TAG, "Disconnected");
  return true;
}

}  // namespace irk_capture
}  // namespace esphome

#endif  // USE_ESP32
