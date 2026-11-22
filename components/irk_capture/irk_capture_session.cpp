// One connection from encryption to outcome: reading the IRK from the bond
// store, fallback timers, the security retry, timeouts, polling, and the
// no-IRK and failure results.

#include "irk_capture_internal.h"

// ESP32-only implementation - requires Bluetooth hardware
#ifdef USE_ESP32

#include <mbedtls/aes.h>

namespace esphome {
namespace irk_capture {

//======================== IRK address check ========================

bool aes128_encrypt_be(const uint8_t key[16], const uint8_t plaintext[16], uint8_t out[16]) {
  // Runs on the NimBLE task for one block per capture; ESP-IDF's mbedTLS uses
  // the AES hardware where the chip has it.
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  int rc = mbedtls_aes_setkey_enc(&ctx, key, 128);
  if (rc == 0) rc = mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, plaintext, out);
  mbedtls_aes_free(&ctx);
  if (rc != 0) {
    ESP_LOGW(TAG, "AES for the IRK address check failed rc=%d; IRK left unverified", rc);
    return false;
  }
  return true;
}

//======================== IRK Validation ========================

/**
 * @brief Validates that an IRK is not all-zero or all-FF
 * (invalid/uninitialized)
 * @param irk 16-byte IRK array
 * @return true if IRK is valid, false if invalid
 */
bool IRKCaptureComponent::is_valid_irk(const uint8_t irk[16]) {
  // All-zero is invalid and all-FF is erased flash; both are one repeated byte.
  const bool uniform = std::all_of(irk + 1, irk + 16, [irk](uint8_t b) { return b == irk[0]; });
  if (uniform && (irk[0] == 0x00 || irk[0] == 0xFF)) {
    ESP_LOGW(TAG, "Rejected %s IRK",
             irk[0] == 0x00 ? "all-zero (invalid)" : "all-FF (uninitialized)");
    return false;
  }
  return true;
}

BondRead IRKCaptureComponent::read_peer_irk_(const ble_addr_t& peer_id,
                                             struct ble_store_value_sec& bond, int* rc_out) {
  // The one place a stored bond is read and classified. Callers differ only in
  // what each outcome means for them. An IRK that fails validation is logged
  // by is_valid_irk().
  struct ble_store_key_sec key {};
  key.peer_addr = peer_id;
  const int rc = ble_store_read_peer_sec(&key, &bond);
  if (rc_out != nullptr) *rc_out = rc;
  if (rc == BLE_HS_ENOENT) return BondRead::NO_BOND;
  if (rc != 0) return BondRead::READ_ERROR;
  if (!bond.irk_present) return BondRead::NO_IRK;
  return is_valid_irk(bond.irk) ? BondRead::IRK : BondRead::INVALID_IRK;
}

//======================== IRK extraction ========================

bool IRKCaptureComponent::try_get_irk(uint16_t conn_handle, uint8_t irk_out[16],
                                      ble_addr_t& peer_id_out, uint32_t connection_generation,
                                      CaptureOrigin origin, ble_addr_t* connection_addr_out) {
  struct ble_store_value_sec bond {};
  struct ble_gap_conn_desc desc {};

  int rc = ble_gap_conn_find(conn_handle, &desc);
  if (rc != 0) {
    ESP_LOGD(TAG, "ble_gap_conn_find failed rc=%d (handle may be invalid)", rc);
    return false;
  }

  ESP_LOGD(TAG, "Reading bond for peer: %s", addr_to_str(desc.peer_id_addr).c_str());

  // Keyed by the peer identity address, which is stable.
  const BondRead read = read_peer_irk_(desc.peer_id_addr, bond, &rc);
  if (read == BondRead::NO_BOND) {
    ESP_LOGD(TAG, "No bond record for peer (ENOENT)");
    return false;
  }
  if (read == BondRead::READ_ERROR) {
    // Polled once a second: say it once per connection, not 45 times.
    bool first;
    {
      MutexGuard lock(state_mutex_);
      first = !irk_read_error_logged_;
      irk_read_error_logged_ = true;
    }
    if (first) ESP_LOGW(TAG, "ble_store_read_peer_sec failed rc=%d; will keep trying", rc);
    return false;
  }

  // Detailed bond information for debugging
  ESP_LOGD(TAG,
           "Bond found: ediv=%u rand=%llu irk_present=%d ltk_present=%d "
           "csrk_present=%d",
           (unsigned) bond.ediv, (unsigned long long) bond.rand_num, (int) bond.irk_present,
           (int) bond.ltk_present, (int) bond.csrk_present);

  ESP_LOGD(TAG, "Bond security: authenticated=%d sc=%d", (int) bond.authenticated, (int) bond.sc);

  if (read == BondRead::NO_IRK) {
    // This read is called only by polling after successful encryption.
    publish_no_irk_(desc.peer_id_addr, connection_generation, true, origin, &desc.peer_ota_addr);
    close_after_no_irk_(conn_handle, connection_generation, "after poll no-IRK result");
    return false;
  }

  if (read == BondRead::INVALID_IRK) {
    ESP_LOGW(TAG, "IRK failed validation (all-zero or all-FF)");
    return false;
  }

  // Return raw IRK bytes (16 bytes guaranteed by NimBLE) and peer identity
  // address
  std::memcpy(irk_out, bond.irk, 16);
  peer_id_out = desc.peer_id_addr;
  if (connection_addr_out != nullptr) *connection_addr_out = desc.peer_ota_addr;
  return true;
}

//======================== Timer helpers ========================

bool IRKCaptureComponent::enqueue_peer_timer_(std::array<PeerTimer, PEER_TIMER_CAPACITY>& timers,
                                              const ble_addr_t& peer_id,
                                              uint32_t connection_generation, uint32_t delay_ms,
                                              bool pairing_completed, CaptureOrigin origin,
                                              const ble_addr_t* connection_addr) {
  const uint32_t due_ms = deadline_after(now_ms(), delay_ms);  // Zero marks an unused slot.
  MutexGuard lock(state_mutex_);
  PeerTimer* available = nullptr;
  PeerTimer* oldest = nullptr;
  for (auto& timer : timers) {
    if (timer.due_ms == 0) {
      if (!available) available = &timer;
      continue;
    }
    if (timer.connection_generation == connection_generation &&
        addr_equal(timer.peer_id, peer_id)) {
      timer.due_ms = due_ms;
      timer.pairing_completed |= pairing_completed;
      // Evidence may strengthen within one connection (for example a repeat
      // pairing following an initially bonded connection), never downgrade it.
      if (origin == CaptureOrigin::REPAIR || timer.origin == CaptureOrigin::UNKNOWN ||
          (origin == CaptureOrigin::FRESH && timer.origin == CaptureOrigin::BONDED)) {
        timer.origin = origin;
      }
      // One connection has one address: keep it once known.
      if (connection_addr != nullptr && !timer.has_connection_addr) {
        timer.has_connection_addr = true;
        timer.connection_addr = *connection_addr;
      }
      return false;
    }
    if (!oldest || static_cast<int32_t>(timer.due_ms - oldest->due_ms) < 0) {
      oldest = &timer;
    }
  }
  // Losing the oldest fallback read is preferable to rebooting and clearing
  // every bond and capture in the current session. Prefer a free slot first.
  PeerTimer* target = available ? available : oldest;
  *target = { due_ms,
              peer_id,
              connection_generation,
              pairing_completed,
              origin,
              connection_addr != nullptr,
              connection_addr != nullptr ? *connection_addr : ble_addr_t {} };
  return available == nullptr;
}

void IRKCaptureComponent::schedule_post_disconnect_check(const ble_addr_t& peer_id,
                                                         uint32_t connection_generation,
                                                         bool pairing_completed,
                                                         CaptureOrigin origin,
                                                         const ble_addr_t* connection_addr) {
  if (enqueue_peer_timer_(post_disc_timers_, peer_id, connection_generation,
                          TimingConfig::POST_DISC_DELAY_MS, pairing_completed, origin,
                          connection_addr)) {
    ESP_LOGW(TAG, "Post-disconnect timer queue full; replaced oldest fallback check");
  }
}

void IRKCaptureComponent::schedule_late_enc_check(const ble_addr_t& peer_id,
                                                  uint32_t connection_generation,
                                                  CaptureOrigin origin,
                                                  const ble_addr_t* connection_addr) {
  if (enqueue_peer_timer_(late_enc_timers_, peer_id, connection_generation,
                          TimingConfig::ENC_LATE_READ_DELAY_MS, true, origin, connection_addr)) {
    ESP_LOGW(TAG, "Late-ENC timer queue full; replaced oldest fallback check");
  }
}

void IRKCaptureComponent::cancel_peer_timers_(const ble_addr_t& peer_id,
                                              uint32_t connection_generation) {
  // Caller holds state_mutex_.
  for (auto* timers : { &post_disc_timers_, &late_enc_timers_ }) {
    for (auto& timer : *timers) {
      if (timer.connection_generation == connection_generation &&
          addr_equal(timer.peer_id, peer_id))
        timer = {};
    }
  }
}

void IRKCaptureComponent::run_peer_timer_(std::array<PeerTimer, PEER_TIMER_CAPACITY>& timers,
                                          uint32_t now, const char* context_tag) {
  // Consume one due entry per loop; any others run on later ticks.
  PeerTimer due;
  {
    MutexGuard lock(state_mutex_);
    auto it = std::find_if(timers.begin(), timers.end(), [now](const PeerTimer& timer) {
      return timer.due_ms != 0 && deadline_reached(now, timer.due_ms);
    });
    if (it == timers.end()) return;
    due = *it;
    *it = {};
  }
  if (addr_is_zero(due.peer_id)) {
    ESP_LOGD(TAG, "%s timer fired without a peer address (skipping)", context_tag);
    return;
  }

  const ble_addr_t* connection_addr = due.has_connection_addr ? &due.connection_addr : nullptr;
  struct ble_store_value_sec bond {};
  int rc = 0;
  const BondRead read = read_peer_irk_(due.peer_id, bond, &rc);
  if (read == BondRead::NO_BOND) {
    ESP_LOGD(TAG, "No bond for peer (ENOENT) - %s check", context_tag);
  } else if (read == BondRead::READ_ERROR) {
    ESP_LOGW(TAG, "ble_store_read_peer_sec rc=%d - %s check", rc, context_tag);
  } else if (read == BondRead::NO_IRK) {
    publish_no_irk_(due.peer_id, due.connection_generation, due.pairing_completed, due.origin,
                    connection_addr);
  } else if (read == BondRead::INVALID_IRK) {
    log_no_irk_for_peer(due.peer_id);
  } else {
    publish_and_log_irk(this, due.peer_id, to_hex_rev(bond.irk, sizeof(bond.irk)), context_tag,
                        due.connection_generation, due.origin,
                        check_irk_against_address(bond.irk, connection_addr));

    // Terminate only if the connection this check was scheduled for is still
    // up. A post-disconnect check never is; a late encryption check may be,
    // unless that peer left and another device connected during the delay.
    uint16_t handle = BLE_HS_CONN_HANDLE_NONE;
    {
      MutexGuard lock(state_mutex_);
      if (connected_ && connection_generation_ == due.connection_generation) handle = conn_handle_;
    }
    struct ble_gap_conn_desc cur {};
    if (handle != BLE_HS_CONN_HANDLE_NONE && ble_gap_conn_find(handle, &cur) == 0 &&
        addr_equal(cur.peer_id_addr, due.peer_id)) {
      terminate_bounded_(handle, due.connection_generation, "after late IRK capture");
    }
  }
}

//======================== Loop helpers ========================

void IRKCaptureComponent::retry_security_if_needed(uint32_t now) {
  // Snapshot the connection and its retry state atomically. Separate snapshots
  // could combine an old connection handle with a newer connection's timers.
  bool is_connected;
  uint16_t conn_handle_copy;
  bool enc_ready_copy;
  bool sec_retry_done_copy;
  uint32_t generation_copy;
  uint32_t sec_init_time_copy;
  {
    MutexGuard lock(state_mutex_);
    is_connected = connected_;
    conn_handle_copy = conn_handle_;
    enc_ready_copy = enc_ready_;
    sec_retry_done_copy = sec_retry_done_;
    generation_copy = connection_generation_;
    sec_init_time_copy = sec_init_time_ms_;
    if (!is_connected || connection_timeout_.reason != TimeoutReason::NONE) return;
  }

  // Tested working behavior: one retry after SEC_RETRY_DELAY_MS from the
  // initial security request.
  if (is_connected && !enc_ready_copy) {
    if (sec_init_time_copy == 0) {
      MutexGuard lock(state_mutex_);
      if (!connected_ || conn_handle_ != conn_handle_copy ||
          connection_generation_ != generation_copy || enc_ready_) {
        return;
      }
      if (sec_init_time_ms_ == 0) {
        sec_init_time_ms_ = now;
      }
      // Update the local copy too so the retry check uses the new baseline.
      sec_init_time_copy = sec_init_time_ms_;
    }

    // Retry security after configured delay
    if (!sec_retry_done_copy && (now - sec_init_time_copy) > TimingConfig::SEC_RETRY_DELAY_MS) {
      uint32_t elapsed = now - sec_init_time_copy;
      // Set sec_retry_done_ under mutex BEFORE calling
      // ble_gap_security_initiate(). Writing it after the BLE call is a race: a
      // NimBLE callback could fire during the call and read sec_retry_done_ as
      // false, causing a double retry.
      bool retry_current_connection = false;
      {
        MutexGuard lock(state_mutex_);
        if (connected_ && conn_handle_ == conn_handle_copy &&
            connection_generation_ == generation_copy && !enc_ready_) {
          sec_retry_done_ = true;
          retry_current_connection = true;
        }
      }
      if (!retry_current_connection) return;
      int rc = ble_gap_security_initiate(conn_handle_copy);
      // rc==0 (started) and EALREADY (a security procedure is already running,
      // started by either side) are benign; only a genuine error is a warning.
      // Logged after the call: only rc says whether a retry really happened.
      // EALREADY means pairing is still running, which is normal for slow peers.
      if (rc == 0) {
        ESP_LOGI(TAG, "Security not started after %" PRIu32 " ms; retried security initiate",
                 elapsed);
      } else if (rc == BLE_HS_EALREADY) {
        ESP_LOGD(TAG, "Security retry skipped after %" PRIu32 " ms: pairing already in progress",
                 elapsed);
      } else {
        ESP_LOGW(TAG, "Security retry after %" PRIu32 " ms failed rc=%d", elapsed, rc);
      }
    }
  }
}

void IRKCaptureComponent::reset_connection_state_() {
  // Caller holds state_mutex_. Keep the generation for delayed capture results;
  // on_connect() advances it before the next peer can become active.
  pending_terminate_handle_ = BLE_HS_CONN_HANDLE_NONE;
  pending_terminate_busy_logged_ = false;
  connected_ = false;
  conn_handle_ = BLE_HS_CONN_HANDLE_NONE;
  connection_origin_ = CaptureOrigin::UNKNOWN;
  pairing_start_time_ = 0;
  enc_ready_ = false;
  enc_time_ = 0;
  sec_retry_done_ = false;
  sec_init_time_ms_ = 0;
  irk_gave_up_ = false;
  irk_read_error_logged_ = false;
  irk_last_try_ms_ = 0;
  connection_timeout_ = {};
}

void IRKCaptureComponent::finish_disconnect_(uint32_t now) {
  // Caller holds state_mutex_. A vanished descriptor and a GAP disconnect must
  // perform the same cleanup, including a pending MAC rotation's handoff.
  if (connection_timeout_.reason != TimeoutReason::NONE) {
    if (advertising_requested_) {
      adv_restart_time_ = deadline_after(now, TimingConfig::TIMEOUT_COOLDOWN_MS);
    }
  }
  reset_connection_state_();
  if (mac_rotation_state_ == MacRotationState::REQUESTED) {
    mac_rotation_state_ = MacRotationState::READY_TO_ROTATE;
  }
}

// Once either deadline latches a reason, recovery owns the link until it closes.
// Encryption completing afterwards does NOT cancel it: the termination may
// already be in flight, and a half-cancelled teardown is harder to reason about
// than always finishing it. This is safe for capture because a successful IRK is
// never lost to the teardown - publish_irk_to_sensors() records the generation in
// last_result_generation_, publish_no_irk_() refuses to overwrite a result already
// recorded for that generation, and the disconnect path skips its fallback read
// once the IRK has been observed. A late capture therefore survives, and at worst
// key distribution that is still in progress is cut short and retried on the next
// pairing attempt.
bool IRKCaptureComponent::handle_connection_timeout_(uint32_t now) {
  uint16_t handle;
  uint32_t generation;
  TimeoutReason reason;
  {
    MutexGuard lock(state_mutex_);
    if (!connected_) return false;
    if (connection_timeout_.reason == TimeoutReason::NONE) {
      if (!enc_ready_ && sec_init_time_ms_ != 0 &&
          elapsed_ms(now, sec_init_time_ms_) > TimingConfig::SEC_TIMEOUT_MS) {
        connection_timeout_.reason = TimeoutReason::ENCRYPTION;
      } else if (pairing_start_time_ != 0 &&
                 elapsed_ms(now, pairing_start_time_) > TimingConfig::PAIRING_TOTAL_TIMEOUT_MS) {
        connection_timeout_.reason = TimeoutReason::PAIRING;
      } else {
        return false;
      }
    }
    if (connection_timeout_.retry_ms != 0 && !deadline_reached(now, connection_timeout_.retry_ms))
      return true;
    handle = conn_handle_;
    generation = connection_generation_;
    reason = connection_timeout_.reason;
  }

  report_capture_failure_(generation);

  // Only call while holding state_mutex_. Every unlocked stack operation can
  // dispatch a reset/disconnect callback and even allow the handle to be reused.
  auto is_current = [&]() {
    return connected_ && conn_handle_ == handle && connection_generation_ == generation &&
           connection_timeout_.reason == reason;
  };
  const char* label = reason == TimeoutReason::ENCRYPTION ? "Encryption" : "Pairing";
  struct ble_gap_conn_desc desc {};
  const int desc_rc = ble_gap_conn_find(handle, &desc);
  uint8_t attempts;
  bool clear_bond;
  {
    MutexGuard lock(state_mutex_);
    if (!is_current()) return true;
    if (desc_rc != 0) {
      // No disconnect callback will arrive. Use the normal disconnect cleanup
      // so timeout recovery cannot strand MAC rotation in REQUESTED.
      finish_disconnect_(now);
      return true;
    }
    attempts = connection_timeout_.attempts;
    clear_bond = !connection_timeout_.bond_cleared;
  }
  if (attempts >= TimingConfig::TERMINATE_MAX_ATTEMPTS) {
    ESP_LOGE(TAG, "%s-timeout connection remains after %u terminate attempts; rebooting", label,
             attempts);
    App.safe_reboot();
    return true;
  }

  if (clear_bond) {
    ESP_LOGW(TAG, "%s timeout; clearing bond for %s", label,
             addr_to_str(desc.peer_id_addr).c_str());
    // Store work takes NimBLE's host lock; run it outside both component locks.
    const int delete_rc = ble_store_util_delete_peer(&desc.peer_id_addr);
    {
      MutexGuard lock(state_mutex_);
      if (!is_current()) return true;
      connection_timeout_.bond_cleared = delete_rc == 0 || delete_rc == BLE_HS_ENOENT;
    }
    if (delete_rc != 0 && delete_rc != BLE_HS_ENOENT) {
      ESP_LOGW(TAG, "%s-timeout bond delete rc=%d; will retry", label, delete_rc);
    }
  }

  int rc = BLE_HS_EBUSY;
  bool acquired;
  {
    BleOpGuard ble_lock(ble_op_mutex_, pdMS_TO_TICKS(TimingConfig::BLE_OP_LOCK_WAIT_MS));
    acquired = ble_lock.acquired();
    {
      MutexGuard lock(state_mutex_);
      if (!is_current()) return true;
      attempts = ++connection_timeout_.attempts;
      // Arm the backoff before the stack call. A synchronous callback must not
      // see an unclaimed attempt, and a disconnect may reset this state entirely.
      connection_timeout_.retry_ms = deadline_after(now, TimingConfig::TERMINATE_RETRY_MS);
    }
    if (acquired) rc = ble_gap_terminate(handle, BLE_ERR_REM_USER_CONN_TERM);
  }

  const bool accepted = rc == 0 || rc == BLE_HS_EALREADY;
  {
    MutexGuard lock(state_mutex_);
    if (!is_current()) return true;
    if (accepted) {
      connection_timeout_.retry_ms = deadline_after(now, TimingConfig::TERMINATE_PENDING_GRACE_MS);
    }
  }
  if (accepted) {
    ESP_LOGI(TAG, "%s-timeout termination initiated (attempt %u); awaiting disconnect", label,
             attempts);
  } else {
    ESP_LOGW(TAG, "%s-timeout terminate %s rc=%d (attempt %u)", label,
             acquired ? "failed" : "mutex timeout", rc, attempts);
    if (attempts >= TimingConfig::TERMINATE_MAX_ATTEMPTS) App.safe_reboot();
  }
  return true;
}

void IRKCaptureComponent::poll_irk_if_due(uint32_t now) {
  // THREAD-SAFE: Snapshot polling state under mutex for main-loop reads
  bool enc_ready_copy;
  bool irk_gave_up_copy;
  uint32_t enc_time_copy;
  uint32_t irk_last_try_copy;
  uint16_t conn_handle_copy;
  uint32_t connection_generation_copy;
  CaptureOrigin origin;
  {
    MutexGuard lock(state_mutex_);
    enc_ready_copy = enc_ready_;
    irk_gave_up_copy = irk_gave_up_;
    enc_time_copy = enc_time_;
    irk_last_try_copy = irk_last_try_ms_;
    conn_handle_copy = conn_handle_;
    connection_generation_copy = connection_generation_;
    origin = connection_origin_;
  }

  // Attempt IRK retrieval after encryption + delay (allow store write)
  if (!enc_ready_copy || irk_gave_up_copy) return;
  if (elapsed_ms(now, enc_time_copy) < TimingConfig::ENC_TO_FIRST_TRY_DELAY_MS) return;
  if (elapsed_ms(now, irk_last_try_copy) < TimingConfig::ENC_TRY_INTERVAL_MS) return;

  {
    MutexGuard lock(state_mutex_);
    irk_last_try_ms_ = now;
  }

  uint8_t irk_bytes[16];
  ble_addr_t peer_id;
  ble_addr_t connection_addr {};
  if (try_get_irk(conn_handle_copy, irk_bytes, peer_id, connection_generation_copy, origin,
                  &connection_addr)) {
    std::string irk_hex = to_hex_rev(irk_bytes, sizeof(irk_bytes));
    publish_and_log_irk(this, peer_id, irk_hex, "POLL_CONNECTED", connection_generation_copy,
                        origin, check_irk_against_address(irk_bytes, &connection_addr));
    // The store read and publish above ran without the lock. If the peer
    // disconnected meanwhile, NimBLE can hand the same handle to a replacement
    // connection, so terminating it now would drop an unrelated device and mark
    // that new connection as finished polling. Re-validate the generation and
    // claim the completion flag atomically before touching the link.
    bool still_current;
    {
      MutexGuard lock(state_mutex_);
      still_current = connected_ && conn_handle_ == conn_handle_copy &&
                      connection_generation_ == connection_generation_copy;
      if (still_current) irk_gave_up_ = true;
    }
    if (!still_current) {
      ESP_LOGD(TAG, "Poll capture completed for a connection that already closed; not terminating");
      return;
    }
    terminate_bounded_(conn_handle_copy, connection_generation_copy, "after poll IRK capture");
  } else {
    if (elapsed_ms(now, enc_time_copy) > TimingConfig::ENC_GIVE_UP_AFTER_MS) {
      ESP_LOGW(TAG, "IRK not found after %" PRIu32 " ms post-encryption",
               TimingConfig::ENC_GIVE_UP_AFTER_MS);
      bool gave_up = false;
      {
        MutexGuard lock(state_mutex_);
        // Only give up on the connection this poll was actually running against.
        if (connected_ && connection_generation_ == connection_generation_copy) {
          irk_gave_up_ = true;
          gave_up = true;
        }
      }
      if (gave_up) report_capture_failure_(connection_generation_copy);
    }
  }
}

//======================== Public publish utility ========================

void IRKCaptureComponent::publish_no_irk_(const ble_addr_t& peer_id, uint32_t connection_generation,
                                          bool pairing_completed, CaptureOrigin origin,
                                          const ble_addr_t* connection_addr) {
  if (!pairing_completed || connection_generation == 0) return;
  const std::string addr = addr_to_str(peer_id);
  {
    MutexGuard lock(state_mutex_);
    // A completed no-IRK observation finishes this connection's capture work,
    // including a bonded reconnect that must not replace the displayed result.
    if (connection_generation != connection_generation_) return;
    if (connected_) irk_gave_up_ = true;
    cancel_peer_timers_(peer_id, connection_generation);
    // Never replace this connection's successful result or repeat a failure.
    // Re-encrypting an existing bond is not a new pairing attempt. Preserve the
    // user's last result, while still allowing the first observation to fill
    // an empty display. UNKNOWN is intentionally not treated as BONDED: a failed
    // initial store read does not prove that this was a background reconnect.
    if (last_result_generation_ == connection_generation ||
        (origin == CaptureOrigin::BONDED && last_result_generation_ != 0))
      return;
    last_result_generation_ = connection_generation;
    pending_irk_hex_ = "Failed: IRK not used";
    pending_irk_addr_ = addr;
    pending_irk_check_ = IrkCheck::UNCHECKED;  // No key to check
    pending_irk_pub_ = true;
    capture_status_ = CaptureStatus::NO_IRK;
    status_result_hold_until_ = deadline_after(now_ms(), TimingConfig::STATUS_RESULT_HOLD_MS);
  }
  ESP_LOGW(TAG, "IRK capture failed for %s: completed pairing did not provide an IRK",
           addr.c_str());
  // The address the peer connected from says whether its MAC can stand in for
  // the missing key. Without it, offer the MAC only as a conditional next step.
  const AddressKind kind =
      connection_addr != nullptr ? classify_address(*connection_addr) : AddressKind::UNKNOWN;
  switch (kind) {
    case AddressKind::PUBLIC:
    case AddressKind::STATIC_RANDOM:
      ESP_LOGW(TAG,
               "The device connected from a fixed %s address, so its MAC should not change. "
               "Use this MAC directly: %s",
               kind == AddressKind::PUBLIC ? "public" : "static random", addr.c_str());
      break;
    case AddressKind::RESOLVABLE_PRIVATE:
    case AddressKind::NON_RESOLVABLE_PRIVATE:
      ESP_LOGW(TAG,
               "The device connected from a private address (%s) that changes over time, so its "
               "MAC cannot be used instead of an IRK",
               addr_to_str(*connection_addr).c_str());
      break;
    case AddressKind::UNKNOWN:
      ESP_LOGW(TAG,
               "No IRK was provided. If this device uses a fixed BLE address, use this MAC "
               "directly: %s",
               addr.c_str());
      break;
  }
}

void IRKCaptureComponent::close_after_no_irk_(uint16_t conn_handle, uint32_t connection_generation,
                                              const char* context) {
  // A completed pairing without an IRK is as final as a capture. Leaving the
  // link up would hold the only connection slot until the 90 s pairing timeout
  // and then report a timeout that never happened. The bond is still stored,
  // so the peer may reconnect at once and reach the same outcome: hold
  // advertising off briefly so that cannot become a tight reconnect loop.
  {
    MutexGuard lock(state_mutex_);
    // The store read ran unlocked. Never close a replacement connection that
    // reused this handle, or a link this component is not tracking.
    if (!connected_ || conn_handle_ != conn_handle || connection_generation == 0 ||
        connection_generation_ != connection_generation)
      return;
    if (advertising_requested_) {
      adv_restart_time_ = deadline_after(now_ms(), TimingConfig::ADV_SUPPRESS_RESTART_DELAY_MS);
    }
  }
  terminate_bounded_(conn_handle, connection_generation, context);
}

void IRKCaptureComponent::terminate_bounded_(uint16_t conn_handle, uint32_t generation,
                                             const char* context) {
  // Never wait on the host queue or terminate a bare handle from the main task.
  // A disconnect/reconnect can reuse it while an operation waits for a lock.
  {
    MutexGuard lock(state_mutex_);
    if (!connected_ || conn_handle_ != conn_handle || connection_generation_ != generation) return;
    if (pending_terminate_handle_ != conn_handle || pending_terminate_generation_ != generation)
      pending_terminate_busy_logged_ = false;
    pending_terminate_handle_ = conn_handle;
    pending_terminate_generation_ = generation;
    pending_terminate_context_ = context;
  }
  notify_bond_clear_();
}

void IRKCaptureComponent::handle_pending_termination_() {
  // Only the NimBLE task runs this callback, serially with connect/disconnect.
  // A busy control lock leaves the request pending for the next worker wake-up.
  {
    MutexGuard lock(state_mutex_);
    if (pending_terminate_handle_ == BLE_HS_CONN_HANDLE_NONE) return;
  }
  int rc;
  const char* context;
  {
    BleOpGuard ble_lock(ble_op_mutex_, 0);
    if (!ble_lock.acquired()) {
      // loop() wakes the worker again. Log once per request, not every 50 ms.
      const char* deferred;
      {
        MutexGuard lock(state_mutex_);
        if (pending_terminate_handle_ == BLE_HS_CONN_HANDLE_NONE || pending_terminate_busy_logged_)
          return;
        pending_terminate_busy_logged_ = true;
        deferred = pending_terminate_context_;  // String literal; safe after unlock
      }
      ESP_LOGW(
          TAG,
          "Disconnect %s deferred: BLE operation lock busy; will retry on the next worker wake-up",
          deferred ? deferred : "");
      return;
    }
    uint16_t handle;
    {
      MutexGuard lock(state_mutex_);
      handle = pending_terminate_handle_;
      context = pending_terminate_context_;
      const bool current = connected_ && handle != BLE_HS_CONN_HANDLE_NONE &&
                           conn_handle_ == handle &&
                           connection_generation_ == pending_terminate_generation_;
      pending_terminate_handle_ = BLE_HS_CONN_HANDLE_NONE;
      if (!current) return;
    }
    rc = ble_gap_terminate(handle, BLE_ERR_REM_USER_CONN_TERM);
  }
  log_terminate_result(rc, context);
}

void IRKCaptureComponent::report_capture_failure_(uint32_t connection_generation) {
  // A pairing that ended without a result used to be visible only in the log:
  // Status went straight back to "advertising" and the wizard kept waiting.
  MutexGuard lock(state_mutex_);
  // Ignore links this component is not tracking, and never replace a result
  // this connection has already produced.
  if (connection_generation == 0 || connection_generation != connection_generation_ ||
      last_result_generation_ == connection_generation ||
      (capture_status_ == CaptureStatus::FAILED && failed_generation_ == connection_generation))
    return;
  failed_generation_ = connection_generation;
  capture_status_ = CaptureStatus::FAILED;
  status_result_hold_until_ = deadline_after(now_ms(), TimingConfig::STATUS_RESULT_HOLD_MS);
}

}  // namespace irk_capture
}  // namespace esphome

#endif  // USE_ESP32
