// One connection from encryption to outcome: reading the IRK from the bond
// store, fallback timers, the security retry, timeouts, polling, and the
// no-IRK and failure results.

#include "irk_capture_internal.h"

// ESP32-only implementation - requires Bluetooth hardware
#ifdef USE_ESP32

// ESP-IDF 6 ships mbedTLS 4, which dropped the legacy mbedtls/aes.h; use PSA there.
#if __has_include(<mbedtls/aes.h>)
#include <mbedtls/aes.h>
#else
#define IRK_CAPTURE_USE_PSA_AES
#include <psa/crypto.h>
#endif

namespace esphome {
namespace irk_capture {

//======================== IRK address check ========================

bool aes128_encrypt_be(const uint8_t key[16], const uint8_t plaintext[16], uint8_t out[16]) {
  // One AES-128 block per RPA check, on the NimBLE or main task. ESP-IDF
  // mbedTLS can use hardware AES when enabled.
#ifdef IRK_CAPTURE_USE_PSA_AES
  // psa_crypto_init() is idempotent; the stack may or may not have run it already.
  int rc = psa_crypto_init();
  psa_key_id_t key_id = 0;
  if (rc == PSA_SUCCESS) {
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attr, 128);
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_ENCRYPT);
    psa_set_key_algorithm(&attr, PSA_ALG_ECB_NO_PADDING);
    rc = psa_import_key(&attr, key, 16, &key_id);
    psa_reset_key_attributes(&attr);
  }
  if (rc == PSA_SUCCESS) {
    size_t out_len = 0;
    rc = psa_cipher_encrypt(key_id, PSA_ALG_ECB_NO_PADDING, plaintext, 16, out, 16, &out_len);
    if (rc == PSA_SUCCESS && out_len != 16) rc = PSA_ERROR_GENERIC_ERROR;
    psa_destroy_key(key_id);
  }
#else
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  int rc = mbedtls_aes_setkey_enc(&ctx, key, 128);
  if (rc == 0) rc = mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, plaintext, out);
  mbedtls_aes_free(&ctx);
#endif
  if (rc != 0) {
    ESP_LOGW(TAG, "AES for the IRK address check failed rc=%d; IRK left unverified", rc);
    return false;
  }
  return true;
}

//======================== IRK Validation ========================

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
  {
    // Caller released state_mutex_ before this call; NimBLE reuses handles, so
    // reject a stale descriptor before reading or publishing a peer's bond.
    MutexGuard lock(state_mutex_);
    if (!connected_ || conn_handle_ != conn_handle ||
        connection_generation_ != connection_generation)
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
  // Evict the oldest fallback check when full rather than failing or allocating.
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
  // Close the link only if this timer's original connection is still active
  // (possible for late-ENC, never for post-disconnect).
  auto live_handle = [&]() {
    uint16_t handle = BLE_HS_CONN_HANDLE_NONE;
    {
      MutexGuard lock(state_mutex_);
      if (connected_ && connection_generation_ == due.connection_generation) handle = conn_handle_;
    }
    struct ble_gap_conn_desc cur {};
    if (handle != BLE_HS_CONN_HANDLE_NONE &&
        (ble_gap_conn_find(handle, &cur) != 0 || !addr_equal(cur.peer_id_addr, due.peer_id)))
      handle = BLE_HS_CONN_HANDLE_NONE;
    return handle;
  };
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
    // publish_no_irk_() ends polling for this connection, so nothing else
    // would close it before the pairing timeout.
    const uint16_t handle = live_handle();
    if (handle != BLE_HS_CONN_HANDLE_NONE) {
      close_after_no_irk_(handle, due.connection_generation, "after late no-IRK result");
    }
  } else if (read == BondRead::INVALID_IRK) {
    log_no_irk_for_peer(due.peer_id);
  } else {
    publish_and_log_irk(this, due.peer_id, to_hex_rev(bond.irk, sizeof(bond.irk)), context_tag,
                        due.connection_generation, due.origin,
                        check_irk_against_address(bond.irk, connection_addr));
    const uint16_t handle = live_handle();
    if (handle != BLE_HS_CONN_HANDLE_NONE) {
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

  // One retry after SEC_RETRY_DELAY_MS if encryption has not completed.
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
      sec_init_time_copy = sec_init_time_ms_;
    }

    if (!sec_retry_done_copy && (now - sec_init_time_copy) > TimingConfig::SEC_RETRY_DELAY_MS) {
      uint32_t elapsed = now - sec_init_time_copy;
      // Claim sec_retry_done_ under the generation check before the BLE call so
      // a reconnect during ble_gap_security_initiate() cannot have its flag clobbered.
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
      // rc==0 and EALREADY (pairing already running, normal for slow peers) are
      // benign; only genuine errors warn.
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
  pending_terminate_for_rotation_ = false;
  pending_terminate_retries_ = 0;
  pending_terminate_retry_time_ = 0;
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

// Once either deadline latches a reason, recovery owns the link until close;
// late encryption does not cancel an in-flight teardown. Any IRK captured
// before disconnect still survives (last_result_generation_ protects it from
// overwrites, and disconnect skips fallback reads once observed).
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
    // Encryption never completed, so the bond this deletes is stale, as after
    // an ENC_CHANGE failure. If the delete fails and the peer then drops, the
    // disconnect must not read and publish it.
    if (reason == TimeoutReason::ENCRYPTION) enc_failed_generation_ = generation;
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
    // Store read/publish ran unlocked; NimBLE may have reused conn_handle_ for
    // a new peer. Re-check generation before setting irk_gave_up_ or terminating.
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

//======================== Capture outcomes & teardown ========================

void IRKCaptureComponent::publish_no_irk_(const ble_addr_t& peer_id, uint32_t connection_generation,
                                          bool pairing_completed, CaptureOrigin origin,
                                          const ble_addr_t* connection_addr) {
  if (!pairing_completed || connection_generation == 0) return;
  const std::string addr = addr_to_str(peer_id);
  {
    MutexGuard lock(state_mutex_);
    // A completed no-IRK observation finishes this connection's capture work,
    // including a bonded reconnect that must not replace the displayed result.
    if (connection_generation != connection_generation_ ||
        capture_cancelled_locked_(connection_generation))
      return;
    if (connected_) irk_gave_up_ = true;
    cancel_peer_timers_(peer_id, connection_generation);
    // Never overwrite this connection's result or let a bonded reconnect
    // replace an existing display (UNKNOWN still writes since bond state is unproved).
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
  // No-IRK is terminal; leaving the link up would trigger a bogus 90s pairing
  // timeout. Because the bond stays stored, hold adv off briefly to avoid a loop.
  {
    MutexGuard lock(state_mutex_);
    // The store read ran unlocked. Never close a replacement connection that
    // reused this handle, or a link this component is not tracking.
    if (!connected_ || conn_handle_ != conn_handle || connection_generation == 0 ||
        connection_generation_ != connection_generation)
      return;
    // A cancelled link's no-IRK result was dropped, and the advertising intent
    // now belongs to a newer run, which must not wait out this link's
    // cooldown. The disconnect below still coalesces with the one cancellation
    // queued for this link.
    if (advertising_requested_ && !capture_cancelled_locked_(connection_generation)) {
      adv_restart_time_ = deadline_after(now_ms(), TimingConfig::ADV_SUPPRESS_RESTART_DELAY_MS);
    }
  }
  terminate_bounded_(conn_handle, connection_generation, context);
}

void IRKCaptureComponent::terminate_bounded_(uint16_t conn_handle, uint32_t generation,
                                             const char* context, bool for_rotation) {
  // Never wait on the host queue or terminate a bare handle from the main task.
  // A disconnect/reconnect can reuse it while an operation waits for a lock.
  {
    MutexGuard lock(state_mutex_);
    if (!connected_ || conn_handle_ != conn_handle || connection_generation_ != generation) return;
    if (pending_terminate_handle_ != conn_handle || pending_terminate_generation_ != generation) {
      pending_terminate_busy_logged_ = false;
      pending_terminate_for_rotation_ = false;
      pending_terminate_retries_ = 0;
      pending_terminate_retry_time_ = 0;
    }
    pending_terminate_handle_ = conn_handle;
    pending_terminate_generation_ = generation;
    pending_terminate_context_ = context;
    // Preserve rotation ownership if a completion disconnect coalesces onto an
    // already-queued rotation disconnect for the same connection.
    pending_terminate_for_rotation_ |= for_rotation;
  }
  notify_bond_clear_();
}

void IRKCaptureComponent::handle_pending_termination_() {
  // Only the NimBLE task runs this callback, serially with connect/disconnect.
  // A busy control lock leaves the request pending for the next worker wake-up.
  {
    MutexGuard lock(state_mutex_);
    if (pending_terminate_handle_ == BLE_HS_CONN_HANDLE_NONE) return;
    // Honor retry backoff if another maintenance wake-up runs early.
    if (pending_terminate_retry_time_ != 0 &&
        !deadline_reached(now_ms(), pending_terminate_retry_time_))
      return;
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
    uint32_t connection_generation;
    uint32_t rotation_generation;
    bool rotation_waiting;
    bool rotation_owned;
    {
      MutexGuard lock(state_mutex_);
      handle = pending_terminate_handle_;
      context = pending_terminate_context_;
      connection_generation = pending_terminate_generation_;
      rotation_owned = pending_terminate_for_rotation_;
      const bool current = connected_ && handle != BLE_HS_CONN_HANDLE_NONE &&
                           conn_handle_ == handle &&
                           connection_generation_ == connection_generation;
      pending_terminate_handle_ = BLE_HS_CONN_HANDLE_NONE;
      pending_terminate_for_rotation_ = false;
      pending_terminate_retry_time_ = 0;
      if (!current) {
        pending_terminate_retries_ = 0;
        return;
      }
      rotation_waiting = mac_rotation_state_ == MacRotationState::REQUESTED;
      rotation_generation = mac_rotation_generation_;
    }
    rc = ble_gap_terminate(handle, BLE_ERR_REM_USER_CONN_TERM);
    {
      MutexGuard lock(state_mutex_);
      bool retry_scheduled = false;
      // Brief buffer contention in the host is not a reason to give up.
      const bool transient = transient_terminate_error(rc);
      if (rotation_waiting && rc != 0 && rc != BLE_HS_EALREADY &&
          mac_rotation_state_ == MacRotationState::REQUESTED &&
          mac_rotation_generation_ == rotation_generation && conn_handle_ == handle &&
          connection_generation_ == connection_generation) {
        // The host callback is serialized with GAP. ENOTCONN here means the
        // tracked descriptor is already gone, so finish the disconnect. Only a
        // termination initiated for the rotation can abort it on other errors.
        if (rc == BLE_HS_ENOTCONN) {
          finish_disconnect_(now_ms());
        } else if (rotation_owned) {
          // Retry the refresh on the same budget the timeout recovery uses.
          if (transient && pending_terminate_retries_ + 1 < TimingConfig::TERMINATE_MAX_ATTEMPTS) {
            ++pending_terminate_retries_;
            pending_terminate_handle_ = handle;
            pending_terminate_generation_ = connection_generation;
            pending_terminate_context_ = context;
            pending_terminate_for_rotation_ = true;
            pending_terminate_retry_time_ =
                deadline_after(now_ms(), TimingConfig::TERMINATE_RETRY_MS);
            retry_scheduled = true;
          } else {
            abort_rotation_locked_();
          }
        }
      }
      // Closes after a capture or a no-IRK result end polling, so nothing else
      // would close the link before the 90 s pairing timeout, which would also
      // delete the bond. Retry them on the same budget. Leave a request queued
      // meanwhile in charge of the next attempt.
      if (!retry_scheduled && !rotation_owned && transient && connected_ &&
          conn_handle_ == handle && connection_generation_ == connection_generation &&
          pending_terminate_handle_ == BLE_HS_CONN_HANDLE_NONE &&
          pending_terminate_retries_ + 1 < TimingConfig::TERMINATE_MAX_ATTEMPTS) {
        ++pending_terminate_retries_;
        pending_terminate_handle_ = handle;
        pending_terminate_generation_ = connection_generation;
        pending_terminate_context_ = context;
        pending_terminate_retry_time_ = deadline_after(now_ms(), TimingConfig::TERMINATE_RETRY_MS);
        retry_scheduled = true;
      }
      if (!retry_scheduled) pending_terminate_retries_ = 0;
    }
  }
  log_terminate_result(rc, context);
}

void IRKCaptureComponent::report_capture_failure_(uint32_t connection_generation) {
  // Surface failed pairings to Status/wizard for STATUS_RESULT_HOLD_MS.
  MutexGuard lock(state_mutex_);
  // Ignore links this component is not tracking, and never replace a result
  // this connection has already produced.
  if (connection_generation == 0 || connection_generation != connection_generation_ ||
      capture_cancelled_locked_(connection_generation) ||
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
