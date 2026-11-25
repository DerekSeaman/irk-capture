// What has been captured: the deduplication cache and publish decision, the
// single IRK output path, bond clearing, and the history JSON.

#include "irk_capture_internal.h"

// ESP32-only implementation - requires Bluetooth hardware
#ifdef USE_ESP32

// NimBLE exports these functions but declares them in ble_hs_pvcy_priv.h,
// outside ESP-IDF's public include path. Keep this SDK boundary here and cover
// its linkage in the firmware build matrix. Reapplying a non-null local IRK
// clears the entire resolving list (including orphaned entries) on both the
// controller and host privacy implementations, and returns the operation error.
extern "C" int ble_hs_pvcy_our_irk(const uint8_t** out_irk);
extern "C" int ble_hs_pvcy_set_our_irk(const uint8_t* irk);

namespace esphome {
namespace irk_capture {

// Call with advertising stopped, no connection and neither component mutex.
// ble_gap_unpair() can swallow a resolving-list error on ESP-IDF, so its return
// code cannot prove cleanup. Always reset the full list, even with an empty
// store: an earlier failed cleanup may have removed the only record of a peer.
BondCleanupResult clear_bonds_and_resolving_list() {
  BondCleanupResult result;
  const uint8_t* current_irk = nullptr;
  result.resolving_rc = ble_hs_pvcy_our_irk(&current_irk);
  if (result.resolving_rc == 0) {
    if (current_irk == nullptr) {
      result.resolving_rc = BLE_HS_EINVAL;
    } else {
      uint8_t local_irk[16];
      std::memcpy(local_irk, current_irk, sizeof(local_irk));
      result.resolving_rc = ble_hs_pvcy_set_our_irk(local_irk);
    }
  }
  result.store_rc = ble_store_clear();
  return result;
}

static void log_spacer() {
  ESP_LOGI(TAG, " ");
}
static void log_banner(const char* context_tag) {
  ESP_LOGI(TAG, "*** IRK CAPTURED *** (%s)", (context_tag ? context_tag : "unknown"));
}

//======================== IRK Deduplication ========================

/**
 * @brief Records one IRK observation and decides whether to publish it
 * (deduplication + rate limiting)
 *
 * CRITICAL: Caller MUST hold state_mutex_ before calling this function.
 * This function modifies irk_cache_: entries, reconnect counters, timestamps
 * and visible history. It therefore does not log: it returns what it
 * decided, and the caller reports that with
 * log_publish_decision() once the lock is released.
 *
 * MEMORY SAFETY: This function prevents duplicate entries in irk_cache_ by
 * checking if the identity address already exists before adding. Even if the
 * same device reconnects 100 times, it will only have ONE entry in the cache
 * (updated in-place). cache_limit_() caps the cache, so memory stays bounded
 * on ESP32-C3 even when max_captures is 0 (unlimited).
 *
 * @param irk_hex IRK in hex string format
 * @param addr MAC address string
 * @param addr_type NimBLE identity-address type
 * @param connection_generation Monotonic connection identifier used to coalesce
 * extraction paths from the same connection
 * @param force_pairing_publish True for an explicit pairing or REPEAT_PAIRING;
 * bypasses rate limiting
 * @param check This observation's address check, kept with the entry
 * @return The outcome and the details needed to log it; publish is false for a
 * duplicate path, stale connection, rate-limited or over-limit reconnect
 */
PublishDecision IRKCaptureComponent::record_observation_locked_(
    const std::string& irk_hex, const std::string& addr, uint8_t addr_type,
    uint32_t connection_generation, bool force_pairing_publish, IrkCheck check) {
  // PRECONDITION: Caller holds state_mutex_
  using Reason = PublishDecision::Reason;
  PublishDecision decision;
  // 64-bit: a 32-bit difference wraps after 49.7 days, which would make a
  // device last published that long ago look as if it had just been published.
  const uint64_t now = (uint64_t) (esp_timer_get_time() / 1000ULL);

  // Identity address, rather than IRK, defines a unique device. A legitimate
  // re-pair can rotate the IRK without becoming a second device.
  for (auto& entry : irk_cache_) {
    if (entry.mac_addr == addr && entry.addr_type == addr_type) {
      // ENC_CHANGE, DISCONNECT, and delayed polling can all find the same IRK.
      // Count the connection once and silently coalesce the remaining paths.
      if (connection_generation != 0) {
        if (entry.last_observed_generation == connection_generation) {
          decision.reason = Reason::DUPLICATE_PATH;
          return decision;
        }
        // A delayed timer from an older connection must not move the cache back
        // to an earlier generation or count as another reconnect.
        if (entry.last_observed_generation != 0 &&
            static_cast<int32_t>(connection_generation - entry.last_observed_generation) < 0) {
          decision.reason = Reason::STALE_CONNECTION;
          return decision;
        }
        entry.last_observed_generation = connection_generation;
      }

      // A changed IRK for a known identity is security-significant and should
      // always be shown, but it is still the same unique device. A fresh
      // pairing or REPEAT_PAIRING is an explicit user action: always surface
      // its IRK, even when unchanged and inside the normal interval.
      const bool key_changed = entry.irk_hex != irk_hex;
      // A new key starts with this observation's check. For the same key, a
      // check that could not run must not hide an earlier result.
      if (key_changed || check == IrkCheck::VERIFIED || check == IrkCheck::MISMATCH) {
        entry.verification = check;
      }
      if (key_changed || force_pairing_publish) {
        entry.irk_hex = irk_hex;
        entry.reconnect_count = 0;
        entry.reconnect_limit_reported = false;
        entry.last_published_ms = now;
        restore_capture_history_(entry, decision);
        decision.reason = key_changed ? Reason::KEY_CHANGED : Reason::PAIRED_AGAIN;
        decision.publish = true;
        return decision;
      }

      if (entry.reconnect_count < std::numeric_limits<uint16_t>::max()) {
        entry.reconnect_count++;
      }
      decision.reconnect_count = entry.reconnect_count;

      // Count reconnecting connections, not the number of extraction paths.
      if (entry.reconnect_count > 5) {
        decision.reason = Reason::RECONNECT_LIMIT;
        decision.limit_just_reached = !entry.reconnect_limit_reported;
        entry.reconnect_limit_reported = true;
        decision.stop_adv = true;
        return decision;
      }

      // Rate limit ordinary bonded reconnects per device.
      const uint64_t since_published = now - entry.last_published_ms;
      decision.since_published_ms =
          since_published > UINT32_MAX ? UINT32_MAX : (uint32_t) since_published;
      if (since_published < TimingConfig::MIN_REPUBLISH_INTERVAL_MS) {
        decision.reason = Reason::RATE_LIMITED;
        return decision;
      }

      entry.last_published_ms = now;
      restore_capture_history_(entry, decision);
      decision.reason = Reason::RECONNECT;
      decision.publish = true;
      return decision;
    }
  }

  // New IRK - add to cache with FIFO eviction. cache_limit_() bounds memory,
  // including in unlimited mode.
  const size_t cache_limit = cache_limit_();
  if (irk_cache_.size() >= cache_limit) {
    // Evict oldest (FIFO). Note: documented behavior; intentional to cap memory
    // and keep UX predictable.
    irk_cache_.erase(irk_cache_.begin());
    decision.evicted_oldest = true;
  }
  IRKCacheEntry entry;
  entry.irk_hex = irk_hex;
  entry.mac_addr = addr;
  entry.addr_type = addr_type;
  entry.last_published_ms = now;
  entry.last_observed_generation = connection_generation;
  entry.label = next_capture_label_;
  entry.verification = check;
  irk_cache_.push_back(std::move(entry));
  if (!next_capture_label_.empty()) {
    decision.label = next_capture_label_;
    consume_capture_label_();  // Consume-once: next device starts unlabeled
  }
  decision.reason = Reason::NEW_DEVICE;
  decision.cache_size = irk_cache_.size();
  decision.cache_limit = cache_limit;
  decision.publish = true;
  return decision;
}

void IRKCaptureComponent::restore_capture_history_(IRKCacheEntry& entry,
                                                   PublishDecision& out_decision) {
  // A new publication can restore a forgotten history row, but duplicate
  // extraction paths from the same connection must leave it hidden.
  if (entry.in_history) return;
  entry.in_history = true;
  entry.label = next_capture_label_;
  if (!next_capture_label_.empty()) {
    out_decision.label = next_capture_label_;
    consume_capture_label_();
  }
}

// Reports a record_observation_locked_() decision. Runs after state_mutex_ is released:
// formatting and the UART must not stall the NimBLE task (see MutexGuard).
static void log_publish_decision(const PublishDecision& decision, const std::string& addr) {
  using Reason = PublishDecision::Reason;
  switch (decision.reason) {
    case Reason::DUPLICATE_PATH:
    case Reason::STALE_CONNECTION:
      break;  // Routine coalescing of read paths; nothing worth a line.
    case Reason::KEY_CHANGED:
      ESP_LOGI(TAG, "IRK updated for known identity %s", addr.c_str());
      break;
    case Reason::PAIRED_AGAIN:
      ESP_LOGI(TAG, "Pairing completed; publishing IRK again");
      break;
    case Reason::RECONNECT_LIMIT:
      if (decision.limit_just_reached) {
        ESP_LOGW(TAG,
                 "Reconnect limit reached (%u connections). Device keeps reconnecting; "
                 "unpair from Bluetooth settings to stop.",
                 decision.reconnect_count);
      }
      break;
    case Reason::RATE_LIMITED:
      // Debug level so every bonded reconnect explains itself: without this
      // line a skipped republish leaves no trace, and the reconnect count in
      // the next "Re-publishing" line appears to jump.
      ESP_LOGD(TAG, "Bonded reconnect (%u/5): IRK published %" PRIu32 " ms ago; not republishing",
               decision.reconnect_count, decision.since_published_ms);
      break;
    case Reason::RECONNECT:
      ESP_LOGI(TAG, "Re-publishing IRK after bonded reconnect (%u/5)", decision.reconnect_count);
      break;
    case Reason::NEW_DEVICE:
      if (decision.evicted_oldest) {
        ESP_LOGD(TAG, "IRK cache full (%zu entries), evicted the oldest entry",
                 decision.cache_limit);
      }
      ESP_LOGD(TAG, "New IRK added to cache (total: %zu/%zu)", decision.cache_size,
               decision.cache_limit);
      break;
  }
  if (!decision.label.empty()) {
    ESP_LOGI(TAG, "Tagged capture with label '%s'", decision.label.c_str());
  }
}

//======================== Output helpers (centralized) ========================

// Reports whether the IRK generated the address the peer connected from. Only
// a mismatch is a warning: a peer on a fixed address has nothing to check.
static void log_irk_check(IrkCheck check) {
  switch (check) {
    case IrkCheck::VERIFIED:
      ESP_LOGI(TAG, "IRK check: verified (it resolves the address the device connected from)");
      break;
    case IrkCheck::MISMATCH:
      ESP_LOGW(TAG,
               "IRK check: MISMATCH. The device connected from a resolvable private address "
               "this IRK does not generate; forget the ESP32 on the device and capture again");
      break;
    case IrkCheck::NOT_RESOLVABLE:
      ESP_LOGI(TAG,
               "IRK check: unverified (the device connected from an address an IRK does not "
               "generate)");
      break;
    case IrkCheck::UNCHECKED:
      ESP_LOGI(TAG, "IRK check: unverified (the connection address could not be checked)");
      break;
  }
}

/**
 * @brief Centralized IRK output helper - logs and publishes IRK to Home
 * Assistant sensors
 * @param self Pointer to IRKCaptureComponent instance (null is ignored)
 * @param peer_id_addr Peer's identity address (stable across reconnections)
 * @param irk_hex IRK in hex string format (already reversed for output
 * compatibility)
 * @param context_tag Read path, for the log: "ENC_CHANGE", "DISC_IMMEDIATE",
 * "POLL_CONNECTED", "DISC_DELAYED" or "ENC_LATE"
 * @param connection_generation Monotonic connection identifier
 * @param origin How the connection began; a fresh pairing or re-pair is always
 * published, a bonded reconnect is rate-limited
 * @param check Whether the IRK resolves the address the peer connected from
 *
 * This is the single point of IRK output, ensuring consistent formatting across
 * all capture paths. Called from ENC_CHANGE, DISCONNECT, the polling loop and
 * both delayed-read timers (post-disconnect and late-encryption).
 */
void publish_and_log_irk(IRKCaptureComponent* self, const ble_addr_t& peer_id_addr,
                         const std::string& irk_hex, const char* context_tag,
                         uint32_t connection_generation, CaptureOrigin origin, IrkCheck check) {
  if (!self) return;  // Early return if no component instance

  const std::string addr_str = addr_to_str(peer_id_addr);

  // THREAD-SAFE: Check deduplication AND increment counter under mutex
  // CRITICAL: record_observation_locked_() assumes caller holds mutex (non-recursive
  // mutex!) All irk_cache_ operations, counter increments, and advertising
  // checks happen atomically
  uint32_t current_events = 0;
  uint32_t current_unique = 0;
  bool max_reached = false;
  bool stop_after_capture_hit = false;
  bool should_publish;
  PublishDecision decision;
  bool force_pairing_publish = false;
  bool is_repair = false;
  bool capture_event = false;
  {
    MutexGuard lock(self->state_mutex_);

    force_pairing_publish = origin == CaptureOrigin::FRESH || origin == CaptureOrigin::REPAIR;
    is_repair = origin == CaptureOrigin::REPAIR;

    // A valid observation makes delayed fallback reads for this connection
    // redundant. Cancel them before they can create log or counter noise.
    self->cancel_peer_timers_(peer_id_addr, connection_generation);

    decision = self->record_observation_locked_(
        irk_hex, addr_str, peer_id_addr.type, connection_generation, force_pairing_publish, check);
    should_publish = decision.publish;
    const bool is_new_device = decision.reason == PublishDecision::Reason::NEW_DEVICE;
    const bool key_changed = decision.reason == PublishDecision::Reason::KEY_CHANGED;

    if (should_publish) {
      self->capture_events_++;
      if (is_new_device) {
        self->unique_devices_++;
      }
      current_events = self->capture_events_;
      current_unique = self->unique_devices_;

      // max_captures limits unique devices. Re-pairing the same device remains
      // visible without consuming another slot.
      max_reached = self->capture_limit_reached_locked_();

      // A genuine capture: a new device, changed key, or explicit (re)pairing. A bonded
      // reconnect republishing a key it already gave us is not one.
      capture_event = is_new_device || key_changed || force_pairing_publish;

      // Wizard "one-shot" mode: independent of continuous_mode/max_captures,
      // stop advertising after a capture so the flow can hand off to
      // confirmation. An earlier device reconnecting in the background must
      // not turn advertising off before the intended device has paired.
      stop_after_capture_hit = self->stop_after_capture_ && capture_event;
    }
  }  // Release mutex before slow logging/publishing operations

  log_publish_decision(decision, addr_str);

  // Reconnect-loop defense: hold advertising off for a few seconds. The
  // deadline covers both cases: a live link (the disconnect handler sees a
  // restart already scheduled) and a delayed read that finds the radio on.
  if (decision.stop_adv) {
    {
      MutexGuard lock(self->state_mutex_);
      if (self->advertising_requested_) {
        self->adv_restart_time_ =
            deadline_after(now_ms(), TimingConfig::ADV_SUPPRESS_RESTART_DELAY_MS);
      }
    }
    // stop_advertising() checks GAP itself and stages no Home Assistant
    // update, so the guard is not about double stops. It skips the BLE lock
    // wait, and the deferred-stop retry and warning a lock timeout would arm,
    // when the radio is already known to be off (as on a live link, or from
    // delayed-read timers such as DISC_DELAYED).
    if (self->is_advertising()) {
      self->stop_advertising();
    }
    if (decision.limit_just_reached) {
      ESP_LOGI(TAG,
               "Reconnect limit reached; suppressing advertising to break the reconnect loop.");
    }
  }

  // Called even for coalesced or rate-limited observations: the sensor decides
  // whether this may replace what is displayed (see publish_irk_to_sensors()).
  self->publish_irk_to_sensors(irk_hex, addr_str, connection_generation, capture_event,
                               should_publish && (max_reached || stop_after_capture_hit), check);

  // Skip duplicate capture logs (deduplication happened under mutex).
  if (!should_publish) {
    return;
  }

  log_spacer();
  log_banner(is_repair ? "REPAIR" : context_tag);
  ESP_LOGI(TAG, "Identity Address: %s (type=%u)", addr_str.c_str(), (unsigned) peer_id_addr.type);
  ESP_LOGI(TAG, "IRK: %s", irk_hex.c_str());
  log_irk_check(check);
  ESP_LOGI(TAG, "Capture events this session: %" PRIu32, current_events);
  ESP_LOGI(TAG, "Unique devices this session: %" PRIu32, current_unique);
  if (max_reached) {
    if (!self->continuous_mode_) {
      ESP_LOGI(TAG, "Single capture mode: advertising will stop after disconnect");
    } else {
      ESP_LOGI(TAG, "Max unique devices (%u) reached - advertising will stop after disconnect",
               self->max_captures_);
    }
  } else if (stop_after_capture_hit) {
    ESP_LOGI(TAG, "Stop-after-capture enabled: advertising will stop after disconnect");
  }

  log_spacer();
}

void IRKCaptureComponent::forget_all_bonds() {
  request_bond_clear();
}

BondClearResult IRKCaptureComponent::get_bond_clear_result() {
  MutexGuard lock(state_mutex_);
  return bond_clear_result_;
}

BondClearResult IRKCaptureComponent::request_bond_clear() {
  BondClearResult result;
  {
    MutexGuard lock(state_mutex_);
    // These entries also coalesce disconnect/timer reads and account for unique
    // devices. Hide history instead of discarding that session bookkeeping.
    for (auto& entry : irk_cache_) {
      entry.in_history = false;
      entry.label.clear();
    }
    if (bond_clear_pending_) return bond_clear_result_;
    if (++bond_clear_result_.operation_id == 0) ++bond_clear_result_.operation_id;
    bond_clear_result_.history_cleared = true;
    bond_clear_result_.reason.clear();
    if (!bond_clear_task_) {
      bond_clear_result_.state = BondClearState::FAILED;
      bond_clear_result_.reason = "Bond-clear worker unavailable; restart the device to retry.";
    } else if (!host_synced_) {
      bond_clear_result_.state = BondClearState::BUSY;
      bond_clear_result_.reason = "Bluetooth is not ready; retry once ready.";
    } else if (mac_rotation_state_ != MacRotationState::IDLE) {
      bond_clear_result_.state = BondClearState::BUSY;
      bond_clear_result_.reason = "Bluetooth identity is changing; retry once idle.";
    } else {
      bond_clear_result_.state = BondClearState::PENDING;
      bond_clear_pending_ = true;
      bond_clear_host_generation_ = host_generation_;
    }
    result = bond_clear_result_;
  }
  if (result.state == BondClearState::PENDING) {
    notify_bond_clear_();
  } else {
    ESP_LOGW(TAG, "Forget All Bonds: history cleared; stored bonds retained. %s",
             result.reason.c_str());
  }
  return result;
}

void IRKCaptureComponent::notify_bond_clear_() {
  {
    MutexGuard lock(state_mutex_);
    // A disconnect waiting out its retry backoff has nothing to do yet; loop()
    // calls this every tick, so waking the host for it would be pure churn.
    const bool terminate_due = pending_terminate_handle_ != BLE_HS_CONN_HANDLE_NONE &&
                               (pending_terminate_retry_time_ == 0 ||
                                deadline_reached(now_ms(), pending_terminate_retry_time_));
    if (!bond_clear_task_ ||
        (!bond_clear_pending_ && !mac_rotation_cleanup_pending_ && !terminate_due))
      return;
  }
  // Task notifications do not wait for queue capacity. A reset racing this
  // notification is harmless: the worker rechecks the pending request.
  xTaskNotify(bond_clear_task_, 1, eSetBits);
}

void IRKCaptureComponent::queue_bond_clear_() {
  {
    MutexGuard lock(state_mutex_);
    if (!host_synced_) return;
    const bool clear_current =
        bond_clear_pending_ && host_generation_ == bond_clear_host_generation_;
    if (!clear_current && !mac_rotation_cleanup_pending_ &&
        pending_terminate_handle_ == BLE_HS_CONN_HANDLE_NONE)
      return;
  }
  // Worker task only, with neither component mutex held. This can block on
  // older SDKs. The event is just a wake-up, not ownership of a request: after
  // reset/resync its callback must consult the current pending state again.
  ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &bond_clear_event_);
}

void IRKCaptureComponent::handle_forget_bonds_() {
  // Runs on the same NimBLE event queue as connect/security callbacks. A peer
  // arriving before this event is checked here; one arriving afterwards cannot
  // start pairing until this callback returns. The pending flag also blocks
  // main-task advertising starts and MAC rotation throughout the store wipe.
  bool can_clear = false;
  bool lock_busy = false;
  bool stopped_advertising = false;
  int stop_rc = BLE_HS_EBUSY;
  uint32_t operation_id;
  uint32_t host_generation;
  BondClearState state = BondClearState::BUSY;
  std::string reason = "Bluetooth maintenance is busy; retry once idle.";
  {
    MutexGuard lock(state_mutex_);
    if (!bond_clear_pending_) return;  // A loop retry may have queued a duplicate.
    operation_id = bond_clear_result_.operation_id;
    host_generation = bond_clear_host_generation_;
  }
  // Call only while holding state_mutex_. A reset may cancel this request and
  // a new one may start while an unlocked BLE/store operation is returning.
  const auto still_current = [&]() {
    return bond_clear_pending_ && bond_clear_result_.operation_id == operation_id &&
           bond_clear_host_generation_ == host_generation && host_generation_ == host_generation;
  };
  {
    // Do not block the host behind a main-task operation that may need it.
    BleOpGuard ble_lock(ble_op_mutex_, 0);
    lock_busy = !ble_lock.acquired();
    if (ble_lock.acquired()) {
      {
        MutexGuard lock(state_mutex_);
        if (!still_current()) return;
        can_clear = host_synced_ && !connected_ && mac_rotation_state_ == MacRotationState::IDLE;
        if (connected_) reason = "A device is connected; disconnect it and retry.";
      }
      if (can_clear) {
        const bool was_active = ble_gap_adv_active() != 0;
        stop_rc = was_active ? ble_gap_adv_stop() : BLE_HS_EALREADY;
        const bool active = ble_gap_adv_active() != 0;
        stopped_advertising = was_active && !active;
        MutexGuard lock(state_mutex_);
        if (!still_current()) return;
        advertising_ = active;
        can_clear = !active && host_synced_ && !connected_ &&
                    mac_rotation_state_ == MacRotationState::IDLE &&
                    (stop_rc == 0 || stop_rc == BLE_HS_EALREADY || stop_rc == BLE_HS_EINVAL);
        if (connected_) {
          reason = "A device is connected; disconnect it and retry.";
        } else if (active ||
                   (stop_rc != 0 && stop_rc != BLE_HS_EALREADY && stop_rc != BLE_HS_EINVAL)) {
          state = stop_rc == BLE_HS_EBUSY ? BondClearState::BUSY : BondClearState::FAILED;
          reason = "Bluetooth advertising could not stop; retry once idle.";
        }
      }
    }
  }
  if (stopped_advertising) {
    // Logged outside ble_op_mutex_ and only when this handler actually stopped
    // the radio.
    ESP_LOGI(TAG, "Advertising stopped to clear bonds");
  }

  if (can_clear) {
    // Store work takes NimBLE's host lock; hold neither component mutex.
    const BondCleanupResult cleanup = clear_bonds_and_resolving_list();
    state = cleanup.ok() ? BondClearState::SUCCEEDED : BondClearState::FAILED;
    reason = cleanup.reason();
  }
  {
    MutexGuard lock(state_mutex_);
    if (!still_current()) return;
    bond_clear_pending_ = false;
    bond_clear_result_.state = state;
    bond_clear_result_.reason = reason;
  }
  if (state == BondClearState::SUCCEEDED) {
    ESP_LOGI(TAG, "All BLE bonds cleared");
  } else {
    ESP_LOGW(TAG, "Forget All Bonds: history cleared; pairings may remain. %s", reason.c_str());
  }
  // While the request was pending, start_advertising() deferred every start to
  // this point, so it must run whatever the outcome: an OFF request or capture
  // limit reached meanwhile is re-evaluated there. With the BLE lock busy, the
  // start would wait for that lock on the host task; hand it to the main loop.
  if (lock_busy) {
    MutexGuard lock(state_mutex_);
    if (advertising_requested_ && !advertising_ && adv_restart_time_ == 0) {
      adv_restart_time_ = deadline_after(now_ms(), TimingConfig::ADV_RETRY_MS);
    }
    return;
  }
  start_advertising();
}

std::string IRKCaptureComponent::build_history_json() {
  // Copy the cache under the mutex, then format JSON outside it — string
  // building is comparatively slow and must not block the NimBLE task.
  std::vector<IRKCacheEntry> cache_copy;
  {
    MutexGuard lock(state_mutex_);
    cache_copy = irk_cache_;
  }
  return format_history_json_(cache_copy);
}

std::string IRKCaptureComponent::format_history_json_(const std::vector<IRKCacheEntry>& cache) {
  // {"mac":"<17>","irk":"<32>","label":"<up to 24>","reconnects":<up to 5>,
  // "verification":"<up to 10>"}, is 94 bytes of framing and fixed-width values
  // plus the label, count and check.
  static constexpr size_t ENTRY_RESERVE = 94 + CAPTURE_LABEL_MAX_LEN + 5 + 28;
  std::string json;
  json.reserve(cache.size() * ENTRY_RESERVE + 2);
  json += "[";
  bool first = true;
  for (const auto& entry : cache) {
    if (!entry.in_history) continue;
    if (!first) json += ",";
    first = false;
    // mac_addr/irk_hex are always hex/colon characters and label is
    // restricted to a safe charset (see set_next_capture_label()), so none of
    // these values can contain a character that needs JSON escaping.
    json += "{\"mac\":\"";
    json += entry.mac_addr;
    json += "\",\"irk\":\"";
    json += entry.irk_hex;
    json += "\",\"label\":\"";
    json += entry.label;
    json += "\",\"reconnects\":";
    json += std::to_string(entry.reconnect_count);
    json += ",\"verification\":\"";
    json += irk_check_name(entry.verification);
    json += "\"}";
  }
  json += "]";

  return json;
}

}  // namespace irk_capture
}  // namespace esphome

#endif  // USE_ESP32
