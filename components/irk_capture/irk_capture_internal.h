#pragma once

// Private to this component's .cpp files: the includes, constants and small
// helpers they share, and the threading, state-machine and IRK-lifecycle notes
// that apply to all of them. Nothing outside irk_capture should include it.
//
// IRKCaptureComponent is one class defined across several files:
//   irk_capture.cpp              setup, loop, staged publishing, status
//   irk_capture_ble.cpp          NimBLE setup, GATT tables and callbacks
//   irk_capture_gap.cpp          GAP event handlers, connect/disconnect
//   irk_capture_advertising.cpp  advertising start, stop and retry
//   irk_capture_identity.cpp     BLE name, profile and MAC rotation
//   irk_capture_session.cpp      IRK reads, timers, timeouts and outcomes
//   irk_capture_history.cpp      dedup cache, IRK output, bond clear, history
//   irk_capture_entities.cpp     switch, button, text and select glue

#include "irk_capture.h"

// ESP32-only implementation - requires Bluetooth hardware
#ifdef USE_ESP32

#include <esp_random.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <host/ble_store.h>
#include <nvs_flash.h>
#include <store/config/ble_store_config.h>

#include <algorithm>
#include <cinttypes>
#include <cstring>
#include <limits>

#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "mutex_guard.h"

namespace esphome {
namespace irk_capture {

static const char* const TAG = "irk_capture";
static constexpr char HEX[] = "0123456789abcdef";

//======================== Threading model ========================
// Tasks touching this component:
// 1. NimBLE host task: GAP/GATT/reset/sync callbacks and queued host work
//    (pending disconnects, bond clears, MAC-rotation cleanup).
// 2. ESPHome main task: setup(), loop(), HA entity callbacks, deferred wizard ops.
// 3. Maintenance worker ("irk_bond_clear"): puts bond_clear_event_ onto NimBLE's
//    queue (may block on older NPL ports) without holding any component mutex.
// 4. esp_http_server task (irk_wizard): thread-safe getters,
//    cancel_capture(), set_next_capture_label(), set_stop_after_capture(), request_bond_clear().
//
// Rules:
// - Guard all mutable shared state with state_mutex_ (see irk_capture.h).
//   host_synced_ is std::atomic<bool>.
// - Release state_mutex_ before calling BLE stack, NVS, or logging APIs.
// - Entity publish_state() is main-task only; other contexts stage pending_*
//   fields for flush_pending_publishes_() in loop().

// Timing configuration (all values in ms)
struct TimingConfig {
  static constexpr uint32_t LOOP_MIN_INTERVAL_MS = 50;  // Minimum time between loop() executions
  static constexpr uint32_t HR_NOTIFY_INTERVAL_MS =
      1000;  // Heart rate notification interval (keeps connection alive)
  static constexpr uint32_t ENC_TO_FIRST_TRY_DELAY_MS =
      1000;  // Delay after ENC_CHANGE before first fallback IRK poll
  static constexpr uint32_t ENC_TRY_INTERVAL_MS =
      1000;  // Interval between IRK polling attempts while connected
  static constexpr uint32_t ENC_GIVE_UP_AFTER_MS =
      45000;  // Maximum time to poll for IRK before giving up
  static constexpr uint32_t POST_DISC_DELAY_MS =
      800;  // Delay after disconnect for the fallback IRK check (lets a bond
            // write that was still in flight finish)
  static constexpr uint32_t ENC_LATE_READ_DELAY_MS =
      5000;  // Fallback IRK check after encryption succeeded but the bond
             // could not be read yet
  static constexpr uint32_t SEC_RETRY_DELAY_MS = 2000;  // Retry security once if encryption has not
                                                        // completed this long after connect
  static constexpr uint32_t SEC_TIMEOUT_MS = 20000;  // Timeout for encryption to complete (assumes
                                                     // peer forgot pairing)
  static constexpr uint32_t ADV_SUPPRESS_RESTART_DELAY_MS =
      5000;  // Delay before auto-restarting advertising after a hold-off
             // (reconnect-loop defense, DHKey failure, no-IRK result)
  static constexpr uint32_t ADV_RETRY_MS = 1000;  // Base backoff for advertising start/stop retries
  static constexpr uint8_t ADV_FAST_ATTEMPTS = 5;
  static constexpr uint32_t ADV_SLOW_RETRY_MS = 60000;
  static constexpr uint32_t ADV_FAILURE_LOG_INTERVAL_MS = 300000;
  static constexpr uint32_t PAIRING_TOTAL_TIMEOUT_MS = 90000;  // Global pairing timeout (90s max)
  static constexpr uint32_t TERMINATE_RETRY_MS = 1000;         // Backoff after terminate failure
  static constexpr uint32_t TERMINATE_PENDING_GRACE_MS =
      2000;  // Wait for asynchronous disconnect callback before retrying
  static constexpr uint8_t TERMINATE_MAX_ATTEMPTS = 5;
  static constexpr uint32_t TIMEOUT_COOLDOWN_MS =
      5000;  // Cooldown after pairing timeout before re-advertising (prevents
             // rapid-fire loop)
  static constexpr uint32_t MIN_REPUBLISH_INTERVAL_MS =
      60000;  // Min time between republishing same IRK (60s)
  static constexpr uint8_t MAC_ROTATION_MAX_RETRIES = 10;  // Max retries for MAC rotation
  static constexpr uint32_t MAC_ROTATION_RETRY_MS = 250;   // First retry delay; doubles each retry
  static constexpr uint32_t MAC_ROTATION_RETRY_MAX_MS = 2000;  // Ceiling for that backoff
  static constexpr uint32_t MAC_ROTATION_SETTLE_DELAY_MS =
      500;  // Delay after adv stop before MAC change
  static constexpr uint32_t HOST_SYNC_TIMEOUT_MS =
      30000;  // How long the Bluetooth host may stay unsynced before it is reported
  static constexpr uint32_t HOST_SYNC_LOG_INTERVAL_MS = 300000;  // Repeat of that report
  static constexpr uint32_t HOST_SYNC_REBOOT_MS =
      300000;  // Unsynced this long with nothing captured: reboot to recover
  static constexpr uint32_t BLE_OP_LOCK_WAIT_MS =
      200;  // Bounded wait for ble_op_mutex_ before deferring/retrying
  static constexpr uint32_t STATUS_RESULT_HOLD_MS =
      4000;  // How long the status sensor holds "captured" / "no_irk" / "failed"
};

//======================== IRK lifecycle ========================
// States: IDLE -> ADVERTISING -> CONNECTED -> ENCRYPTED -> IDLE/ADVERTISING.
// Connect -> initiate security, with one retry after 2 s if encryption is pending.
// ENC_CHANGE (ok) -> read the peer's bond immediately. An IRK or confirmed no-IRK
//   result requests disconnect. A no-IRK result also holds off advertising.
//   If the read is inconclusive, check again at +5 s and poll once a second,
//   starting at +1 s and giving up after about 45 s.
// ENC_CHANGE (fail) -> delete only this peer's bond, report failure, and request
//   disconnect. DHKey failure also delays the next advertising restart.
// Polling queues disconnects. ENC_CHANGE can defer termination if the BLE lock is busy.
// Timeout recovery starts if encryption is still pending 20 s after security
//   initiation, or if the link remains open 90 s after connection.
// DISCONNECT -> if no IRK was observed, read the bond immediately and retry at
//   +800 ms if inconclusive. Restart advertising unless limited, OFF, or held off.
// Report the peer's identity address and reverse IRK hex for Arduino parity.

//======================== Forward decls ========================

// Defined in irk_capture_history.cpp.
void publish_and_log_irk(IRKCaptureComponent* self, const ble_addr_t& peer_id_addr,
                         const std::string& irk_hex, const char* context_tag,
                         uint32_t connection_generation, CaptureOrigin origin, IrkCheck check,
                         bool host_task);

// Defined in irk_capture_identity.cpp.
bool persist_ble_profile(BLEProfile profile);

//======================== Utilities ========================

static inline uint32_t now_ms() {
  return (uint32_t) (esp_timer_get_time() / 1000ULL);
}

// Wraparound-safe "has deadline passed?" check for 32-bit ms timestamps.
// Returns true when `now` is at or past `deadline` (handles 49.7-day overflow).
// IMPORTANT: Only valid when deadline was set within the last ~24.8 days.
static inline bool deadline_reached(uint32_t now, uint32_t deadline) {
  return (int32_t) (now - deadline) >= 0;
}

// The deadline `delay_ms` after `now`. Zero means "no deadline", so it is never returned.
static inline uint32_t deadline_after(uint32_t now, uint32_t delay_ms) {
  const uint32_t deadline = now + delay_ms;
  return deadline != 0 ? deadline : 1;
}

// Milliseconds from `since` to `now`; zero when another task stamped `since` after `now` was read.
static inline uint32_t elapsed_ms(uint32_t now, uint32_t since) {
  const int32_t elapsed = (int32_t) (now - since);
  return elapsed > 0 ? (uint32_t) elapsed : 0;
}

// Hex formatter: reversed byte order (matches Arduino BLE library convention
// for IRK display)
static inline std::string to_hex_rev(const uint8_t* data, size_t len) {
  std::string out;
  out.reserve(len * 2);
  for (int i = (int) len - 1; i >= 0; --i) {
    uint8_t c = data[i];
    out.push_back(HEX[(c >> 4) & 0xF]);
    out.push_back(HEX[c & 0xF]);
  }
  return out;
}

// NimBLE stores an address little-endian; this prints it most significant byte first.
static inline std::string mac_to_str(const uint8_t mac[6]) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2],
           mac[1], mac[0]);
  return std::string(buf);
}

static inline std::string addr_to_str(const ble_addr_t& a) {
  return mac_to_str(a.val);
}

// A BLE static random address: the top two bits of the most significant byte
// (the last one, as NimBLE stores it) are 11. That is the only structural
// requirement (Core Spec Vol 6, Part B, 1.3.2); the IEEE 802 unicast/multicast
// bit does not apply to BLE. The result can never be all-zero.
static inline void make_static_random_addr(uint8_t out[6]) {
  esp_fill_random(out, 6);
  out[5] |= 0xC0;
}

static inline bool addr_is_zero(const ble_addr_t& a) {
  return a.type == 0 && a.val[0] == 0 && a.val[1] == 0 && a.val[2] == 0 && a.val[3] == 0 &&
         a.val[4] == 0 && a.val[5] == 0;
}

static inline bool addr_equal(const ble_addr_t& lhs, const ble_addr_t& rhs) {
  return lhs.type == rhs.type && std::memcmp(lhs.val, rhs.val, sizeof(lhs.val)) == 0;
}

// MutexGuard alias for ble_op_mutex_. The NimBLE host task never waits for it:
// queued host work, disconnects, and advertising start/stop called from GAP
// callbacks try it without waiting and hand contended work to the worker or to
// loop(). Only the main task uses the bounded BLE_OP_LOCK_WAIT_MS wait.
using BleOpGuard = MutexGuard;

void log_sm_config(const char* label = "SM config");

static inline bool get_own_addr(uint8_t out_mac[6], uint8_t* out_type = nullptr) {
  uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
  int rc = ble_hs_id_infer_auto(0, &own_addr_type);
  if (rc != 0) return false;

  ble_addr_t addr {};
  rc = ble_hs_id_copy_addr(own_addr_type, addr.val, nullptr);
  if (rc != 0) return false;

  for (int i = 0; i < 6; ++i) out_mac[i] = addr.val[i];
  if (out_type) *out_type = own_addr_type;
  return true;
}

static inline void log_mac(const char* prefix) {
  uint8_t mac[6];
  uint8_t type;
  if (get_own_addr(mac, &type)) {
    ESP_LOGI(TAG, "%s MAC: %s (type=%u)", prefix, mac_to_str(mac).c_str(), type);
  } else {
    ESP_LOGW(TAG, "%s MAC: unknown (could not read)", prefix);
  }
}

// ble_gap_terminate() returns EALREADY when a disconnect is already under way,
// for example one started by a timeout or a MAC rotation. That is not a failure.
// Host resource contention that clears by itself, so a disconnect is worth
// retrying. ENOMEM_EVT means NimBLE ran out of HCI command buffers.
static inline bool transient_terminate_error(int rc) {
  return rc == BLE_HS_EBUSY || rc == BLE_HS_ENOMEM || rc == BLE_HS_ENOMEM_EVT;
}

static inline void log_terminate_result(int rc, const char* context) {
  if (rc == 0) {
    ESP_LOGD(TAG, "Disconnect %s requested", context);
  } else if (rc == BLE_HS_EALREADY) {
    ESP_LOGD(TAG, "Disconnect %s already in progress", context);
  } else if (rc != 0) {
    ESP_LOGW(TAG, "ble_gap_terminate %s rc=%d", context, rc);
  }
}

// Independent outcomes: an empty store does not prove resolving keys are gone.
struct BondCleanupResult {
  int store_rc { 0 };
  int resolving_rc { 0 };
  bool ok() const {
    return store_rc == 0 && resolving_rc == 0;
  }
  const char* reason() const {
    if (store_rc != 0 && resolving_rc != 0)
      return "Bond storage and identity keys could not be cleared. "
             "Press Forget All Bonds to retry; "
             "restart the ESP32 if identity-key cleanup keeps failing.";
    if (store_rc != 0) return "Bond storage could not be cleared; press Forget All Bonds to retry.";
    if (resolving_rc != 0)
      return "Identity keys could not be cleared. Press Forget All Bonds to retry; "
             "restart the ESP32 if identity-key cleanup keeps failing.";
    return "";
  }
};

// Clears stored bonds and resolving-list entries; reports each step's failure.
BondCleanupResult clear_bonds_and_resolving_list();

static inline void log_no_irk_for_peer(const ble_addr_t& peer_id) {
  ESP_LOGW(TAG, "Bond for %s has no usable IRK; key distribution may be incomplete or unsupported",
           addr_to_str(peer_id).c_str());
}

}  // namespace irk_capture
}  // namespace esphome

#endif  // USE_ESP32
