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

//======================== NAMING CONVENTIONS ========================
/*
This codebase follows ESPHome style guidelines:
-    Member variables: trailing underscore (conn_handle_, advertising_)
-    Local variables/parameters: snake_case (conn_handle, peer_id)
-    Functions: snake_case (try_get_irk, start_advertising)
-    Constants: UPPER_SNAKE_CASE or constexpr with descriptive names
-    Classes: PascalCase (IRKCaptureComponent)
*/

//======================== THREADING MODEL ========================
/*
CRITICAL: This component runs in a multi-threaded environment:
1. NimBLE task: Executes all GAP event callbacks (connect, disconnect,
enc_change, etc.)
2. ESPHome main task: Executes loop(), setup(), and UI callbacks (switch,
button, text)
3. Bond-clear worker: Enqueues NimBLE wake-ups; may wait for queue capacity,
but never holds a component mutex while doing so or deletes bonds itself
4. esp_http_server task (only with irk_wizard): calls the thread-safe getters,
set_next_capture_label(), set_stop_after_capture() and request_bond_clear().
Everything else the wizard does is deferred to the main task.

THREAD SAFETY RULES:
-    ALL reads/writes to shared state MUST use state_mutex_
-    The state it protects is listed next to state_mutex_ in irk_capture.h
-    RAII MutexGuard class ensures exception-safe lock/unlock
-    Mutex MUST be released before calling BLE stack APIs (prevents deadlock)
-    Entity publish_state() is NOT safe to call from the NimBLE task. BLE-context
     code stages values into the pending_* fields (under state_mutex_) and the
     ESPHome main loop() publishes them via flush_pending_publishes_()
-    host_synced_ uses std::atomic<bool> (writes from NimBLE reset/sync callbacks,
     reads from the main loop)

IMPORTANT: Do NOT rely on "aligned writes are atomic" - always use mutex for
shared state to ensure:
  1. Memory barriers (visibility across cores)
  2. Compiler optimization safety (prevents register caching)
  3. Consistent threading model (easier maintenance)

ESPHome component lifecycle guarantees:
-    setup() completes before loop() starts
-    All set_*() configuration calls complete before setup()
-    Parent pointers (IRKCaptureText, IRKCaptureSwitch, etc.) are always valid
     after setup() IF those optional components are configured in YAML
*/

//======================== ERROR HANDLING STRATEGY ========================
/*
Logging severity levels used throughout:
-    ESP_LOGE: Critical failures that prevent IRK capture (NimBLE init fails,
store errors)
-    ESP_LOGW: Recoverable issues or unexpected states (pairing retry, IRK not
yet available)
-    ESP_LOGI: Normal operational events (connection, disconnection, IRK
captured)
-    ESP_LOGD: Detailed debugging (GAP events, state transitions, timer
scheduling)

Philosophy: Log enough to debug pairing issues remotely, but avoid spam during
normal operation.
*/

//======================== STATE MACHINE OVERVIEW ========================
/*
Component operates as a state machine with these primary states:

1. IDLE (advertising_ = false, connected_ = false)
   - Waiting for user to enable advertising switch
   - Transitions to ADVERTISING when start_advertising() called

2. ADVERTISING (advertising_ = true, connected_ = false)
   - Broadcasting BLE advertisements with current MAC and device name
   - Transitions to CONNECTED on BLE_GAP_EVENT_CONNECT

3. CONNECTED (connected_ = true, advertising_ = false)
   - Peer device connected, initiating security/pairing
   - Transitions to ENCRYPTED when BLE_GAP_EVENT_ENC_CHANGE succeeds

4. ENCRYPTED (connected_ = true, enc_ready_ = true)
   - Secure connection established, polling NimBLE's bond store for the IRK
   - Stops IRK polling after 45s; transitions to DISCONNECTING when an IRK is
     captured, when the bond turns out to hold no IRK, or when the global 90s
     pairing timeout terminates the link

5. DISCONNECTING (connected_ = false, adv_restart_time_ may be set)
   - IRK retrieval attempts continue via timers after disconnect
   - Transitions back to ADVERTISING at once, or when adv_restart_time_ expires

Key flags:
-    advertising_: BLE stack is actively advertising
-    connected_: Peer is currently connected
-    enc_ready_: Encryption/pairing completed successfully
-    irk_gave_up_: Exceeded 45s timeout without capturing IRK
-    adv_restart_time_: Nonzero while a restart of advertising is being held
off (reconnect-loop defense, DHKey failure, no-IRK result, timeout cooldown,
start retry)
-    sec_retry_done_: The one security retry for this connection has been used
(it fires when encryption has not completed ~2s after connect)
*/

// Timing configuration (all values in ms)
struct TimingConfig {
  static constexpr uint32_t LOOP_MIN_INTERVAL_MS = 50;  // Minimum time between loop() executions
  static constexpr uint32_t HR_NOTIFY_INTERVAL_MS =
      1000;  // Heart rate notification interval (keeps connection alive)
  static constexpr uint32_t ENC_TO_FIRST_TRY_DELAY_MS =
      1000;  // Delay from encryption to first IRK poll (NimBLE stores the
             // keys after it reports ENC_CHANGE)
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
  static constexpr uint32_t ADV_RETRY_MS = 1000;  // Backoff after advertising start contention
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
      200;  // Longest the main loop waits for ble_op_mutex_ before retrying later
  static constexpr uint32_t STATUS_RESULT_HOLD_MS =
      4000;  // How long the status sensor holds "captured" / "no_irk" / "failed"
};

// GATT service and characteristic UUIDs
static constexpr uint16_t UUID_SVC_HEART_RATE = 0x180D;
static constexpr uint16_t UUID_CHR_HEART_RATE_MEASUREMENT = 0x2A37;
static constexpr uint16_t UUID_SVC_DEVICE_INFO = 0x180A;
static constexpr uint16_t UUID_CHR_MANUFACTURER_NAME = 0x2A29;
static constexpr uint16_t UUID_CHR_MODEL_NUMBER = 0x2A24;
static constexpr uint16_t UUID_SVC_BATTERY = 0x180F;
static constexpr uint16_t UUID_CHR_BATTERY_LEVEL = 0x2A19;
static constexpr uint16_t UUID_SVC_HID = 0x1812;  // Human Interface Device (for Keyboard profile)
static constexpr uint16_t UUID_CHR_HID_PROTOCOL_MODE = 0x2A4E;

// BLE Appearance values
static constexpr uint16_t APPEARANCE_HEART_RATE_SENSOR = 0x0340;

//======================== IRK lifecycle (for readers) ========================
/*
Connect → Initiate security
ENC_CHANGE (success) → immediate IRK read from store; if available: publish &
disconnect (tested working behavior); otherwise schedule a late read at +5s.
A stored bond with no IRK is also a final outcome: publish "no IRK", disconnect,
and hold advertising off briefly so a bonded peer cannot reconnect in a loop.
ENC_CHANGE (failure) → delete the failing peer's bond and terminate the
connection to force fresh pairing on reconnect; on a DHKey check failure,
additionally suppress the next advertising cycle to prod the peer into resetting.
(Separately, retry_security_if_needed() initiates security once more if
encryption has not completed ~2s after connect — this is timeout-driven, not a
response to an ENC_CHANGE failure.)
DISCONNECT → if this connection has not produced a result yet: immediate store
read, and a delayed read at +800ms if that is inconclusive. Then restart
advertising unless a hold-off is pending or the capture limit was reached.
While connected (post ENC) → poll every 1s starting at +1s, stop polling after
45s, and disconnect immediately whenever an IRK is captured.
All address reporting uses the peer identity address; IRK hex is reversed for
parity with Arduino output.

Why this lifecycle: Maintains compatibility (immediate disconnect after capture)
while adding robustness for timing variations across different BLE peer
implementations.
*/

//======================== UUIDs ========================

static const ble_uuid16_t UUID_SVC_HR = BLE_UUID16_INIT(UUID_SVC_HEART_RATE);
static const ble_uuid16_t UUID_CHR_HR_MEAS = BLE_UUID16_INIT(UUID_CHR_HEART_RATE_MEASUREMENT);

static const ble_uuid16_t UUID_SVC_DEVINFO = BLE_UUID16_INIT(UUID_SVC_DEVICE_INFO);
static const ble_uuid16_t UUID_CHR_MANUF = BLE_UUID16_INIT(UUID_CHR_MANUFACTURER_NAME);
static const ble_uuid16_t UUID_CHR_MODEL = BLE_UUID16_INIT(UUID_CHR_MODEL_NUMBER);

static const ble_uuid16_t UUID_SVC_BAS = BLE_UUID16_INIT(UUID_SVC_BATTERY);
static const ble_uuid16_t UUID_CHR_BATT_LVL = BLE_UUID16_INIT(UUID_CHR_BATTERY_LEVEL);

// HID service for Keyboard profile
static const ble_uuid16_t UUID_SVC_HID_BLE = BLE_UUID16_INIT(UUID_SVC_HID);
static const ble_uuid16_t UUID_CHR_HID_PROTO = BLE_UUID16_INIT(UUID_CHR_HID_PROTOCOL_MODE);

// Optional protected service/characteristic to force pairing via READ_ENC
static const ble_uuid128_t UUID_SVC_PROT = BLE_UUID128_INIT(
    0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF, 0xFE, 0xDC, 0xBA, 0x09, 0x87, 0x65, 0x43, 0x21);
static const ble_uuid128_t UUID_CHR_PROT = BLE_UUID128_INIT(
    0x21, 0x43, 0x65, 0x87, 0x09, 0xBA, 0xDC, 0xFE, 0xEF, 0xCD, 0xAB, 0x90, 0x78, 0x56, 0x34, 0x12);

//======================== Forward decls ========================

// Defined in irk_capture_history.cpp.
void publish_and_log_irk(IRKCaptureComponent* self, const ble_addr_t& peer_id_addr,
                         const std::string& irk_hex, const char* context_tag,
                         uint32_t connection_generation, CaptureOrigin origin);

// Defined in irk_capture_identity.cpp.
bool persist_ble_profile(BLEProfile profile);

//======================== Small utilities (readability only)
//========================

static inline uint32_t now_ms() {
  return (uint32_t) (esp_timer_get_time() / 1000ULL);
}

// Wraparound-safe "has deadline passed?" check for 32-bit ms timestamps.
// Returns true when `now` is at or past `deadline` (handles 49.5-day overflow).
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

// Serializes BLE host control operations across the ESPHome and NimBLE tasks.
// The same guard as MutexGuard; the name says which lock is being taken.
//
//   BleOpGuard g(ble_op_mutex_);            // waits without limit (NimBLE callbacks)
//   BleOpGuard g(ble_op_mutex_, pdMS_TO_TICKS(TimingConfig::BLE_OP_LOCK_WAIT_MS));  // bounded
//   if (!g.acquired()) { ...; return; }     // bail on timeout
using BleOpGuard = MutexGuard;

// Security Manager IO capability names (Core spec Vol 3 Part H 3.5.1).
static inline const char* io_cap_str(uint8_t io_cap) {
  switch (io_cap) {
    case BLE_HS_IO_DISPLAY_ONLY:
      return "DisplayOnly";
    case BLE_HS_IO_DISPLAY_YESNO:
      return "DisplayYesNo";
    case BLE_HS_IO_KEYBOARD_ONLY:
      return "KeyboardOnly";
    case BLE_HS_IO_NO_INPUT_OUTPUT:
      return "NoInputNoOutput";
    case BLE_HS_IO_KEYBOARD_DISPLAY:
      return "KeyboardDisplay";
    default:
      return "unknown";
  }
}

// Key distribution bits as "ENC|ID|SIGN|LINK"; "none" when no bit is set.
// Writes into `buf`, which must hold at least 20 bytes, and returns it.
static inline const char* key_dist_str(uint8_t dist, char* buf, size_t len) {
  static const struct {
    uint8_t bit;
    const char* name;
  } kBits[] = {
    { BLE_SM_PAIR_KEY_DIST_ENC, "ENC" },
    { BLE_SM_PAIR_KEY_DIST_ID, "ID" },
    { BLE_SM_PAIR_KEY_DIST_SIGN, "SIGN" },
    { BLE_SM_PAIR_KEY_DIST_LINK, "LINK" },
  };
  size_t pos = 0;
  buf[0] = '\0';
  for (const auto& b : kBits) {
    if ((dist & b.bit) == 0) continue;
    int n = snprintf(buf + pos, len - pos, "%s%s", pos ? "|" : "", b.name);
    if (n < 0 || (size_t) n >= len - pos) break;
    pos += (size_t) n;
  }
  if (pos == 0) snprintf(buf, len, "none");
  return buf;
}

static inline void log_sm_config(const char* label = "SM config") {
  char ours[20];
  char theirs[20];
  ESP_LOGI(TAG,
           "%s: bonding=%d mitm=%d sc=%d io_cap=%d (%s) our_key_dist=0x%02X (%s) "
           "their_key_dist=0x%02X (%s)",
           label, (int) ble_hs_cfg.sm_bonding, (int) ble_hs_cfg.sm_mitm, (int) ble_hs_cfg.sm_sc,
           (int) ble_hs_cfg.sm_io_cap, io_cap_str(ble_hs_cfg.sm_io_cap),
           (unsigned) ble_hs_cfg.sm_our_key_dist,
           key_dist_str(ble_hs_cfg.sm_our_key_dist, ours, sizeof(ours)),
           (unsigned) ble_hs_cfg.sm_their_key_dist,
           key_dist_str(ble_hs_cfg.sm_their_key_dist, theirs, sizeof(theirs)));
}

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
static inline void log_terminate_result(int rc, const char* context) {
  if (rc == 0) {
    ESP_LOGD(TAG, "Disconnect %s requested", context);
  } else if (rc == BLE_HS_EALREADY) {
    ESP_LOGD(TAG, "Disconnect %s already in progress", context);
  } else if (rc != 0) {
    ESP_LOGW(TAG, "ble_gap_terminate %s rc=%d", context, rc);
  }
}

static inline void log_no_irk_for_peer(const ble_addr_t& peer_id) {
  ESP_LOGW(TAG, "Bond for %s has no usable IRK; key distribution may be incomplete or unsupported",
           addr_to_str(peer_id).c_str());
}

}  // namespace irk_capture
}  // namespace esphome

#endif  // USE_ESP32
