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

#include <cinttypes>
#include <cstring>
#include <limits>

#include "esphome/core/application.h"
#include "esphome/core/log.h"

// Some ESP-IDF 5.x package variants omit this prototype from headers. NimBLE's
// implementation returns void; declaring it as int reads an undefined return
// register (observed as nonzero on ESP32-C6).
extern "C" void ble_store_config_init(void);

namespace esphome {
namespace irk_capture {

static const char* const TAG = "irk_capture";
static constexpr char VERSION[] = "1.7.0";
static constexpr char HEX[] = "0123456789abcdef";

// Global instance pointer for NimBLE callbacks that don't accept user args
static IRKCaptureComponent* g_irk_instance = nullptr;

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

THREAD SAFETY RULES:
-    ALL reads/writes to shared state MUST use state_mutex_
-    Protected by state_mutex_ (FreeRTOS mutex):
     * post_disc_timers_ / late_enc_timers_ (queued timer targets, peers, and generations)
     * conn_handle_ (connection handle)
     * connected_ (connection state flag)
     * advertising_ (advertising state flag)
     * advertising_requested_ (persistent user intent)
     * pairing_start_time_ (pairing timeout tracking)
     * ble_name_, manufacturer_name_ (device names)
     * mac_rotation_state_ (MAC rotation state machine)
     * pending_mac_ (6-byte pre-generated MAC buffer)
     * suppress_next_adv_ (advertising suppression flag)
     * adv_restart_time_ (advertising restart timer)
     * capture_events_ / unique_devices_ (session counters - NOT atomic)
     * irk_cache_ (deduplication vector - push_back/erase NOT thread-safe)
     * connection_generation_ / pairing_generation_ / repair_generation_
       (capture coalescing state)
     * enc_ready_, enc_time_ (encryption/pairing completion state)
     * sec_retry_done_, sec_init_time_ms_ (security retry state)
     * irk_gave_up_, irk_last_try_ms_ (IRK polling state)
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
   - Secure connection established, polling for IRK in NVS bond store
   - Stops IRK polling after 45s; transitions to DISCONNECTING when an IRK is
     captured or when the global 90s pairing timeout terminates the link

5. DISCONNECTING (connected_ = false, suppress_next_adv_ may be true)
   - IRK retrieval attempts continue via timers after disconnect
   - Transitions back to ADVERTISING after suppression period expires

Key flags:
-    advertising_: BLE stack is actively advertising
-    connected_: Peer is currently connected
-    enc_ready_: Encryption/pairing completed successfully
-    irk_gave_up_: Exceeded 45s timeout without capturing IRK
-    suppress_next_adv_: Prevent immediate re-advertising after successful IRK
capture
-    sec_retry_done_: Already attempted one security retry after ENC failure
*/

// Timing configuration (all values in ms)
struct TimingConfig {
  static constexpr uint32_t LOOP_MIN_INTERVAL_MS = 50;  // Minimum time between loop() executions
  static constexpr uint32_t HR_NOTIFY_INTERVAL_MS =
      1000;  // Heart rate notification interval (keeps connection alive)
  static constexpr uint32_t ENC_TO_FIRST_TRY_DELAY_MS =
      1000;  // Delay from encryption to first IRK poll (allows NVS write to
             // complete)
  static constexpr uint32_t ENC_TRY_INTERVAL_MS =
      1000;  // Interval between IRK polling attempts while connected
  static constexpr uint32_t ENC_GIVE_UP_AFTER_MS =
      45000;  // Maximum time to poll for IRK before giving up
  static constexpr uint32_t POST_DISC_DELAY_MS =
      800;  // Delay after disconnect for deferred IRK check (allows NVS flush)
  static constexpr uint32_t ENC_LATE_READ_DELAY_MS =
      5000;  // Extended delay for IRK check after failed encryption
  static constexpr uint32_t SEC_RETRY_DELAY_MS =
      2000;  // Delay before retrying security after encryption failure
  static constexpr uint32_t SEC_TIMEOUT_MS = 20000;  // Timeout for encryption to complete (assumes
                                                     // peer forgot pairing)
  static constexpr uint32_t ADV_SUPPRESS_RESTART_DELAY_MS =
      5000;  // Delay before auto-restarting advertising after a suppression
             // (reconnect-loop break / IRK re-publish)
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
  static constexpr uint32_t MAC_ROTATION_SETTLE_DELAY_MS =
      500;  // Delay after adv stop before MAC change
  static constexpr uint32_t STATUS_RESULT_HOLD_MS =
      4000;  // How long the status sensor holds "captured" / "no_irk"
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

//======================== IRK lifecycle (for readers) ========================
/*
Connect → Initiate security
ENC_CHANGE (success) → immediate IRK read from store; if available: publish &
disconnect (tested working behavior); otherwise schedule a late read at +5s.
ENC_CHANGE (failure) → delete the failing peer's bond and terminate the
connection to force fresh pairing on reconnect; on a DHKey check failure,
additionally suppress the next advertising cycle to prod the peer into resetting.
(Separately, retry_security_if_needed() initiates security once more if
encryption has not completed ~2s after connect — this is timeout-driven, not a
response to an ENC_CHANGE failure.)
DISCONNECT → immediate store read; schedule delayed read at +800ms; restart
advertising.
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

int chr_read_devinfo(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt,
                     void* arg);
int chr_read_batt(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt,
                  void* arg);
int chr_read_hr(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt,
                void* arg);
int chr_read_hid_protocol(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt* ctxt, void* arg);
int chr_read_protected(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt* ctxt, void* arg);

// GAP event handlers (forward decls)
class IRKCaptureComponent;
int handle_gap_connect(IRKCaptureComponent* self, struct ble_gap_event* ev);
int handle_gap_disconnect(IRKCaptureComponent* self, struct ble_gap_event* ev);
int handle_gap_enc_change(IRKCaptureComponent* self, struct ble_gap_event* ev);
int handle_gap_repeat_pairing(IRKCaptureComponent* self, struct ble_gap_event* ev);

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

// Hex formatter: reversed byte order (matches Arduino BLE library convention
// for IRK display)
static std::string to_hex_rev(const uint8_t* data, size_t len) {
  std::string out;
  out.reserve(len * 2);
  for (int i = (int) len - 1; i >= 0; --i) {
    uint8_t c = data[i];
    out.push_back(HEX[(c >> 4) & 0xF]);
    out.push_back(HEX[c & 0xF]);
  }
  return out;
}

static std::string addr_to_str(const ble_addr_t& a) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", a.val[5], a.val[4], a.val[3],
           a.val[2], a.val[1], a.val[0]);
  return std::string(buf);
}

static bool addr_is_zero(const ble_addr_t& a) {
  return a.type == 0 && a.val[0] == 0 && a.val[1] == 0 && a.val[2] == 0 && a.val[3] == 0 &&
         a.val[4] == 0 && a.val[5] == 0;
}

static bool addr_equal(const ble_addr_t& lhs, const ble_addr_t& rhs) {
  return lhs.type == rhs.type && std::memcmp(lhs.val, rhs.val, sizeof(lhs.val)) == 0;
}

static bool is_encrypted(uint16_t conn_handle) {
  struct ble_gap_conn_desc d;
  if (ble_gap_conn_find(conn_handle, &d) == 0) {
    return d.sec_state.encrypted;
  }
  return false;
}

//======================== Thread Safety: RAII Mutex Guard
//========================
/*
RAII wrapper for FreeRTOS mutex to ensure exception-safe locking.
Used to protect shared state accessed by both NimBLE task and ESPHome main task.

Protected state:
- post_disc_timers_ / late_enc_timers_ (queued peer ids and deadlines)
- conn_handle_ (connection state)
- advertising_ (advertising state)
- pairing_start_time_ (timeout tracking)

CRITICAL PERFORMANCE OPTIMIZATION:
Always minimize mutex hold time by copying data out before releasing the lock,
then performing slow operations (logging, NVS access) AFTER the lock is
released.

BAD (blocks NimBLE task during slow UART logging):
  {
    MutexGuard lock(state_mutex_);
    peer_id = post_disc_timers_[0].peer_id;
    ESP_LOGD(TAG, "Peer: %s", addr_to_str(peer_id).c_str()); // SLOW - UART
bottleneck!
  }

GOOD (minimal lock hold time):
  ble_addr_t peer_id_copy;
  {
    MutexGuard lock(state_mutex_);
    peer_id_copy = post_disc_timers_[0].peer_id;  // Fast memory copy
  }  // Lock released immediately
  ESP_LOGD(TAG, "Peer: %s", addr_to_str(peer_id_copy).c_str());  // Safe - no
lock held

Why: On ESP32-C3 (single core), holding a mutex during logging can cause the
NimBLE task to miss critical timing windows (e.g., supervision timeout), leading
to dropped connections.

Usage pattern:
  {
    MutexGuard lock(state_mutex_);
    // ONLY fast memory operations here (reads, writes, simple arithmetic)
  }  // Mutex automatically released
  // Logging, NVS access, string formatting happen here (outside critical
section)
*/
class MutexGuard {
 public:
  explicit MutexGuard(SemaphoreHandle_t mutex) : mutex_(mutex) {
    if (mutex_) {
      xSemaphoreTake(mutex_, portMAX_DELAY);
    }
  }

  ~MutexGuard() {
    if (mutex_) {
      xSemaphoreGive(mutex_);
    }
  }

  // Disable copy and move
  MutexGuard(const MutexGuard&) = delete;
  MutexGuard& operator=(const MutexGuard&) = delete;

 private:
  SemaphoreHandle_t mutex_;
};

// Serialize BLE host control operations across ESPHome and NimBLE task
// contexts.
//
// Usage:
//   BleOpGuard g(mutex_);                         // portMAX_DELAY (NimBLE callbacks)
//   BleOpGuard g(mutex_, pdMS_TO_TICKS(200));     // timeout (ESPHome main loop)
//   if (!g.acquired()) { ESP_LOGW(...); return; } // bail on timeout
class BleOpGuard {
 public:
  explicit BleOpGuard(SemaphoreHandle_t mutex, TickType_t timeout = portMAX_DELAY)
      : mutex_(mutex), acquired_(false) {
    if (mutex_) {
      acquired_ = (xSemaphoreTake(mutex_, timeout) == pdTRUE);
    }
  }

  ~BleOpGuard() {
    if (mutex_ && acquired_) {
      xSemaphoreGive(mutex_);
    }
  }

  // Returns false if the mutex was not acquired (timed out).
  bool acquired() const {
    return acquired_;
  }

  BleOpGuard(const BleOpGuard&) = delete;
  BleOpGuard& operator=(const BleOpGuard&) = delete;

 private:
  SemaphoreHandle_t mutex_;
  bool acquired_;
};

static void hr_measurement_sample(uint8_t* buf, size_t* len) {
  buf[0] = 0x00;  // Flags byte: bit0=0 (UINT8 format), bit1-2=0 (sensor contact
                  // not supported),
                  //             bit3=0 (no energy expended), bit4=0 (RR
                  //             interval not present)
  buf[1] = (uint8_t) (60 + (esp_random() % 40));  // 60-99 bpm
  *len = 2;
}

static void log_conn_desc(uint16_t conn_handle) {
  struct ble_gap_conn_desc d;
  if (ble_gap_conn_find(conn_handle, &d) == 0) {
    // Security state
    ESP_LOGI(TAG, "sec: enc=%d bonded=%d auth=%d key_size=%u", d.sec_state.encrypted,
             d.sec_state.bonded, d.sec_state.authenticated, d.sec_state.key_size);

    // Addresses
    ESP_LOGI(TAG, "peer ota=%s type=%d", addr_to_str(d.peer_ota_addr).c_str(),
             d.peer_ota_addr.type);
    ESP_LOGI(TAG, "peer id =%s type=%d", addr_to_str(d.peer_id_addr).c_str(), d.peer_id_addr.type);

    // Connection parameters (helpful for Android watch debugging)
    // supervision_timeout is the link supervision timeout per BLE spec
    ESP_LOGD(TAG, "conn params: interval=%u latency=%u supervision_timeout=%u", d.conn_itvl,
             d.conn_latency, d.supervision_timeout);

    // Role and features
    ESP_LOGD(TAG, "role=%s our_ota=%s", d.role == BLE_GAP_ROLE_MASTER ? "master" : "slave",
             addr_to_str(d.our_ota_addr).c_str());
  }
}

static void log_sm_config() {
  ESP_LOGI(TAG,
           "SM config: bonding=%d mitm=%d sc=%d io_cap=%d our_key_dist=0x%02X "
           "their_key_dist=0x%02X",
           (int) ble_hs_cfg.sm_bonding, (int) ble_hs_cfg.sm_mitm, (int) ble_hs_cfg.sm_sc,
           (int) ble_hs_cfg.sm_io_cap, (unsigned) ble_hs_cfg.sm_our_key_dist,
           (unsigned) ble_hs_cfg.sm_their_key_dist);
}

static bool get_own_addr(uint8_t out_mac[6], uint8_t* out_type = nullptr) {
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

static void log_mac(const char* prefix) {
  uint8_t mac[6];
  uint8_t type;
  if (get_own_addr(mac, &type)) {
    ble_addr_t addr {};
    for (int i = 0; i < 6; ++i) addr.val[i] = mac[i];
    ESP_LOGI(TAG, "%s MAC: %s (type=%u)", prefix, addr_to_str(addr).c_str(), type);
  } else {
    ESP_LOGW(TAG, "%s MAC: unknown (could not read)", prefix);
  }
}

// Helper to safely append const string or default value
static int append_const_string_or_default(struct os_mbuf* om, const char* str,
                                          const char* default_str) {
  // Safely append string to mbuf, using default if str is nullptr or empty
  const char* val = (str && str[0] != '\0') ? str : default_str;
  return os_mbuf_append(om, val, strlen(val));
}

static void log_spacer() {
  ESP_LOGI(TAG, " ");
}
static void log_banner(const char* context_tag) {
  ESP_LOGI(TAG, "*** IRK CAPTURED *** (%s)", (context_tag ? context_tag : "unknown"));
}

//======================== IRK Validation ========================

/**
 * @brief Validates that an IRK is not all-zero or all-FF
 * (invalid/uninitialized)
 * @param irk 16-byte IRK array
 * @return true if IRK is valid, false if invalid
 */
bool IRKCaptureComponent::is_valid_irk(const uint8_t irk[16]) {
  // Reject all-zero (invalid) and all-FF (uninitialized) IRKs in a single pass
  bool all_zero = true;
  bool all_ff = true;
  for (int i = 0; i < 16; i++) {
    if (irk[i] != 0x00) all_zero = false;
    if (irk[i] != 0xFF) all_ff = false;
    if (!all_zero && !all_ff) break;  // Early exit: IRK is valid
  }
  if (all_zero) {
    ESP_LOGW(TAG, "Rejected all-zero IRK (invalid)");
    return false;
  }
  if (all_ff) {
    ESP_LOGW(TAG, "Rejected all-FF IRK (uninitialized)");
    return false;
  }
  return true;
}

//======================== IRK Deduplication ========================

/**
 * @brief Checks if IRK should be published (deduplication + rate limiting)
 *
 * CRITICAL: Caller MUST hold state_mutex_ before calling this function.
 * This function modifies irk_cache_.
 *
 * MEMORY SAFETY: This function prevents duplicate entries in irk_cache_ by
 * checking if the identity address already exists before adding. Even if the
 * same device reconnects 100 times, it will only have ONE entry in the cache
 * (updated in-place). Cache cap derived from max(max_captures_, 10) prevents
 * unbounded memory growth on ESP32-C3.
 *
 * @param irk_hex IRK in hex string format
 * @param addr MAC address string
 * @param addr_type NimBLE identity-address type
 * @param connection_generation Monotonic connection identifier used to coalesce
 * extraction paths from the same connection
 * @param force_pairing_publish True for an explicit pairing or REPEAT_PAIRING;
 * bypasses rate limiting
 * @param out_should_stop_adv [out] Set to true if caller should stop
 * advertising (limit reached)
 * @param out_is_new_device [out] True when a new identity address was cached
 * @param out_limit_just_reached [out] True only on the first over-limit reconnect
 * @return true if should publish, false if duplicate/rate-limited
 */
bool IRKCaptureComponent::should_publish_irk(const std::string& irk_hex, const std::string& addr,
                                             uint8_t addr_type, uint32_t connection_generation,
                                             bool force_pairing_publish, bool& out_should_stop_adv,
                                             bool& out_is_new_device,
                                             bool& out_limit_just_reached) {
  // PRECONDITION: Caller holds state_mutex_
  out_should_stop_adv = false;
  out_is_new_device = false;
  out_limit_just_reached = false;
  uint32_t now = now_ms();

  // Identity address, rather than IRK, defines a unique device. A legitimate
  // re-pair can rotate the IRK without becoming a second device.
  for (auto& entry : irk_cache_) {
    if (entry.mac_addr == addr && entry.addr_type == addr_type) {
      // ENC_CHANGE, DISCONNECT, and delayed polling can all find the same IRK.
      // Count the connection once and silently coalesce the remaining paths.
      if (connection_generation != 0) {
        if (entry.last_observed_generation == connection_generation) {
          return false;
        }
        // A delayed timer from an older connection must not move the cache back
        // to an earlier generation or count as another reconnect.
        if (entry.last_observed_generation != 0 &&
            static_cast<int32_t>(connection_generation - entry.last_observed_generation) < 0) {
          return false;
        }
        entry.last_observed_generation = connection_generation;
      }

      // A changed IRK for a known identity is security-significant and should
      // always be shown, but it is still the same unique device.
      if (entry.irk_hex != irk_hex) {
        entry.irk_hex = irk_hex;
        entry.reconnect_count = 0;
        entry.reconnect_limit_reported = false;
        entry.last_published_ms = now;
        restore_capture_history_(entry);
        ESP_LOGI(TAG, "IRK updated for known identity %s", addr.c_str());
        return true;
      }

      // A fresh pairing or REPEAT_PAIRING is an explicit user action. Always
      // surface its IRK, even when unchanged and inside the normal interval.
      if (force_pairing_publish) {
        entry.reconnect_count = 0;
        entry.reconnect_limit_reported = false;
        entry.last_published_ms = now;
        restore_capture_history_(entry);
        ESP_LOGI(TAG, "Pairing completed; publishing IRK again");
        return true;
      }

      if (entry.reconnect_count < std::numeric_limits<uint16_t>::max()) {
        entry.reconnect_count++;
      }

      // Count reconnecting connections, not the number of extraction paths.
      if (entry.reconnect_count > 5) {
        if (!entry.reconnect_limit_reported) {
          ESP_LOGW(TAG,
                   "Reconnect limit reached (%u connections). Device keeps reconnecting; "
                   "unpair from Bluetooth settings to stop.",
                   entry.reconnect_count);
          entry.reconnect_limit_reported = true;
          out_limit_just_reached = true;
        }
        out_should_stop_adv = true;
        return false;
      }

      // Rate limit ordinary bonded reconnects per device. This is intentionally
      // verbose-only: routine suppression should not dominate debug logs.
      if ((now - entry.last_published_ms) < TimingConfig::MIN_REPUBLISH_INTERVAL_MS) {
        ESP_LOGV(TAG, "Suppressing bonded reconnect IRK (published %" PRIu32 " ms ago)",
                 now - entry.last_published_ms);
        return false;
      }

      ESP_LOGI(TAG, "Re-publishing IRK after bonded reconnect (%u/5)", entry.reconnect_count);
      entry.last_published_ms = now;
      restore_capture_history_(entry);
      return true;
    }
  }

  // New IRK - add to cache with FIFO eviction
  // Cache cap derived from max(max_captures_, 10) to prevent unbounded memory
  // growth
  size_t cache_limit = (max_captures_ > 10) ? max_captures_ : 10;
  if (irk_cache_.size() >= cache_limit) {
    // Evict oldest (FIFO). Note: documented behavior; intentional to cap memory
    // and keep UX predictable.
    ESP_LOGD(TAG, "IRK cache full (%zu entries), evicting oldest entry", cache_limit);
    irk_cache_.erase(irk_cache_.begin());
  }
  irk_cache_.push_back(
      { irk_hex, addr, addr_type, now, connection_generation, 0, false, next_capture_label_ });
  if (!next_capture_label_.empty()) {
    ESP_LOGI(TAG, "Tagged new capture with label '%s'", next_capture_label_.c_str());
    next_capture_label_.clear();  // Consume-once: next device starts unlabeled
  }
  out_is_new_device = true;
  ESP_LOGD(TAG, "New IRK added to cache (total: %zu/%zu)", irk_cache_.size(), cache_limit);
  return true;
}

void IRKCaptureComponent::restore_capture_history_(IRKCacheEntry& entry) {
  // A new publication can restore a forgotten history row, but duplicate
  // extraction paths from the same connection must leave it hidden.
  if (entry.in_history) return;
  entry.in_history = true;
  entry.label = next_capture_label_;
  next_capture_label_.clear();
}

//======================== Output helpers (centralized) ========================

/**
 * @brief Centralized IRK output helper - logs and publishes IRK to Home
 * Assistant sensors
 * @param self Pointer to IRKCaptureComponent instance (may be null during
 * cleanup)
 * @param peer_id_addr Peer's identity address (stable across reconnections)
 * @param irk_hex IRK in hex string format (already reversed for output
 * compatibility)
 * @param context_tag Context label for logging (e.g., "ENC_IMMEDIATE",
 * "DISC_IMMEDIATE", "POLL_CONNECTED")
 * @param connection_generation Monotonic connection identifier
 *
 * This is the single point of IRK output, ensuring consistent formatting across
 * all capture paths. Called from multiple locations: ENC_CHANGE, DISCONNECT,
 * post-disconnect timer, polling loop.
 */
void publish_and_log_irk(IRKCaptureComponent* self, const ble_addr_t& peer_id_addr,
                         const std::string& irk_hex, const char* context_tag,
                         uint32_t connection_generation) {
  if (!self) return;  // Early return if no component instance

  const std::string addr_str = addr_to_str(peer_id_addr);

  // THREAD-SAFE: Check deduplication AND increment counter under mutex
  // CRITICAL: should_publish_irk() assumes caller holds mutex (non-recursive
  // mutex!) All irk_cache_ operations, counter increments, and advertising
  // checks happen atomically
  uint32_t current_events = 0;
  uint32_t current_unique = 0;
  bool max_reached = false;
  bool stop_after_capture_hit = false;
  bool should_publish;
  bool should_stop_adv = false;
  bool is_new_device = false;
  bool limit_just_reached = false;
  bool force_pairing_publish = false;
  bool is_repair = false;
  bool capture_event = false;
  {
    MutexGuard lock(self->state_mutex_);

    force_pairing_publish =
        connection_generation != 0 && self->pairing_generation_ == connection_generation;
    is_repair = connection_generation != 0 && self->repair_generation_ == connection_generation;

    // A valid observation makes delayed fallback reads for this connection
    // redundant. Cancel them before they can create log or counter noise.
    for (auto& timer : self->post_disc_timers_) {
      if (timer.due_ms != 0 && timer.connection_generation == connection_generation &&
          addr_equal(timer.peer_id, peer_id_addr)) {
        timer = {};
      }
    }
    for (auto& timer : self->late_enc_timers_) {
      if (timer.due_ms != 0 && timer.connection_generation == connection_generation &&
          addr_equal(timer.peer_id, peer_id_addr)) {
        timer = {};
      }
    }

    should_publish = self->should_publish_irk(irk_hex, addr_str, peer_id_addr.type,
                                              connection_generation, force_pairing_publish,
                                              should_stop_adv, is_new_device, limit_just_reached);

    if (should_publish) {
      self->capture_events_++;
      if (is_new_device) {
        self->unique_devices_++;
      }
      current_events = self->capture_events_;
      current_unique = self->unique_devices_;

      // max_captures limits unique devices. Re-pairing the same device remains
      // visible without consuming another slot.
      if (!self->continuous_mode_ ||
          (self->max_captures_ > 0 && self->unique_devices_ >= self->max_captures_)) {
        max_reached = true;
      }

      // A genuine capture: a new device, or an explicit (re)pairing. A bonded
      // reconnect republishing a key it already gave us is not one.
      capture_event = is_new_device || force_pairing_publish || is_repair;

      // Wizard "one-shot" mode: independent of continuous_mode/max_captures,
      // stop advertising after a capture so the flow can hand off to
      // confirmation. An earlier device reconnecting in the background must
      // not turn advertising off before the intended device has paired.
      stop_after_capture_hit = self->stop_after_capture_ && capture_event;
    }
  }  // Release mutex before slow logging/publishing operations

  // Handle auto-stop advertising due to reconnect-loop defense.
  // Set suppress flag BEFORE checking is_advertising(): when called from
  // handle_gap_disconnect(), advertising_ is already false (cleared on connect
  // and on disconnect), so is_advertising() returns false. But the disconnect
  // handler's restart logic runs AFTER this call and checks suppress_next_adv_
  // to decide whether to restart. Without setting the flag unconditionally,
  // the restart logic in handle_gap_disconnect() would call start_advertising()
  // in continuous+unlimited mode, defeating the reconnect-loop defense.
  if (should_stop_adv) {
    {
      MutexGuard lock(self->state_mutex_);
      self->suppress_next_adv_ = true;
    }
    // BUG5 FIX: Guard stop_advertising() with is_advertising() to prevent
    // double stop (spurious HA state updates). Only needed when advertising is
    // actually running (e.g., called from timer paths like DISC_DELAYED).
    if (self->is_advertising()) {
      self->stop_advertising();
    }
    if (limit_just_reached) {
      ESP_LOGI(TAG,
               "Reconnect limit reached; suppressing advertising to break the reconnect loop.");
    }
  }

  // Called even for coalesced or rate-limited observations: the sensor decides
  // whether this may replace what is displayed (see publish_irk_to_sensors()).
  self->publish_irk_to_sensors(irk_hex, addr_str.c_str(), connection_generation, capture_event);

  // Skip duplicate capture logs (deduplication happened under mutex).
  if (!should_publish) {
    return;
  }

  log_spacer();
  log_banner(is_repair ? "REPAIR" : context_tag);
  ESP_LOGI(TAG, "Identity Address: %s", addr_str.c_str());
  ESP_LOGI(TAG, "IRK: %s", irk_hex.c_str());
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
  if (max_reached || stop_after_capture_hit) {
    self->set_advertising_requested(false);
  }
}

//======================== GATT DB ========================
// WARNING: These static GATT arrays and g_irk_instance assume a single
// IRKCaptureComponent instance per process. Multiple instances would share
// these statics and overwrite each other's .arg pointers, causing UB.

static uint16_t g_hr_handle;
static uint16_t g_prot_handle;

static struct ble_gatt_chr_def hr_chrs[] = {
  {
      .uuid = &UUID_CHR_HR_MEAS.u,
      .access_cb = chr_read_hr,
      .arg = nullptr,
      // READ grants read permission; READ_ENC adds
      // the encryption requirement. READ_ENC alone
      // maps to no ATT read permission at all, so
      // reads would return READ_NOT_PERMITTED.
      .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
      .val_handle = &g_hr_handle,
  },
  { 0 }
};

static struct ble_gatt_chr_def devinfo_chrs[] = {
  {
      .uuid = &UUID_CHR_MANUF.u,
      .access_cb = chr_read_devinfo,
      .arg = nullptr,  // Will be set to 'this' in register_gatt_services()
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
  },
  {
      .uuid = &UUID_CHR_MODEL.u,
      .access_cb = chr_read_devinfo,
      .arg = nullptr,  // Will be set to 'this' in register_gatt_services()
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
  },
  { 0 }
};

static struct ble_gatt_chr_def batt_chrs[] = { {
                                                   .uuid = &UUID_CHR_BATT_LVL.u,
                                                   .access_cb = chr_read_batt,
                                                   .arg = nullptr,
                                                   .flags =
                                                       BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                                               },
                                               { 0 } };

// Minimal HID characteristic so HID service is valid on NimBLE builds that
// reject empty-characteristic services.
static struct ble_gatt_chr_def hid_chrs[] = { {
                                                  .uuid = &UUID_CHR_HID_PROTO.u,
                                                  .access_cb = chr_read_hid_protocol,
                                                  .arg = nullptr,
                                                  .flags = BLE_GATT_CHR_F_READ,
                                              },
                                              { 0 } };

static struct ble_gatt_chr_def prot_chrs[] = { {
                                                   .uuid = &UUID_CHR_PROT.u,
                                                   .access_cb = chr_read_protected,
                                                   .arg = (void*) "Protected Info",
                                                   .flags = BLE_GATT_CHR_F_READ |
                                                            BLE_GATT_CHR_F_READ_ENC,
                                                   .val_handle = &g_prot_handle,
                                               },
                                               { 0 } };

// Heart Sensor profile GATT services
static struct ble_gatt_svc_def gatt_svcs_heart_sensor[] = {
  {
      // Heart Rate service
      .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &UUID_SVC_HR.u,
      .characteristics = hr_chrs,
  },
  {
      // Device Information service
      .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &UUID_SVC_DEVINFO.u,
      .characteristics = devinfo_chrs,
  },
  {
      // Battery service
      .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &UUID_SVC_BAS.u,
      .characteristics = batt_chrs,
  },
  {
      // Encrypted-read test service; pairing itself is initiated on connect.
      .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &UUID_SVC_PROT.u,
      .characteristics = prot_chrs,
  },
  { 0 }
};

// Keyboard profile GATT services
static struct ble_gatt_svc_def gatt_svcs_keyboard[] = {
  {
      // HID service
      .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &UUID_SVC_HID_BLE.u,
      .characteristics = hid_chrs,
  },
  {
      // Device Information service
      .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &UUID_SVC_DEVINFO.u,
      .characteristics = devinfo_chrs,
  },
  {
      // Battery service
      .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &UUID_SVC_BAS.u,
      .characteristics = batt_chrs,
  },
  {
      // Encrypted-read test service; pairing itself is initiated on connect.
      .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &UUID_SVC_PROT.u,
      .characteristics = prot_chrs,
  },
  { 0 }
};

//======================== Access callbacks ========================

int chr_read_devinfo(uint16_t conn_handle, uint16_t, struct ble_gatt_access_ctxt* ctxt, void* arg) {
  if (!is_encrypted(conn_handle)) return BLE_ATT_ERR_INSUFFICIENT_ENC;

  // THREAD-SAFE: arg points to IRKCaptureComponent instance for both
  // characteristics We determine which characteristic by UUID comparison
  auto* self = static_cast<IRKCaptureComponent*>(arg);

  // Determine which characteristic is being read
  const ble_uuid_t* char_uuid = ctxt->chr->uuid;
  bool is_manufacturer = ble_uuid_cmp(char_uuid, &UUID_CHR_MANUF.u) == 0;

  std::string value_copy;
  {
    MutexGuard lock(self->state_mutex_);
    if (is_manufacturer) {
      value_copy = self->manufacturer_name_;
    } else {
      // Model Number: reflect the effective advertised identity so GATT stays
      // consistent with the advertised name (Keyboard profile poses as a
      // "Logitech K380", so ble_name_ would otherwise leak the real device name).
      value_copy = (self->ble_profile_ == BLEProfile::KEYBOARD) ? "Logitech K380" : self->ble_name_;
    }
  }

  ESP_LOGD(TAG, "DevInfo read (%s): value='%s'", is_manufacturer ? "Manufacturer" : "Model",
           value_copy.c_str());
  append_const_string_or_default(ctxt->om, value_copy.c_str(),
                                 is_manufacturer ? "ESPresense" : "IRK Capture");
  return 0;
}
int chr_read_batt(uint16_t, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*) {
  uint8_t lvl = 100;  // Placeholder static battery level
  os_mbuf_append(ctxt->om, &lvl, 1);
  return 0;
}
int chr_read_hr(uint16_t conn_handle, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*) {
  if (!is_encrypted(conn_handle)) return BLE_ATT_ERR_INSUFFICIENT_ENC;
  uint8_t buf[2];
  size_t len;
  hr_measurement_sample(buf, &len);
  os_mbuf_append(ctxt->om, buf, len);
  return 0;
}
int chr_read_hid_protocol(uint16_t, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*) {
  uint8_t protocol_mode = 0x01;  // Report protocol mode
  os_mbuf_append(ctxt->om, &protocol_mode, 1);
  return 0;
}
int chr_read_protected(uint16_t conn_handle, uint16_t, struct ble_gatt_access_ctxt* ctxt,
                       void* arg) {
  if (!is_encrypted(conn_handle)) return BLE_ATT_ERR_INSUFFICIENT_ENC;
  append_const_string_or_default(ctxt->om, (const char*) arg, "Protected Info");
  return 0;
}

//======================== BLE name sanitization ========================

std::string IRKCaptureComponent::sanitize_ble_name(const std::string& name) {
  // Validate and sanitize BLE name for runtime changes from Home Assistant
  std::string sanitized;
  sanitized.reserve(12);  // 12 chars for Samsung S24/S25 compatibility with
                          // single-UUID advertising

  // Check for empty string
  if (name.empty()) {
    ESP_LOGW(TAG, "BLE name cannot be empty, using default 'IRK Capture'");
    return "IRK Capture";
  }

  // Sanitize: allow only safe characters (alphanumeric, space, hyphen,
  // underscore)
  for (char c : name) {
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == ' ' ||
        c == '-' || c == '_') {
      sanitized += c;
    } else {
      // Log rejected character
      ESP_LOGW(TAG, "Rejected invalid character in BLE name: 0x%02X ('%c')", (uint8_t) c,
               (c >= 32 && c <= 126) ? c : '?');
    }

    // Enforce 12-byte limit for Samsung S24/S25 compatibility (clean profile
    // with single UUID)
    if (sanitized.length() >= 12) {
      ESP_LOGW(TAG, "BLE name truncated to 12 bytes (Samsung compatibility)");
      break;
    }
  }

  // Trim leading and trailing whitespace
  size_t start = sanitized.find_first_not_of(' ');
  size_t end = sanitized.find_last_not_of(' ');
  if (start != std::string::npos) {
    sanitized = sanitized.substr(start, end - start + 1);
  } else {
    sanitized.clear();
  }

  // Final validation: ensure we have at least one character
  if (sanitized.empty()) {
    ESP_LOGW(TAG,
             "BLE name contained only invalid characters, using default 'IRK "
             "Capture'");
    return "IRK Capture";
  }

  // Log if sanitization changed the input
  if (sanitized != name) {
    ESP_LOGI(TAG, "BLE name sanitized: '%s' -> '%s'", name.c_str(), sanitized.c_str());
  }

  return sanitized;
}

//======================== Entity impls ========================

void IRKCaptureText::control(const std::string& value) {
  if (parent_->get_ble_profile() == BLEProfile::KEYBOARD) {
    ESP_LOGW(TAG,
             "BLE Device Name is fixed to 'Logitech K380' in Keyboard profile; "
             "switch to Heart Sensor to customize it");
    publish_state("Logitech K380");
    return;
  }

  // Sanitize and validate user input from Home Assistant
  std::string sanitized = parent_->sanitize_ble_name(value);

  // Publish only after the device accepts it; otherwise Home Assistant would
  // show a name the device never adopted.
  if (!parent_->update_ble_name(sanitized)) {
    const std::string current = parent_->get_ble_name();
    ESP_LOGW(TAG, "BLE name update rejected; keeping '%s'", current.c_str());
    publish_state(current);
    return;
  }

  ESP_LOGI(TAG, "BLE name changed to: %s", sanitized.c_str());
  publish_state(sanitized);
}

void IRKCaptureSwitch::write_state(bool state) {
  parent_->set_advertising_requested(state);
}

void IRKCaptureButton::press_action() {
  ESP_LOGI(TAG, "Refreshing MAC address...");
  parent_->refresh_mac();
}

void IRKCaptureForgetBondsButton::press_action() {
  ESP_LOGI(TAG, "Forget All Bonds pressed");
  parent_->forget_all_bonds();
}

void IRKCaptureStopAfterCaptureSwitch::write_state(bool state) {
  // set_stop_after_capture() stages the entity publish; publishing here as
  // well would send every change twice.
  parent_->set_stop_after_capture(state);
}

void IRKCaptureLabelText::control(const std::string& value) {
  // Stages the publish of the accepted (sanitized) value; see above.
  parent_->set_next_capture_label(value);
}

//======================== Entity dump_config ========================

void IRKCaptureText::dump_config() {
  ESP_LOGCONFIG(TAG, "IRK Capture BLE Name Text:");
  ESP_LOGCONFIG(TAG, "  Value: '%s'", state.c_str());
}

void IRKCaptureSwitch::dump_config() {
  ESP_LOGCONFIG(TAG, "IRK Capture Advertising Switch:");
  ESP_LOGCONFIG(TAG, "  State: %s", state ? "ON" : "OFF");
}

void IRKCaptureButton::dump_config() {
  ESP_LOGCONFIG(TAG, "IRK Capture New MAC Button");
}

void IRKCaptureForgetBondsButton::dump_config() {
  ESP_LOGCONFIG(TAG, "IRK Capture Forget Bonds Button");
}

void IRKCaptureStopAfterCaptureSwitch::dump_config() {
  ESP_LOGCONFIG(TAG, "IRK Capture Stop-After-Capture Switch:");
  ESP_LOGCONFIG(TAG, "  State: %s", state ? "ON" : "OFF");
}

void IRKCaptureLabelText::dump_config() {
  ESP_LOGCONFIG(TAG, "IRK Capture Next Capture Label Text:");
  ESP_LOGCONFIG(TAG, "  Value: '%s'", state.c_str());
}

void IRKCaptureSelect::control(const std::string& value) {
  if (value == "Heart Sensor") {
    parent_->set_ble_profile(BLEProfile::HEART_SENSOR);
  } else if (value == "Keyboard") {
    parent_->set_ble_profile(BLEProfile::KEYBOARD);
  } else {
    ESP_LOGW(TAG,
             "Invalid BLE profile value: '%s' (expected 'Heart Sensor' or "
             "'Keyboard')",
             value.c_str());
    return;  // Don't publish invalid state to Home Assistant
  }
  // Persistence failure restores the previous profile. Reflect the accepted
  // state instead of overwriting that rollback with the requested selection.
  publish_state(parent_->get_ble_profile() == BLEProfile::HEART_SENSOR ? "Heart Sensor"
                                                                       : "Keyboard");
}

void IRKCaptureSelect::dump_config() {
  ESP_LOGCONFIG(TAG, "IRK Capture BLE Profile Select");
}

void IRKCaptureTextSensor::dump_config() {
  ESP_LOGCONFIG(TAG, "IRK Capture Text Sensor");
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

/**
 * @brief Handles BLE_GAP_EVENT_DISCONNECT - called when peer disconnects
 * @param self Pointer to IRKCaptureComponent instance
 * @param ev GAP event structure containing disconnect reason
 * @return 0 (always)
 *
 * Attempts immediate IRK read from NVS, schedules delayed read (+800ms),
 * and restarts advertising unless suppressed (IRK re-publish case).
 */
static void log_no_irk_for_peer(const ble_addr_t& peer_id) {
  ESP_LOGW(TAG, "Bond for %s has no usable IRK; key distribution may be incomplete or unsupported",
           addr_to_str(peer_id).c_str());
}

int handle_gap_disconnect(IRKCaptureComponent* self, struct ble_gap_event* ev) {
  ESP_LOGI(TAG, "Disconnect reason=%d (0x%02x)", ev->disconnect.reason, ev->disconnect.reason);

  // Use the connection descriptor embedded in the disconnect event directly
  // (ble_gap_conn_find may fail after disconnect since NimBLE removes the
  // descriptor). The post-disconnect timer's peer is set by
  // schedule_post_disconnect_check() below, so no separate cache is needed here.
  const struct ble_gap_conn_desc& d = ev->disconnect.conn;
  uint32_t connection_generation = 0;
  bool already_observed = false;
  bool pairing_completed = false;
  {
    MutexGuard lock(self->state_mutex_);
    if (self->connected_ && self->conn_handle_ == d.conn_handle) {
      connection_generation = self->connection_generation_;
      pairing_completed = self->enc_ready_;
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
  if (!self->on_disconnect(d.conn_handle)) {
    ESP_LOGW(TAG, "Ignoring disconnect for non-active handle=%u", d.conn_handle);
    return 0;
  }

  // ENC_CHANGE normally captures first. Only use the disconnect store read as
  // a fallback when this connection has not produced a valid IRK yet.
  bool needs_delayed_check = !already_observed;
  if (!already_observed) {
    struct ble_store_value_sec bond {};
    struct ble_store_key_sec key {};
    key.peer_addr = d.peer_id_addr;
    int rc = ble_store_read_peer_sec(&key, &bond);
    if (rc == BLE_HS_ENOENT) {
      ESP_LOGD(TAG, "No bond for peer (ENOENT)");
    } else if (rc != 0) {
      ESP_LOGW(TAG, "ble_store_read_peer_sec rc=%d", rc);
    } else if (bond.irk_present && self->is_valid_irk(bond.irk)) {
      std::string irk_hex = to_hex_rev(bond.irk, sizeof(bond.irk));
      publish_and_log_irk(self, d.peer_id_addr, irk_hex, "DISC_IMMEDIATE", connection_generation);
      needs_delayed_check = false;
    } else {
      log_no_irk_for_peer(d.peer_id_addr);
      // A public identity alone cannot establish whether a device uses privacy.
      // Only a completed pairing with a stored bond lacking an IRK is conclusive.
      if (!bond.irk_present && pairing_completed) {
        self->publish_no_irk_(d.peer_id_addr, connection_generation, pairing_completed);
        needs_delayed_check = false;
      }
    }
  }

  if (needs_delayed_check) {
    self->schedule_post_disconnect_check(d.peer_id_addr, connection_generation, pairing_completed);
  }

  // Thread-safe advertising state update
  {
    MutexGuard lock(self->state_mutex_);
    self->advertising_ = false;
  }

  // Check if we should stop advertising
  // THREAD-SAFE: Snapshot capture counts and suppression flag under mutex
  bool should_stop_adv = false;
  bool suppressed;
  bool requested;
  bool restart_scheduled;
  {
    MutexGuard lock(self->state_mutex_);
    // Stop if:
    // 1. continuous_mode is false AND we've captured at least one IRK, OR
    // 2. continuous_mode is true AND max_captures > 0 AND we've hit the limit
    if (!self->continuous_mode_ && self->capture_events_ > 0) {
      should_stop_adv = true;
    } else if (self->continuous_mode_ && self->max_captures_ > 0 &&
               self->unique_devices_ >= self->max_captures_) {
      should_stop_adv = true;
    }
    suppressed = self->suppress_next_adv_;
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
    ESP_LOGI(TAG, "Advertising restart already scheduled after timeout cooldown");
  } else if (!suppressed) {
    // Normal case: restart advertising (will set flag with mutex internally)
    if (self->continuous_mode_) {
      ESP_LOGI(TAG, "Continuous mode: restarting advertising for next device");
    }
    self->start_advertising();
  } else {
    // Suppression case: delay restart
    ESP_LOGI(TAG, "Advertising suppressed to break reconnect loop; auto-restart in 5s");

    // THREAD-SAFE: Reset suppression flag and set timer atomically
    {
      MutexGuard lock(self->state_mutex_);
      self->suppress_next_adv_ = false;  // Reset for next time
      self->adv_restart_time_ = now_ms() + TimingConfig::ADV_SUPPRESS_RESTART_DELAY_MS;
    }
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
 * On success (status=0): Attempts immediate IRK read and schedules delayed read
 * (+5s). On failure: Logs detailed error, deletes stale bond, and retries
 * security once.
 */
int handle_gap_enc_change(IRKCaptureComponent* self, struct ble_gap_event* ev) {
  ESP_LOGI(TAG, "ENC_CHANGE status=%d (0x%02X)", ev->enc_change.status, ev->enc_change.status);
  uint32_t connection_generation = 0;
  {
    MutexGuard lock(self->state_mutex_);
    if (self->connected_ && self->conn_handle_ == ev->enc_change.conn_handle) {
      connection_generation = self->connection_generation_;
    }
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
      self->enc_ready_ = true;
      self->enc_time_ = now_ms();
      // BUG1 FIX: Initialize irk_last_try_ms_ to now so the ENC_TRY_INTERVAL_MS
      // guard in poll_irk_if_due() is correctly enforced on the first poll
      // attempt. Without this, (now - 0) is huge and the interval check is
      // bypassed immediately.
      self->irk_last_try_ms_ = now_ms();
    }

    // Immediate store read using identity address
    struct ble_gap_conn_desc d {};
    const int desc_rc = ble_gap_conn_find(ev->enc_change.conn_handle, &d);
    if (desc_rc == 0) {
      struct ble_store_key_sec key {};
      key.peer_addr = d.peer_id_addr;
      struct ble_store_value_sec bond {};
      int rc = ble_store_read_peer_sec(&key, &bond);
      if (rc == BLE_HS_ENOENT) {
        ESP_LOGD(TAG, "No bond for peer yet (ENOENT); scheduling late check");
        self->schedule_late_enc_check(d.peer_id_addr, connection_generation);
      } else if (rc != 0) {
        ESP_LOGW(TAG, "ble_store_read_peer_sec rc=%d; scheduling late check", rc);
        self->schedule_late_enc_check(d.peer_id_addr, connection_generation);
      } else if (bond.irk_present && self->is_valid_irk(bond.irk)) {
        std::string irk_hex = to_hex_rev(bond.irk, sizeof(bond.irk));
        publish_and_log_irk(self, d.peer_id_addr, irk_hex, "ENC_CHANGE", connection_generation);
        // Tested working behavior: terminate immediately after successful ENC +
        // IRK capture
        int term_rc;
        {
          BleOpGuard ble_lock(self->ble_op_mutex_);
          term_rc = ble_gap_terminate(ev->enc_change.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        if (term_rc != 0) {
          ESP_LOGW(TAG, "ble_gap_terminate after ENC capture rc=%d", term_rc);
        }
      } else if (!bond.irk_present) {
        // NimBLE has completed key distribution before reporting ENC_CHANGE
        // success. Preserve this outcome before timeout cleanup can delete it.
        self->publish_no_irk_(d.peer_id_addr, connection_generation, true);
      } else {
        ESP_LOGD(TAG, "Bond present but no IRK yet; scheduling late check");
        self->schedule_late_enc_check(d.peer_id_addr, connection_generation);
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
      ble_store_util_delete_peer(&d.peer_id_addr);
    } else {
      ESP_LOGW(TAG, "ENC_CHANGE failed status=%d; conn desc not found, no bond cleared",
               ev->enc_change.status);
    }
    int term_rc;
    {
      BleOpGuard ble_lock(self->ble_op_mutex_);
      term_rc = ble_gap_terminate(ev->enc_change.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    if (term_rc != 0) {
      ESP_LOGW(TAG, "ble_gap_terminate after ENC failure rc=%d", term_rc);
    }

    // A DHKey check failure means the peer's key material is stale, so briefly
    // stop advertising to prod it into resetting. Match the reason from either
    // side: the encoded status differs (0x40B local, 0x50B peer).
    const SmFailure failure = decode_sm_failure(ev->enc_change.status);
    if (failure.is_sm && failure.reason == BLE_SM_ERR_DHKEY) {
      ESP_LOGW(TAG,
               "DHKey failure detected - suppressing advertising to force peer "
               "reset");
      MutexGuard lock(self->state_mutex_);
      self->suppress_next_adv_ = true;
    }
  }
  return 0;
}

/**
 * @brief Handles BLE_GAP_EVENT_REPEAT_PAIRING - peer attempting to pair when
 * already bonded
 * @param self Pointer to IRKCaptureComponent instance (unused, but kept for
 * consistency)
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
    ESP_LOGW(TAG,
             "Repeat pairing from %02X:%02X:%02X:%02X:%02X:%02X (clearing "
             "stale peer bond)",
             peer->val[5], peer->val[4], peer->val[3], peer->val[2], peer->val[1], peer->val[0]);
    // NimBLE requires the application to delete the old bond before returning
    // RETRY. Returning RETRY after a failed delete would just re-enter the same
    // repeat-pairing check and can strand the peer in a timeout loop.
    int delete_rc = ble_store_util_delete_peer(peer);
    if (delete_rc == 0 || delete_rc == BLE_HS_ENOENT) {
      // Remember that this connection represents an explicit re-pair. Its IRK
      // must be published even when unchanged and inside the normal rate limit.
      MutexGuard lock(self->state_mutex_);
      if (self->connected_ && self->conn_handle_ == ev->repeat_pairing.conn_handle) {
        self->pairing_generation_ = self->connection_generation_;
        self->repair_generation_ = self->connection_generation_;
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

      // Log passkey if we're supposed to display it (shouldn't happen with
      // NO_INPUT_OUTPUT)
      if (ev->passkey.params.action == BLE_SM_IOACT_DISP) {
        ESP_LOGW(TAG, "UNEXPECTED: Peer requested passkey display (passkey=%06lu)",
                 (unsigned long) ev->passkey.params.numcmp);
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
               "Subscription changed: handle=%u attr=%u notify=%u indicate=%u "
               "reason=%u",
               ev->subscribe.conn_handle, ev->subscribe.attr_handle, ev->subscribe.cur_notify,
               ev->subscribe.cur_indicate, ev->subscribe.reason);
      return 0;
#endif

#ifdef BLE_GAP_EVENT_IDENTITY_RESOLVED
    case BLE_GAP_EVENT_IDENTITY_RESOLVED:
      // Identity resolved successfully (IRK working!)
      ESP_LOGD(TAG, "Peer identity resolved using IRK");
      return 0;
#endif

#ifdef BLE_GAP_EVENT_PARING_COMPLETE
    case BLE_GAP_EVENT_PARING_COMPLETE:
      // NimBLE intentionally emits this before persisting keys and before the
      // ENC_CHANGE callback. Keep it diagnostic-only so ENC_CHANGE remains the
      // single owner of IRK capture, failure cleanup, and termination.
      if (ev->pairing_complete.status == 0) {
        ESP_LOGD(TAG, "Pairing complete: handle=%u", ev->pairing_complete.conn_handle);
      } else {
        ESP_LOGW(TAG, "Pairing failed: handle=%u status=%d (0x%X)",
                 ev->pairing_complete.conn_handle, ev->pairing_complete.status,
                 ev->pairing_complete.status);
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
      // final. A successful event is routine; a failure is followed by the
      // normal disconnect path, which owns connection-state cleanup.
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

    default:
      // More verbose default logging to aid future SDK changes
      ESP_LOGD(TAG, "Unhandled GAP event type=%d", ev->type);
      return 0;
  }
}

//======================== Component lifecycle ========================

void IRKCaptureComponent::setup() {
  ESP_LOGI(TAG, "IRK Capture v%s ready", VERSION);

  // Sanitize initial name from YAML
  ble_name_ = sanitize_ble_name(ble_name_);

  // Initialize mutex for thread-safe access to shared state
  state_mutex_ = xSemaphoreCreateMutex();
  if (!state_mutex_) {
    ESP_LOGE(TAG, "CRITICAL: Failed to create state mutex - thread safety compromised!");
    this->mark_failed(LOG_STR("state mutex allocation failed"));
    return;
  }
  ble_op_mutex_ = xSemaphoreCreateMutex();
  if (!ble_op_mutex_) {
    ESP_LOGE(TAG,
             "CRITICAL: Failed to create BLE op mutex - cannot serialize BLE "
             "host calls");
    this->mark_failed(LOG_STR("BLE operation mutex allocation failed"));
    return;
  }

  // Load persisted BLE profile from NVS (before BLE stack init so GATT is
  // correct)
  nvs_handle_t nvs_handle;
  if (nvs_open("irk_capture", NVS_READONLY, &nvs_handle) == ESP_OK) {
    uint8_t profile_val = 0;
    if (nvs_get_u8(nvs_handle, "ble_profile", &profile_val) == ESP_OK) {
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
  // Derive cache capacity from max_captures_ (floor of 10 to handle
  // deduplication of reconnections)
  size_t cache_cap = (max_captures_ > 10) ? max_captures_ : 10;
  irk_cache_.reserve(cache_cap);
  capture_events_ = 0;
  unique_devices_ = 0;
  connection_generation_ = 0;
  pairing_generation_ = 0;
  repair_generation_ = 0;
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

  this->setup_ble();
  if (this->is_failed()) {
    return;
  }

  ble_npl_event_init(
      &bond_clear_event_,
      [](struct ble_npl_event* event) {
        auto* self = static_cast<IRKCaptureComponent*>(ble_npl_event_get_arg(event));
        self->handle_forget_bonds_();
      },
      this);

  // Older NPL ports wait indefinitely for space in the host event queue. Keep
  // that wait off both the ESPHome main task and the shared timer task. A
  // notification bit coalesces retries while this single producer is blocked.
  if (xTaskCreate(
          [](void* arg) {
            auto* self = static_cast<IRKCaptureComponent*>(arg);
            for (;;) {
              uint32_t notification;
              xTaskNotifyWait(0, UINT32_MAX, &notification, portMAX_DELAY);
              self->queue_bond_clear_();
            }
          },
          "irk_bond_clear", 2048, this, tskIDLE_PRIORITY + 1, &bond_clear_task_) != pdPASS) {
    bond_clear_task_ = nullptr;
    ESP_LOGE(TAG, "Bond-clear worker allocation failed; stored-bond clearing is unavailable");
  }

  // on_ble_host_synced() applies the resolved intent when NimBLE becomes ready.

  if (ble_name_text_) {
    // Update name based on profile
    if (ble_profile_ == BLEProfile::KEYBOARD) {
      ble_name_text_->publish_state("Logitech K380");
    } else {
      ble_name_text_->publish_state(ble_name_);
    }
  }
  if (ble_profile_select_) {
    // Initialize select to persisted profile
    ble_profile_select_->publish_state(ble_profile_ == BLEProfile::KEYBOARD ? "Keyboard"
                                                                            : "Heart Sensor");
  }
  if (advertising_switch_) {
    advertising_switch_->publish_state(is_advertising_requested());
  }
  if (stop_after_capture_switch_) {
    // Apply the switch's restore mode, the same way the advertising switch
    // does. Without this the declared RESTORE_DEFAULT_OFF never takes effect
    // and the setting silently reverts on every boot.
    auto restored = stop_after_capture_switch_->get_initial_state_with_restore_mode();
    if (restored.has_value()) {
      stop_after_capture_ = restored.value() != stop_after_capture_switch_->is_inverted();
    }
    stop_after_capture_switch_->publish_state(stop_after_capture_);
  }
  if (status_sensor_) {
    last_status_value_ = "idle";
    status_sensor_->publish_state(last_status_value_);
  }
}

void IRKCaptureComponent::dump_config() {
  // THREAD-SAFE: Copy state before logging
  std::string name_copy;
  bool adv_state;
  BLEProfile current_profile;
  {
    MutexGuard lock(state_mutex_);
    name_copy = ble_name_;
    adv_state = advertising_;
    current_profile = ble_profile_;
  }

  const char* effective_name =
      (current_profile == BLEProfile::KEYBOARD) ? "Logitech K380" : name_copy.c_str();
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

  // Drain entity publishes staged by the NimBLE task (see flush impl).
  flush_pending_publishes_();

  // Wizard-facing session status (advertising/pairing/capturing/captured/idle).
  update_status_sensor_(now);

  // Some NPL ports drop full-queue events; others block the producer. Retry
  // through the worker so neither behavior can stall the ESPHome main task.
  notify_bond_clear_();

  // Timers for IRK checks
  handle_post_disconnect_timer(now);
  handle_late_enc_timer(now);

  // Rotation must not starve security timeouts or IRK polling on a connection
  // that arrived while the radio was stopping.
  handle_mac_rotation_(now);
  uint16_t conn_handle_copy;

  // Auto-restart advertising if suppressed and timer expired
  bool should_restart_adv = false;
  bool should_stop_unwanted_adv = false;
  {
    MutexGuard lock(state_mutex_);
    should_stop_unwanted_adv = advertising_ && !advertising_requested_;
    if (advertising_requested_ && !advertising_ && !connected_ &&
        mac_rotation_state_ == MacRotationState::IDLE && adv_restart_time_ != 0 &&
        deadline_reached(now, adv_restart_time_)) {
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
  // Security initiation can dispatch a new connection while this loop is in
  // progress. Sample time again so its start cannot appear newer than "now".
  if (handle_connection_timeout_(now_ms())) return;

  // THREAD-SAFE: Check connection state before proceeding with
  // connection-specific work
  bool is_connected;
  {
    MutexGuard lock(state_mutex_);
    is_connected = connected_;
    conn_handle_copy = conn_handle_;
  }
  if (!is_connected || conn_handle_copy == BLE_HS_CONN_HANDLE_NONE) return;

  // HR notify and IRK polling (post-ENC)
  notify_hr_if_due(now);
  poll_irk_if_due(now);
}

//======================== BLE setup/registry ========================

void IRKCaptureComponent::setup_ble() {
  // NVS for key store
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "NVS full or version mismatch - erasing");
    esp_err_t erase_err = nvs_flash_erase();
    if (erase_err != ESP_OK) {
      ESP_LOGE(TAG, "nvs_flash_erase failed (err=%d)", erase_err);
      this->mark_failed(LOG_STR("NVS erase failed"));
      return;
    }
    err = nvs_flash_init();
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_flash_init failed (err=%d)", err);
    this->mark_failed(LOG_STR("NVS initialization failed"));
    return;
  }

  // NVS health check - verify storage actually works
  // Single open/close with write, verify, and cleanup in one pass
  nvs_handle_t nvs_test_handle;
  err = nvs_open("irk_test", NVS_READWRITE, &nvs_test_handle);
  if (err == ESP_OK) {
    esp_err_t werr = nvs_set_u32(nvs_test_handle, "test", 0xDEADBEEF);
    if (werr == ESP_OK) {
      esp_err_t cerr = nvs_commit(nvs_test_handle);
      if (cerr == ESP_OK) {
        ESP_LOGI(TAG, "NVS health check passed");
      } else {
        ESP_LOGW(TAG, "NVS health check commit failed (err=%d) - continuing", cerr);
      }
    } else {
      ESP_LOGW(TAG, "NVS health check write failed (err=%d) - continuing", werr);
    }
    // Clean up test data in same handle (best-effort)
    esp_err_t erase_test_err = nvs_erase_all(nvs_test_handle);
    if (erase_test_err != ESP_OK) {
      ESP_LOGW(TAG, "NVS cleanup erase failed (err=%d)", erase_test_err);
    }
    esp_err_t cleanup_commit_err = nvs_commit(nvs_test_handle);
    if (cleanup_commit_err != ESP_OK) {
      ESP_LOGW(TAG, "NVS cleanup commit failed (err=%d)", cleanup_commit_err);
    }
    nvs_close(nvs_test_handle);
  } else {
    ESP_LOGW(TAG, "NVS health check open failed (err=%d) - continuing", err);
  }

  // NimBLE host. Without a working host and bond store the component cannot
  // perform its only job, so initialization failures are fatal rather than a
  // degraded mode that still advertises.
  int rc = nimble_port_init();
  if (rc != 0) {
    ESP_LOGE(TAG, "nimble_port_init failed rc=%d", rc);
    this->mark_failed(LOG_STR("NimBLE host initialization failed"));
    return;
  }

  // Set global instance for callbacks that don't accept user args
  g_irk_instance = this;

  // Security (set once here; no later re-asserts)
  ble_hs_cfg.reset_cb = [](int reason) {
    ESP_LOGW(TAG, "NimBLE reset reason=%d", reason);
    if (!g_irk_instance) return;

    g_irk_instance->host_synced_ = false;
    MutexGuard lock(g_irk_instance->state_mutex_);
    g_irk_instance->host_generation_++;
    g_irk_instance->random_address_ready_ = false;
    g_irk_instance->advertising_start_attempts_ = 0;
    g_irk_instance->advertising_failure_log_time_ = 0;
    g_irk_instance->advertising_ = false;
    g_irk_instance->reset_connection_state_();
    g_irk_instance->suppress_next_adv_ = false;
    g_irk_instance->adv_restart_time_ = 0;
    // Cancel the request without relying on the old host event being serviced.
    // Leave its event object intact: a delayed wake-up checks current state.
    g_irk_instance->bond_clear_pending_ = false;
    g_irk_instance->bond_clear_host_generation_ = 0;
    // A host reset proves the old controller state is gone. Abort any
    // in-flight MAC rotation instead of leaving REQUESTED waiting for a
    // disconnect callback that can no longer arrive.
    g_irk_instance->mac_rotation_generation_++;
    g_irk_instance->mac_rotation_state_ = MacRotationState::IDLE;
    g_irk_instance->mac_rotation_retries_ = 0;
    g_irk_instance->mac_rotation_ready_time_ = 0;
  };
  ble_hs_cfg.sync_cb = []() {
    ESP_LOGI(TAG, "NimBLE host synced");
    // Print SM config once early to reduce chances of log drops later
    ESP_LOGI(TAG,
             "SM config (on sync): bonding=%d mitm=%d sc=%d io_cap=%d "
             "our_key_dist=0x%02X "
             "their_key_dist=0x%02X",
             (int) ble_hs_cfg.sm_bonding, (int) ble_hs_cfg.sm_mitm, (int) ble_hs_cfg.sm_sc,
             (int) ble_hs_cfg.sm_io_cap, (unsigned) ble_hs_cfg.sm_our_key_dist,
             (unsigned) ble_hs_cfg.sm_their_key_dist);

    // Now that host is synced, set flag and start advertising if configured
    if (g_irk_instance) {
      g_irk_instance->on_ble_host_synced();
    }
  };

  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_mitm = 0;
  ble_hs_cfg.sm_sc = 1;                              // Secure Connections enabled
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;  // Just Works pairing (no PIN)
  ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  log_sm_config();

  // Key-value store for bonding/keys
  // This API returns void. The store callbacks it installs report later
  // operation failures through their own return values.
  ble_store_config_init();

  // ble_store_config_init() installs only read/write/delete callbacks, leaving
  // store_status_cb null. NimBLE caps bonds at CONFIG_BT_NIMBLE_MAX_BONDS (3 by
  // default) — far below max_captures — and when that is reached it asks the
  // application to make room. With no callback the request returns ENOTSUP and
  // the Security Manager rejects the next fresh pairing outright, so a fourth
  // device in one session could never pair. Round-robin eviction is the right
  // policy here: a captured IRK is already published and held in irk_cache_,
  // so the bond itself is disposable once the capture completes.
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  // Clear all bonds on boot for a "clean slate" - prevents bond table from
  // filling up and ensures privacy (no old IRKs persist across reboots or
  // device ownership changes)
  rc = ble_store_clear();
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_store_clear on boot failed rc=%d", rc);
    this->mark_failed(LOG_STR("BLE bond store clear failed"));
    return;
  }
  ESP_LOGI(TAG, "Bond table cleared on boot - fresh pairing session ready");

  // GAP/GATT and name
  ble_svc_gap_init();
  ble_svc_gatt_init();
  rc = ble_svc_gap_device_name_set(ble_name_.c_str());
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_svc_gap_device_name_set failed rc=%d", rc);
    this->mark_failed(LOG_STR("BLE GAP name initialization failed"));
    return;
  }

  // Register services
  if (!this->register_gatt_services()) {
    this->mark_failed(LOG_STR("BLE GATT service registration failed"));
    return;
  }

  // Host task
  nimble_port_freertos_init([](void*) {
    nimble_port_run();
    nimble_port_freertos_deinit();
    vTaskDelete(NULL);
  });
}

void IRKCaptureComponent::on_ble_host_synced() {
  {
    MutexGuard lock(state_mutex_);
    host_generation_++;
    random_address_ready_ = false;
    advertising_start_attempts_ = 0;
    advertising_failure_log_time_ = 0;
    host_synced_ = true;
  }
  // start_advertising() configures the random address first and retries that
  // step too if it fails. A failed address setup must not leave us endlessly
  // retrying advertising with no usable random address.
  if (is_advertising_requested()) {
    ESP_LOGI(TAG, "Host synced - starting advertising (requested=true)");
    start_advertising();
  }
}

bool IRKCaptureComponent::register_gatt_services() {
  // THREAD-SAFE: Pass 'this' pointer to BOTH DevInfo characteristics
  // Callback will determine which field to read based on UUID
  devinfo_chrs[0].arg = (void*) this;  // Manufacturer Name
  devinfo_chrs[1].arg = (void*) this;  // Model Number

  // BUG4 FIX: Explicitly zero the handle globals before any registration
  // attempt. The Keyboard profile GATT table has no HR service (it does include
  // the Protected service), so g_hr_handle would retain a stale value from a
  // prior boot if a Keyboard→Heart Sensor fallback occurs. Zeroing both here
  // makes the state explicit and predictable regardless of which registration
  // path is taken.
  g_hr_handle = 0;
  g_prot_handle = 0;

  // Select GATT services based on current profile
  BLEProfile current_profile;
  {
    MutexGuard lock(state_mutex_);
    current_profile = ble_profile_;
  }

  auto register_svcs = [](struct ble_gatt_svc_def* svcs, const char* profile_name) -> int {
    ESP_LOGI(TAG, "Registering GATT services for %s profile", profile_name);
    int rc = ble_gatts_count_cfg(svcs);
    if (rc == 0) {
      rc = ble_gatts_add_svcs(svcs);
    }
    if (rc != 0) {
      ESP_LOGE(TAG, "GATT registration for %s profile failed rc=%d", profile_name, rc);
    }
    return rc;
  };

  int rc;
  if (current_profile == BLEProfile::KEYBOARD) {
    rc = register_svcs(gatt_svcs_keyboard, "Keyboard");
    if (rc != 0) {
      ESP_LOGW(TAG,
               "Keyboard GATT registration failed; falling back to Heart "
               "Sensor profile "
               "to keep component operational");
      rc = register_svcs(gatt_svcs_heart_sensor, "Heart Sensor");
      if (rc == 0) {
        ESP_LOGW(TAG, "Keyboard->Heart Sensor fallback succeeded");
        // Persist fallback so next boot doesn't repeat the same failure path.
        {
          MutexGuard lock(state_mutex_);
          ble_profile_ = BLEProfile::HEART_SENSOR;
        }
        nvs_handle_t nvs_handle;
        if (nvs_open("irk_capture", NVS_READWRITE, &nvs_handle) == ESP_OK) {
          // BUG10 FIX: Track set and commit errors separately so the log
          // message accurately reports which operation failed (commit was "not
          // attempted" if the set itself failed).
          esp_err_t set_err =
              nvs_set_u8(nvs_handle, "ble_profile", static_cast<uint8_t>(BLEProfile::HEART_SENSOR));
          esp_err_t commit_err = ESP_OK;
          if (set_err == ESP_OK) {
            commit_err = nvs_commit(nvs_handle);
          }
          nvs_close(nvs_handle);
          if (set_err != ESP_OK || commit_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to persist fallback BLE profile (set=%d commit=%s)", set_err,
                     (set_err != ESP_OK) ? "not attempted" : esp_err_to_name(commit_err));
          }
        } else {
          ESP_LOGW(TAG, "Failed to open NVS for fallback profile persistence");
        }
      }
    }
  } else {
    rc = register_svcs(gatt_svcs_heart_sensor, "Heart Sensor");
  }

  if (rc != 0) {
    ESP_LOGE(TAG,
             "No valid GATT profile could be registered; IRK capture cannot "
             "operate");
    return false;
  }

  // Do NOT snapshot g_hr_handle/g_prot_handle here. ble_gatts_add_svcs() only
  // queues the service definitions; NimBLE assigns characteristic handles later,
  // in ble_gatts_start(), when the host task starts. Any copy taken now is 0
  // forever. Readers use the globals, which NimBLE populates via val_handle.
  return true;
}

//======================== Advertising ========================

void IRKCaptureComponent::start_advertising() {
  if (!host_synced_) {
    ESP_LOGW(TAG, "Host not synced; cannot advertise");
    return;
  }

  // Get current profile and enforce the single-connection invariant.
  BLEProfile current_profile;
  std::string name_copy;
  bool requested;
  bool connected;
  bool rotating;
  uint32_t host_generation;
  {
    MutexGuard lock(state_mutex_);
    current_profile = ble_profile_;
    name_copy = ble_name_;
    requested = advertising_requested_;
    connected = connected_;
    rotating = mac_rotation_state_ != MacRotationState::IDLE || bond_clear_pending_;
    host_generation = host_generation_;
  }
  if (!requested) {
    ESP_LOGD(TAG, "Advertising start skipped: user intent is OFF");
    return;
  }
  if (connected || rotating) {
    ESP_LOGD(TAG, "Advertising start deferred until connection/BLE maintenance completes");
    return;
  }

  static const char* keyboard_name = "Logitech K380";
  struct ble_hs_adv_fields fields;
  memset(&fields, 0, sizeof(fields));

  const char* profile_name;

  // Scan response fields (used for Keyboard profile to fit name in separate
  // packet)
  struct ble_hs_adv_fields rsp_fields;
  memset(&rsp_fields, 0, sizeof(rsp_fields));
  bool use_scan_response = false;

  if (current_profile == BLEProfile::KEYBOARD) {
    // Keyboard profile: Logitech K380
    // Move name to scan response to stay within 31-byte advertising packet
    // limit
    profile_name = "Keyboard";

    // Advertising data: flags, appearance, HID service UUID (keep small)
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.appearance = 0x03C1;  // Keyboard
    fields.appearance_is_present = 1;
    fields.uuids16 = const_cast<ble_uuid16_t*>(&UUID_SVC_HID_BLE);
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;
    // Do NOT put name in advertising packet - put it in scan response

    // Scan response data: device name (separate 31-byte budget)
    rsp_fields.name = (uint8_t*) keyboard_name;
    rsp_fields.name_len = strlen(keyboard_name);
    rsp_fields.name_is_complete = 1;
    use_scan_response = true;
  } else {
    // Heart Sensor profile: use the configured BLE name.
    profile_name = "Heart Sensor";

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t*) name_copy.c_str();
    // BUG6 FIX: Clamp name length defensively before casting to uint8_t.
    // sanitize_ble_name() enforces a 12-byte limit, but guard here in case that
    // is ever bypassed. 29 bytes is the practical BLE adv name field maximum
    // (31-byte packet minus 2 bytes for type+length overhead).
    {
      size_t raw_len = name_copy.size();
      if (raw_len > 29) {
        ESP_LOGW(TAG, "BLE name too long (%zu bytes), truncating to 29", raw_len);
        raw_len = 29;
      }
      fields.name_len = (uint8_t) raw_len;
    }
    fields.name_is_complete = 1;
    fields.appearance = APPEARANCE_HEART_RATE_SENSOR;
    fields.appearance_is_present = 1;
    fields.uuids16 = const_cast<ble_uuid16_t*>(&UUID_SVC_HR);
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;
  }

  int rc = 0;
  const char* operation = "start";
  bool address_initialized = false;
  {
    BleOpGuard ble_lock(ble_op_mutex_, pdMS_TO_TICKS(200));
    if (!ble_lock.acquired()) {
      handle_advertising_failure_(BLE_HS_EBUSY, host_generation, "BLE operation lock");
      return;
    }
    bool address_ready;
    {
      MutexGuard lock(state_mutex_);
      if (host_generation != host_generation_ || !host_synced_ || !advertising_requested_ ||
          connected_ || bond_clear_pending_ || mac_rotation_state_ != MacRotationState::IDLE)
        return;
      address_ready = random_address_ready_;
    }
    if (!address_ready) {
      uint8_t rnd[6];
      esp_fill_random(rnd, sizeof(rnd));
      rnd[5] |= 0xC0;
      operation = "random address setup";
      rc = ble_hs_id_set_rnd(rnd);
      {
        MutexGuard lock(state_mutex_);
        if (host_generation != host_generation_ || !host_synced_) return;
        random_address_ready_ = rc == 0;
      }
      address_initialized = rc == 0;
    }

    if (rc == 0) {
      // Defensive stop - ensure clean GAP state before starting.
      int stop_rc = ble_gap_adv_stop();
      if (stop_rc != 0 && stop_rc != BLE_HS_EALREADY && stop_rc != BLE_HS_EINVAL) {
        ESP_LOGD(TAG, "ble_gap_adv_stop before start rc=%d", stop_rc);
      }
      const char* target_name =
          (current_profile == BLEProfile::KEYBOARD) ? keyboard_name : name_copy.c_str();
      operation = "GAP name update";
      rc = ble_svc_gap_device_name_set(target_name);
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

      // Use explicit RANDOM address type (our static random address set in
      // setup_ble/refresh_mac) Avoids Samsung One UI 7 "Maximum Restrictions"
      // filtering RPA addresses as tracking risks
      constexpr uint8_t own_addr_type = BLE_OWN_ADDR_RANDOM;
      operation = "start";
      rc = ble_gap_adv_start(own_addr_type, nullptr, BLE_HS_FOREVER, &advp,
                             IRKCaptureComponent::gap_event_handler, this);
    }
  }

  bool recovered = false;
  {
    MutexGuard lock(state_mutex_);
    if (host_generation != host_generation_ || !host_synced_) return;
    advertising_ = (rc == 0);
    if (rc == 0) {
      recovered = advertising_start_attempts_ != 0;
      advertising_start_attempts_ = 0;
      advertising_failure_log_time_ = 0;
      adv_restart_time_ = 0;
      pending_adv_pub_ = true;
      pending_adv_val_ = advertising_requested_;
    }
  }
  // Log outside both component locks, including when address setup succeeds
  // but a later step fails. This keeps MAC diagnostics available during recovery.
  if (address_initialized) log_mac("Effective");
  if (rc != 0) {
    handle_advertising_failure_(rc, host_generation, operation);
  } else {
    if (recovered) ESP_LOGI(TAG, "Advertising recovered; normal operation resumed");
    ESP_LOGD(TAG, "Advertising with profile: %s", profile_name);
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
    if (host_generation != host_generation_ || !host_synced_ || !advertising_requested_ ||
        connected_ || bond_clear_pending_ || mac_rotation_state_ != MacRotationState::IDLE)
      return;
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
    adv_restart_time_ = now + delay_ms;
    if (adv_restart_time_ == 0) adv_restart_time_ = 1;
    log_failure = attempts < TimingConfig::ADV_FAST_ATTEMPTS || entering_slow_recovery ||
                  now - advertising_failure_log_time_ >= TimingConfig::ADV_FAILURE_LOG_INTERVAL_MS;
    if (log_failure) advertising_failure_log_time_ = now;
    // The switch remains the user's requested state throughout recovery.
    pending_adv_pub_ = true;
    pending_adv_val_ = advertising_requested_;
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
  {
    BleOpGuard ble_lock(ble_op_mutex_, pdMS_TO_TICKS(200));
    if (!ble_lock.acquired()) {
      ESP_LOGW(TAG, "stop_advertising: ble_op_mutex timeout, skipping");
      return;
    }
    rc = ble_gap_adv_stop();
    still_active = ble_gap_adv_active() != 0;
  }
  if (rc != 0 && rc != BLE_HS_EALREADY && rc != BLE_HS_EINVAL) {
    ESP_LOGW(TAG, "ble_gap_adv_stop rc=%d", rc);
  }

  // Thread-safe actual-state update (after BLE stack call)
  {
    MutexGuard lock(state_mutex_);
    advertising_ = still_active;
  }

  if (still_active) {
    ESP_LOGW(TAG, "Advertising stop requested but NimBLE still reports it active");
  } else {
    ESP_LOGD(TAG, "Advertising stopped");
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
      suppress_next_adv_ = false;
    }
    actual = advertising_;
    connected = connected_;
  }

  // The switch is an authoritative desired-state control and therefore stays
  // ON while a peer is connected even though advertising is temporarily idle.
  stage_advertising_publish_(requested);
  if (!requested) {
    if (actual) stop_advertising();
  } else if (!connected) {
    start_advertising();
  }
}

bool IRKCaptureComponent::is_advertising() {
  MutexGuard lock(state_mutex_);
  return advertising_;
}

bool IRKCaptureComponent::is_advertising_requested() {
  MutexGuard lock(state_mutex_);
  return advertising_requested_;
}

BLEProfile IRKCaptureComponent::get_ble_profile() {
  MutexGuard lock(state_mutex_);
  return ble_profile_;
}

std::string IRKCaptureComponent::get_ble_name() {
  MutexGuard lock(state_mutex_);
  return ble_name_;
}

//======================== MAC refresh (event-driven, non-blocking)
//========================

void IRKCaptureComponent::handle_mac_rotation_(uint32_t now) {
  uint32_t generation;
  uint32_t ready_time;
  uint8_t retries;
  uint8_t mac[6];
  bool complete;
  uint16_t waiting_handle = BLE_HS_CONN_HANDLE_NONE;
  {
    MutexGuard lock(state_mutex_);
    if (!host_synced_ || bond_clear_pending_ || mac_rotation_state_ == MacRotationState::IDLE ||
        mac_rotation_state_ == MacRotationState::REQUESTED)
      return;
    generation = mac_rotation_generation_;
    complete = mac_rotation_state_ == MacRotationState::ROTATION_COMPLETE;
    if (complete) {
      mac_rotation_state_ = MacRotationState::IDLE;
    } else if (conn_handle_ != BLE_HS_CONN_HANDLE_NONE) {
      // Log once for this connection, then wait for on_disconnect() to make
      // the rotation ready again. The rest of loop() still services the link.
      waiting_handle = conn_handle_;
      mac_rotation_state_ = MacRotationState::REQUESTED;
    }
    ready_time = mac_rotation_ready_time_;
    retries = mac_rotation_retries_;
    std::memcpy(mac, pending_mac_, sizeof(mac));
  }
  if (complete) {
    ESP_LOGI(TAG, "Restarting advertising with new MAC");
    start_advertising();
    return;
  }
  if (waiting_handle != BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGW(TAG, "MAC rotation: waiting for connection handle=%u to close", waiting_handle);
    return;
  }
  if (ready_time != 0 && !deadline_reached(now, ready_time)) return;

  if (ready_time == 0 && retries == 0) {
    {
      MutexGuard lock(state_mutex_);
      if (generation != mac_rotation_generation_ || !host_synced_ || connected_ ||
          mac_rotation_state_ != MacRotationState::READY_TO_ROTATE)
        return;
      suppress_next_adv_ = false;
      adv_restart_time_ = 0;
    }
    // NVS and logging must not hold either component mutex: a slow flash
    // operation must not make a NimBLE callback wait on ble_op_mutex_.
    log_mac("Previous");
    ESP_LOGI(TAG, "Clearing bonds and waiting %" PRIu32 " ms before MAC rotation",
             TimingConfig::MAC_ROTATION_SETTLE_DELAY_MS);
    int clear_rc = ble_store_clear();
    if (clear_rc != 0) ESP_LOGW(TAG, "ble_store_clear during MAC rotation rc=%d", clear_rc);
    {
      MutexGuard lock(state_mutex_);
      if (generation != mac_rotation_generation_ || !host_synced_ || connected_ ||
          mac_rotation_state_ != MacRotationState::READY_TO_ROTATE)
        return;
      mac_rotation_ready_time_ = now_ms() + TimingConfig::MAC_ROTATION_SETTLE_DELAY_MS;
      if (mac_rotation_ready_time_ == 0) mac_rotation_ready_time_ = 1;
    }
    return;
  }

  int rc;
  {
    BleOpGuard ble_lock(ble_op_mutex_, pdMS_TO_TICKS(200));
    if (!ble_lock.acquired()) return;  // Retry on a later loop without log flooding.
    {
      MutexGuard lock(state_mutex_);
      if (generation != mac_rotation_generation_ || !host_synced_ || connected_ ||
          mac_rotation_state_ != MacRotationState::READY_TO_ROTATE)
        return;
    }
    rc = ble_hs_id_set_rnd(mac);
  }

  bool aborted = false;
  {
    MutexGuard lock(state_mutex_);
    // A reset or replacement request during the BLE call invalidates its
    // result. Never resurrect the old rotation or overwrite the new counter.
    if (generation != mac_rotation_generation_ || !host_synced_ ||
        mac_rotation_state_ != MacRotationState::READY_TO_ROTATE)
      return;
    if (rc == 0) {
      random_address_ready_ = true;
      enc_ready_ = false;
      enc_time_ = 0;
      mac_rotation_state_ = MacRotationState::ROTATION_COMPLETE;
      mac_rotation_retries_ = 0;
      mac_rotation_ready_time_ = 0;
    } else {
      retries = ++mac_rotation_retries_;
      aborted = rc != BLE_HS_EINVAL || retries >= TimingConfig::MAC_ROTATION_MAX_RETRIES;
      if (aborted) {
        mac_rotation_state_ = MacRotationState::IDLE;
        mac_rotation_retries_ = 0;
        mac_rotation_ready_time_ = 0;
      }
    }
  }
  if (rc == 0) {
    log_mac("Effective");
    ESP_LOGI(TAG, "MAC rotation complete: %02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3],
             mac[2], mac[1], mac[0]);
  } else if (aborted) {
    ESP_LOGE(TAG, "MAC rotation aborted rc=%d after %u attempts", rc, retries);
    start_advertising();
  } else {
    ESP_LOGW(TAG, "MAC rotation rc=%d, retry %u/%u", rc, retries,
             TimingConfig::MAC_ROTATION_MAX_RETRIES);
  }
}

void IRKCaptureComponent::refresh_mac() {
  ESP_LOGI(TAG, "MAC rotation requested (non-blocking event-driven)");

  // Pre-generate the new MAC address before taking mutex (esp_fill_random is
  // slow)
  uint8_t temp_mac[6];
  esp_fill_random(temp_mac, sizeof(temp_mac));
  // NimBLE uses little-endian: temp_mac[5] is the MSB (displayed first in
  // XX:XX:XX:XX:XX:XX). BLE Static Random Address requirement (Core Spec
  // Vol 6, Part B, §1.3.2): top two bits of MSB must be 11. That is the
  // only structural constraint — the unicast/multicast bit (IEEE 802) does
  // not apply to BLE addressing.
  temp_mac[5] |=
      0xC0;  // Set top two bits of MSB for static random address type (BLE spec requirement)
  // Note: all-zero is impossible after setting 0xC0 on MSB (temp_mac[5] >=
  // 0xC0)
  ESP_LOGD(TAG, "Pre-generated MAC: %02X:%02X:%02X:%02X:%02X:%02X", temp_mac[5], temp_mac[4],
           temp_mac[3], temp_mac[2], temp_mac[1], temp_mac[0]);

  // THREAD-SAFE: Atomically update state machine and pending MAC buffer
  bool should_stop_adv;
  uint16_t conn_handle_copy;
  uint32_t rotation_generation;
  {
    MutexGuard lock(state_mutex_);

    // Set rotation state and commit pre-generated MAC to shared buffer
    mac_rotation_generation_++;
    rotation_generation = mac_rotation_generation_;
    mac_rotation_state_ = MacRotationState::REQUESTED;
    mac_rotation_ready_time_ = 0;
    mac_rotation_retries_ = 0;  // Reset retry counter for new rotation attempt
    std::memcpy(pending_mac_, temp_mac, sizeof(pending_mac_));

    // Prevent handle_gap_disconnect from restarting advertising immediately,
    // which would race with ble_hs_id_set_rnd() in loop() (BLE_HS_EBUSY)
    suppress_next_adv_ = true;

    // Snapshot connection state for disconnect logic
    should_stop_adv = advertising_;
    conn_handle_copy = conn_handle_;
  }  // Release mutex before slow BLE stack calls

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
      BleOpGuard ble_lock(ble_op_mutex_, pdMS_TO_TICKS(200));
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
      // only handles READY_TO_ROTATE / ROTATION_COMPLETE). Abort the rotation
      // and restore advertising instead of getting stuck.
      if (rc != 0) {
        ESP_LOGW(TAG, "ble_gap_terminate during MAC rotation rc=%d", rc);
      }
      ESP_LOGW(TAG, "MAC rotation aborted (could not disconnect); restoring prior state");
      {
        MutexGuard lock(state_mutex_);
        if (rotation_generation != mac_rotation_generation_) return;
        mac_rotation_state_ = MacRotationState::IDLE;
        mac_rotation_retries_ = 0;
        mac_rotation_ready_time_ = 0;
        suppress_next_adv_ = false;
      }
      start_advertising();
    }
  } else {
    // No connection - safe to rotate immediately
    ESP_LOGD(TAG, "No active connection, ready to rotate MAC");
    MutexGuard lock(state_mutex_);
    if (rotation_generation != mac_rotation_generation_) return;
    mac_rotation_state_ = MacRotationState::READY_TO_ROTATE;
    // loop() will handle the actual rotation
  }
}

//======================== BLE name update ========================

bool IRKCaptureComponent::update_ble_name(const std::string& name) {
  // Compare the sanitized name before any operation that could disrupt pairing.
  {
    MutexGuard lock(state_mutex_);
    if (ble_name_ == name) return true;
  }

  // Update GAP device name
  int rc;
  {
    BleOpGuard ble_lock(ble_op_mutex_, pdMS_TO_TICKS(200));
    if (!ble_lock.acquired()) {
      ESP_LOGW(TAG, "update_ble_name: ble_op_mutex timeout, name update skipped");
      return false;
    }
    rc = ble_svc_gap_device_name_set(name.c_str());
  }
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_svc_gap_device_name_set failed rc=%d", rc);
    return false;
  }
  // Commit only after the GAP update succeeds so a failed write remains
  // retryable instead of being mistaken for an unchanged name on the next try.
  {
    MutexGuard lock(state_mutex_);
    ble_name_ = name;
  }

  // NOTE: devinfo_chrs[1].arg already points to 'this' (set in
  // register_gatt_services) GATT callback will read updated ble_name_ with
  // mutex protection

  // Initiate non-blocking MAC rotation (event-driven state machine)
  // Sequence: refresh_mac() → on_disconnect() → loop() → start_advertising()
  // The advertising will restart automatically with the new name after MAC
  // rotation completes
  this->refresh_mac();
  return true;
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
    bool persisted = false;
    nvs_handle_t nvs_handle;
    if (nvs_open("irk_capture", NVS_READWRITE, &nvs_handle) == ESP_OK) {
      // BUG10 FIX: Track set and commit errors separately so the log message
      // accurately reports which operation failed. The previous shorthand
      // (commit_err = set failed ? set_err : nvs_commit()) caused the log to
      // print the set-error code as if it were a commit error.
      esp_err_t set_err = nvs_set_u8(nvs_handle, "ble_profile", static_cast<uint8_t>(profile));
      esp_err_t commit_err = ESP_OK;
      if (set_err == ESP_OK) {
        commit_err = nvs_commit(nvs_handle);
      }
      nvs_close(nvs_handle);
      persisted = (set_err == ESP_OK && commit_err == ESP_OK);
      if (persisted) {
        ESP_LOGI(TAG, "Persisted BLE profile to NVS");
      } else {
        ESP_LOGW(TAG, "Failed to persist BLE profile to NVS (set=%d commit=%s)", set_err,
                 (set_err != ESP_OK) ? "not attempted" : esp_err_to_name(commit_err));
      }
    } else {
      ESP_LOGW(TAG, "Failed to persist BLE profile to NVS");
    }

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
        ble_name_text_->publish_state("Logitech K380");
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
    // set_ble_profile() can be reached from a caller that is not the main task
    // (the wizard serves it from esp_http_server's task), and App.safe_reboot()
    // tears down every component, which must happen on the main task. defer()
    // is safe to call from another task and runs on the main loop, which also
    // lets the caller finish first - an HTTP handler gets to send its response
    // instead of the connection dropping mid-reply.
    this->defer([]() { App.safe_reboot(); });
  }
}

//======================== GAP helpers ========================

void IRKCaptureComponent::on_connect(uint16_t conn_handle) {
  // Thread-safe connection and pairing state update
  bool was_advertising = false;
  bool reject_extra_connection = false;
  uint16_t existing_handle = BLE_HS_CONN_HANDLE_NONE;
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
      terminate_failed = rc != 0;
      if (terminate_failed) {
        ESP_LOGE(TAG, "Failed to terminate unexpected second connection rc=%d; rebooting", rc);
      }
    }
    if (terminate_failed) {
      // on_connect() runs on the NimBLE host task, but App.safe_reboot() tears
      // down every component and so has to run on the main task. Rebooting
      // from here also did it while ble_op_mutex_ was still held. defer()
      // hands the reboot to the main loop, after this handler has returned and
      // released the guard.
      this->defer([]() { App.safe_reboot(); });
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
    if (ble_store_read_peer_sec(&key, &bond) == BLE_HS_ENOENT) {
      MutexGuard lock(state_mutex_);
      if (connected_ && conn_handle_ == conn_handle) {
        pairing_generation_ = connection_generation_;
      }
    }
  }

  // Proactively initiate pairing; peer should show pairing dialog now
  // THREAD-SAFE: use the conn_handle parameter (already stored to conn_handle_
  // under mutex at the top of on_connect)
  int rc = ble_gap_security_initiate(conn_handle);
  if (rc == BLE_HS_EBUSY) {
    // Peer is already initiating security - skip our retry to avoid conflicts
    ESP_LOGD(TAG, "Peer already initiating security (EBUSY); skipping retry");
    MutexGuard lock(state_mutex_);
    sec_retry_done_ = true;  // Skip the 2-second retry since peer is handling it
  } else if (rc != 0 && rc != BLE_HS_EALREADY) {
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

//======================== IRK extraction ========================

bool IRKCaptureComponent::try_get_irk(uint16_t conn_handle, uint8_t irk_out[16],
                                      ble_addr_t& peer_id_out, uint32_t connection_generation) {
  struct ble_store_value_sec bond {};
  struct ble_gap_conn_desc desc {};

  int rc = ble_gap_conn_find(conn_handle, &desc);
  if (rc != 0) {
    ESP_LOGD(TAG, "ble_gap_conn_find failed rc=%d (handle may be invalid)", rc);
    return false;
  }

  // Build store key from the peer identity address (stable)
  struct ble_store_key_sec key_sec {};
  key_sec.peer_addr = desc.peer_id_addr;

  ESP_LOGD(TAG, "Reading bond for peer: %s", addr_to_str(desc.peer_id_addr).c_str());

  rc = ble_store_read_peer_sec(&key_sec, &bond);
  if (rc == BLE_HS_ENOENT) {
    ESP_LOGD(TAG, "No bond for peer (ENOENT) - IRK not yet written to NVS");
    return false;
  }
  if (rc != 0) {
    ESP_LOGW(TAG, "ble_store_read_peer_sec failed rc=%d", rc);
    return false;
  }

  // Detailed bond information for debugging
  ESP_LOGD(TAG,
           "Bond found: ediv=%u rand=%llu irk_present=%d ltk_present=%d "
           "csrk_present=%d",
           (unsigned) bond.ediv, (unsigned long long) bond.rand_num, (int) bond.irk_present,
           (int) bond.ltk_present, (int) bond.csrk_present);

  ESP_LOGD(TAG, "Bond security: authenticated=%d sc=%d", (int) bond.authenticated, (int) bond.sc);

  // Defensive bounds check: Ensure IRK is present before copying
  if (!bond.irk_present) {
    // This read is called only by polling after successful encryption.
    publish_no_irk_(desc.peer_id_addr, connection_generation, true);
    return false;
  }

  // Validate IRK before accepting
  if (!is_valid_irk(bond.irk)) {
    ESP_LOGW(TAG, "IRK failed validation (all-zero or all-FF)");
    return false;
  }

  // Return raw IRK bytes (16 bytes guaranteed by NimBLE) and peer identity
  // address
  std::memcpy(irk_out, bond.irk, 16);
  peer_id_out = desc.peer_id_addr;
  return true;
}

//======================== Timer helpers ========================

bool IRKCaptureComponent::enqueue_peer_timer_(std::array<PeerTimer, PEER_TIMER_CAPACITY>& timers,
                                              const ble_addr_t& peer_id,
                                              uint32_t connection_generation, uint32_t delay_ms,
                                              bool pairing_completed) {
  uint32_t due_ms = now_ms() + delay_ms;
  if (due_ms == 0) due_ms = 1;  // Zero is the unused-slot sentinel.
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
      return false;
    }
    if (!oldest || static_cast<int32_t>(timer.due_ms - oldest->due_ms) < 0) {
      oldest = &timer;
    }
  }
  // Losing the oldest fallback read is preferable to rebooting and clearing
  // every bond and capture in the current session. Prefer a free slot first.
  PeerTimer* target = available ? available : oldest;
  *target = { due_ms, peer_id, connection_generation, pairing_completed };
  return available == nullptr;
}

void IRKCaptureComponent::schedule_post_disconnect_check(const ble_addr_t& peer_id,
                                                         uint32_t connection_generation,
                                                         bool pairing_completed) {
  if (enqueue_peer_timer_(post_disc_timers_, peer_id, connection_generation,
                          TimingConfig::POST_DISC_DELAY_MS, pairing_completed)) {
    ESP_LOGW(TAG, "Post-disconnect timer queue full; replaced oldest fallback check");
  }
}

void IRKCaptureComponent::schedule_late_enc_check(const ble_addr_t& peer_id,
                                                  uint32_t connection_generation) {
  if (enqueue_peer_timer_(late_enc_timers_, peer_id, connection_generation,
                          TimingConfig::ENC_LATE_READ_DELAY_MS)) {
    ESP_LOGW(TAG, "Late-ENC timer queue full; replaced oldest fallback check");
  }
}

void IRKCaptureComponent::handle_post_disconnect_timer(uint32_t now) {
  // Consume one due entry per loop; additional due entries run on subsequent
  // loop ticks without being overwritten.
  ble_addr_t peer_id {};
  uint32_t connection_generation = 0;
  bool pairing_completed = false;
  bool timer_due = false;
  {
    MutexGuard lock(state_mutex_);
    for (auto& timer : post_disc_timers_) {
      if (timer.due_ms != 0 && deadline_reached(now, timer.due_ms)) {
        peer_id = timer.peer_id;
        connection_generation = timer.connection_generation;
        pairing_completed = timer.pairing_completed;
        timer = {};
        timer_due = true;
        break;
      }
    }
  }
  if (!timer_due) return;

  if (addr_is_zero(peer_id)) {
    ESP_LOGD(TAG, "Post-disc timer fired but post_disc_peer_id is zero (skipping)");
    return;
  }

  struct ble_store_value_sec bond {};
  struct ble_store_key_sec key {};
  key.peer_addr = peer_id;
  int rc = ble_store_read_peer_sec(&key, &bond);
  if (rc == BLE_HS_ENOENT) {
    ESP_LOGD(TAG, "No bond for peer (ENOENT) - post-disc delayed check");
  } else if (rc != 0) {
    ESP_LOGW(TAG, "ble_store_read_peer_sec rc=%d - post-disc delayed check", rc);
  } else if (bond.irk_present && is_valid_irk(bond.irk)) {
    std::string irk_hex = to_hex_rev(bond.irk, sizeof(bond.irk));
    publish_and_log_irk(this, peer_id, irk_hex, "DISC_DELAYED", connection_generation);
  } else if (!bond.irk_present) {
    publish_no_irk_(peer_id, connection_generation, pairing_completed);
  } else {
    log_no_irk_for_peer(peer_id);
  }
}

void IRKCaptureComponent::handle_late_enc_timer(uint32_t now) {
  ble_addr_t peer_id {};
  uint32_t connection_generation = 0;
  bool timer_due = false;
  {
    MutexGuard lock(state_mutex_);
    for (auto& timer : late_enc_timers_) {
      if (timer.due_ms != 0 && deadline_reached(now, timer.due_ms)) {
        peer_id = timer.peer_id;
        connection_generation = timer.connection_generation;
        timer = {};
        timer_due = true;
        break;
      }
    }
  }
  if (!timer_due) return;

  if (addr_is_zero(peer_id)) {
    ESP_LOGD(TAG, "Late ENC timer fired but enc_peer_id is zero (skipping)");
    return;
  }

  struct ble_store_key_sec key {};
  key.peer_addr = peer_id;
  struct ble_store_value_sec bond {};
  int rc = ble_store_read_peer_sec(&key, &bond);
  if (rc == BLE_HS_ENOENT) {
    ESP_LOGD(TAG, "No bond for peer (ENOENT) - late ENC check");
  } else if (rc != 0) {
    ESP_LOGW(TAG, "ble_store_read_peer_sec rc=%d - late ENC check", rc);
  } else if (bond.irk_present && is_valid_irk(bond.irk)) {
    std::string irk_hex = to_hex_rev(bond.irk, sizeof(bond.irk));
    publish_and_log_irk(this, peer_id, irk_hex, "ENC_LATE", connection_generation);

    // Tested working behavior: terminate after late capture if still connected
    // THREAD-SAFE: Check connection state before terminating
    bool is_connected;
    uint16_t conn_handle_copy;
    uint32_t current_generation;
    {
      MutexGuard lock(state_mutex_);
      is_connected = connected_;
      conn_handle_copy = conn_handle_;
      current_generation = connection_generation_;
    }
    // Only terminate if the CURRENT connection is still the same peer this late
    // check was scheduled for. In continuous mode the original peer may have
    // disconnected and a different device connected during the delay; without
    // this guard we would drop that unrelated device.
    bool same_peer = false;
    if (is_connected && current_generation == connection_generation &&
        conn_handle_copy != BLE_HS_CONN_HANDLE_NONE) {
      struct ble_gap_conn_desc cur {};
      same_peer =
          (ble_gap_conn_find(conn_handle_copy, &cur) == 0) && addr_equal(cur.peer_id_addr, peer_id);
    }
    if (same_peer) {
      int term_rc;
      {
        BleOpGuard ble_lock(ble_op_mutex_);
        term_rc = ble_gap_terminate(conn_handle_copy, BLE_ERR_REM_USER_CONN_TERM);
      }
      if (term_rc != 0) {
        ESP_LOGW(TAG, "ble_gap_terminate after late ENC IRK capture rc=%d", term_rc);
      }
    }
  } else if (!bond.irk_present) {
    // Late encryption checks are scheduled only after ENC_CHANGE success.
    publish_no_irk_(peer_id, connection_generation, true);
  } else {
    log_no_irk_for_peer(peer_id);
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
      ESP_LOGI(TAG, "Retrying security initiate after %" PRIu32 " ms", elapsed);
      // BUG2 FIX: Set sec_retry_done_ under mutex BEFORE calling
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
      // rc==0 (started), EALREADY/EBUSY (peer already pairing) are all benign;
      // only a genuine error deserves a warning.
      if (rc == 0 || rc == BLE_HS_EALREADY || rc == BLE_HS_EBUSY) {
        ESP_LOGD(TAG, "Retry security initiate rc=%d", rc);
      } else {
        ESP_LOGW(TAG, "Retry security initiate rc=%d", rc);
      }
    }
  }
}

void IRKCaptureComponent::reset_connection_state_() {
  // Caller holds state_mutex_. Keep the generation for delayed capture results;
  // on_connect() advances it before the next peer can become active.
  connected_ = false;
  conn_handle_ = BLE_HS_CONN_HANDLE_NONE;
  pairing_start_time_ = 0;
  enc_ready_ = false;
  enc_time_ = 0;
  sec_retry_done_ = false;
  sec_init_time_ms_ = 0;
  irk_gave_up_ = false;
  irk_last_try_ms_ = 0;
  connection_timeout_ = {};
}

void IRKCaptureComponent::finish_disconnect_(uint32_t now) {
  // Caller holds state_mutex_. A vanished descriptor and a GAP disconnect must
  // perform the same cleanup, including a pending MAC rotation's handoff.
  if (connection_timeout_.reason != TimeoutReason::NONE) {
    suppress_next_adv_ = false;
    if (advertising_requested_) {
      adv_restart_time_ = now + TimingConfig::TIMEOUT_COOLDOWN_MS;
      if (adv_restart_time_ == 0) adv_restart_time_ = 1;
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
          now - sec_init_time_ms_ > TimingConfig::SEC_TIMEOUT_MS) {
        connection_timeout_.reason = TimeoutReason::ENCRYPTION;
      } else if (pairing_start_time_ != 0 &&
                 now - pairing_start_time_ > TimingConfig::PAIRING_TOTAL_TIMEOUT_MS) {
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
    // Flash work must run outside both component locks.
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
    BleOpGuard ble_lock(ble_op_mutex_, pdMS_TO_TICKS(200));
    acquired = ble_lock.acquired();
    {
      MutexGuard lock(state_mutex_);
      if (!is_current()) return true;
      attempts = ++connection_timeout_.attempts;
      // Arm the backoff before the stack call. A synchronous callback must not
      // see an unclaimed attempt, and a disconnect may reset this state entirely.
      connection_timeout_.retry_ms = now + TimingConfig::TERMINATE_RETRY_MS;
      if (connection_timeout_.retry_ms == 0) connection_timeout_.retry_ms = 1;
    }
    if (acquired) rc = ble_gap_terminate(handle, BLE_ERR_REM_USER_CONN_TERM);
  }

  const bool accepted = rc == 0 || rc == BLE_HS_EALREADY;
  {
    MutexGuard lock(state_mutex_);
    if (!is_current()) return true;
    if (accepted) {
      connection_timeout_.retry_ms = now + TimingConfig::TERMINATE_PENDING_GRACE_MS;
      if (connection_timeout_.retry_ms == 0) connection_timeout_.retry_ms = 1;
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

void IRKCaptureComponent::notify_hr_if_due(uint32_t now) {
  // Read the handle NimBLE populated during ble_gatts_start(); it is still 0
  // when the Keyboard profile is active (no HR service) or before host start.
  const uint16_t hr_handle = g_hr_handle;
  if (hr_handle == 0) return;
  if (now - last_notify_ <= TimingConfig::HR_NOTIFY_INTERVAL_MS) return;

  // THREAD-SAFE: Copy connection handle before BLE stack call
  uint16_t conn_handle_copy;
  {
    MutexGuard lock(state_mutex_);
    conn_handle_copy = conn_handle_;
  }

  last_notify_ = now;
  uint8_t buf[2];
  size_t len;
  hr_measurement_sample(buf, &len);
  struct os_mbuf* om = ble_hs_mbuf_from_flat(buf, len);
  int rc = ble_gatts_notify_custom(conn_handle_copy, hr_handle, om);
  if (rc != 0) {
    ESP_LOGD(TAG, "notify rc=%d", rc);
  }
}

void IRKCaptureComponent::poll_irk_if_due(uint32_t now) {
  // THREAD-SAFE: Snapshot polling state under mutex for main-loop reads
  bool enc_ready_copy;
  bool irk_gave_up_copy;
  uint32_t enc_time_copy;
  uint32_t irk_last_try_copy;
  uint16_t conn_handle_copy;
  uint32_t connection_generation_copy;
  {
    MutexGuard lock(state_mutex_);
    enc_ready_copy = enc_ready_;
    irk_gave_up_copy = irk_gave_up_;
    enc_time_copy = enc_time_;
    irk_last_try_copy = irk_last_try_ms_;
    conn_handle_copy = conn_handle_;
    connection_generation_copy = connection_generation_;
  }

  // Attempt IRK retrieval after encryption + delay (allow store write)
  if (!enc_ready_copy || irk_gave_up_copy) return;
  if ((now - enc_time_copy) < TimingConfig::ENC_TO_FIRST_TRY_DELAY_MS) return;
  if ((now - irk_last_try_copy) < TimingConfig::ENC_TRY_INTERVAL_MS) return;

  {
    MutexGuard lock(state_mutex_);
    irk_last_try_ms_ = now;
  }

  uint8_t irk_bytes[16];
  ble_addr_t peer_id;
  if (try_get_irk(conn_handle_copy, irk_bytes, peer_id, connection_generation_copy)) {
    std::string irk_hex = to_hex_rev(irk_bytes, sizeof(irk_bytes));
    publish_and_log_irk(this, peer_id, irk_hex, "POLL_CONNECTED", connection_generation_copy);
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
    int term_rc;
    {
      BleOpGuard ble_lock(ble_op_mutex_);
      term_rc = ble_gap_terminate(conn_handle_copy, BLE_ERR_REM_USER_CONN_TERM);
    }
    if (term_rc != 0) {
      ESP_LOGW(TAG, "ble_gap_terminate after poll IRK capture rc=%d", term_rc);
    }
  } else {
    if ((now - enc_time_copy) > TimingConfig::ENC_GIVE_UP_AFTER_MS) {
      ESP_LOGW(TAG, "IRK not found after %" PRIu32 " ms post-encryption",
               TimingConfig::ENC_GIVE_UP_AFTER_MS);
      MutexGuard lock(state_mutex_);
      // Only give up on the connection this poll was actually running against.
      if (connected_ && connection_generation_ == connection_generation_copy) {
        irk_gave_up_ = true;
      }
    }
  }
}

//======================== Public publish utility ========================

void IRKCaptureComponent::publish_no_irk_(const ble_addr_t& peer_id, uint32_t connection_generation,
                                          bool pairing_completed) {
  if (!pairing_completed || connection_generation == 0) return;
  const std::string addr = addr_to_str(peer_id);
  {
    MutexGuard lock(state_mutex_);
    // Only complete the current attempt, and never replace its successful result
    // or repeatedly emit the same failure from polling/disconnect/timer paths.
    if (connection_generation != connection_generation_ ||
        last_result_generation_ == connection_generation)
      return;
    last_result_generation_ = connection_generation;
    pending_irk_hex_ = "Failed: IRK not used";
    pending_irk_addr_ = addr;
    pending_irk_pub_ = true;
    capture_status_ = CaptureStatus::NO_IRK;
    status_result_hold_until_ = now_ms() + TimingConfig::STATUS_RESULT_HOLD_MS;
    if (status_result_hold_until_ == 0) status_result_hold_until_ = 1;
    if (connected_) irk_gave_up_ = true;
    for (auto* timers : { &post_disc_timers_, &late_enc_timers_ }) {
      for (auto& timer : *timers) {
        if (timer.connection_generation == connection_generation &&
            addr_equal(timer.peer_id, peer_id))
          timer = {};
      }
    }
  }
  // A completed pairing without an identity key does not by itself prove the
  // peer uses a fixed address, so offer the MAC as a conditional next step
  // rather than asserting that privacy is disabled.
  ESP_LOGW(TAG, "IRK capture failed for %s: completed pairing did not provide an IRK",
           addr.c_str());
  ESP_LOGW(TAG,
           "No IRK was provided. If this device uses a fixed BLE address, use this MAC "
           "directly: %s",
           addr.c_str());
}

void IRKCaptureComponent::publish_irk_to_sensors(const std::string& irk_hex, const char* addr_str,
                                                 uint32_t connection_generation,
                                                 bool capture_event) {
  // Stage only; the ESPHome main loop() performs the actual publish_state().
  // Callers may run in the NimBLE task, where publish_state() is unsafe.
  MutexGuard lock(state_mutex_);
  // A valid IRK for the live connection ends its capture work whether or not it
  // becomes the displayed result, so Status stops reporting "capturing".
  if (connected_ && connection_generation != 0 && connection_generation == connection_generation_) {
    irk_gave_up_ = true;
  }
  // An older delayed result must not overwrite the outcome of a newer pairing.
  if (connection_generation != 0 && last_result_generation_ != 0 &&
      static_cast<int32_t>(connection_generation - last_result_generation_) < 0)
    return;
  // Stage status and IRK under the same ordering check. Only a genuine capture
  // replaces what is on display. A bonded reconnect of an earlier device may
  // restore its IRK over a no-IRK failure (or an empty sensor), but must never
  // replace another device's result while the user is copying it.
  if (capture_event) {
    capture_status_ = CaptureStatus::CAPTURED;
    status_result_hold_until_ = now_ms() + TimingConfig::STATUS_RESULT_HOLD_MS;
    if (status_result_hold_until_ == 0) status_result_hold_until_ = 1;
  } else if (capture_status_ == CaptureStatus::NO_IRK || last_result_generation_ == 0) {
    capture_status_ = CaptureStatus::NONE;
    status_result_hold_until_ = 0;
  } else {
    return;
  }
  if (connection_generation != 0) last_result_generation_ = connection_generation;
  pending_irk_hex_ = irk_hex;
  pending_irk_addr_ = addr_str;
  pending_irk_pub_ = true;
}

void IRKCaptureComponent::publish_effective_mac() {
  if (!effective_mac_sensor_) return;

  uint8_t mac[6];
  if (get_own_addr(mac, nullptr)) {
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3],
             mac[2], mac[1], mac[0]);
    // Stage only; drained on the main task in loop().
    {
      MutexGuard lock(state_mutex_);
      pending_effmac_ = mac_str;
      pending_effmac_pub_ = true;
    }
    ESP_LOGD(TAG, "Staged effective MAC: %s", mac_str);
  }
}

void IRKCaptureComponent::stage_advertising_publish_(bool value) {
  MutexGuard lock(state_mutex_);
  pending_adv_val_ = value;
  pending_adv_pub_ = true;
}

void IRKCaptureComponent::flush_pending_publishes_() {
  // Runs on the ESPHome main task. Copy staged values out under the mutex, then
  // publish outside it (publish_state can be slow and must not hold the lock).
  bool adv_pub, adv_val, irk_pub, effmac_pub;
  bool stop_after_pub, stop_after_val, label_pub;
  std::string irk_hex, irk_addr, effmac, label_val;
  {
    MutexGuard lock(state_mutex_);
    adv_pub = pending_adv_pub_;
    adv_val = pending_adv_val_;
    pending_adv_pub_ = false;
    irk_pub = pending_irk_pub_;
    pending_irk_pub_ = false;
    irk_hex.swap(pending_irk_hex_);
    irk_addr.swap(pending_irk_addr_);
    effmac_pub = pending_effmac_pub_;
    pending_effmac_pub_ = false;
    effmac.swap(pending_effmac_);
    stop_after_pub = pending_stop_after_capture_pub_;
    stop_after_val = pending_stop_after_capture_val_;
    pending_stop_after_capture_pub_ = false;
    label_pub = pending_label_pub_;
    pending_label_pub_ = false;
    label_val.swap(pending_label_val_);
  }
  if (adv_pub && advertising_switch_) advertising_switch_->publish_state(adv_val);
  if (irk_pub) {
    if (irk_sensor_) irk_sensor_->publish_state(irk_hex);
    if (address_sensor_) address_sensor_->publish_state(irk_addr);
  }
  if (effmac_pub && effective_mac_sensor_) effective_mac_sensor_->publish_state(effmac);
  if (stop_after_pub && stop_after_capture_switch_)
    stop_after_capture_switch_->publish_state(stop_after_val);
  if (label_pub && next_capture_label_text_) next_capture_label_text_->publish_state(label_val);
}

//======================== Wizard-facing controls (1.7.0) ========================

void IRKCaptureComponent::set_stop_after_capture(bool enabled) {
  MutexGuard lock(state_mutex_);
  stop_after_capture_ = enabled;
  pending_stop_after_capture_val_ = enabled;
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

std::string IRKCaptureComponent::set_next_capture_label(const std::string& value) {
  // Same safe charset as sanitize_ble_name(), but labels aren't advertised
  // over the air, so they get a longer budget than the 12-byte BLE-name cap.
  std::string sanitized;
  sanitized.reserve(24);
  for (char c : value) {
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == ' ' ||
        c == '-' || c == '_') {
      sanitized += c;
    }
    if (sanitized.length() >= 24) break;
  }
  {
    MutexGuard lock(state_mutex_);
    next_capture_label_ = sanitized;
    pending_label_val_ = sanitized;
    pending_label_pub_ = true;
  }
  ESP_LOGD(TAG, "Next capture label set to '%s'", sanitized.c_str());
  return sanitized;
}

void IRKCaptureComponent::forget_all_bonds() {
  bool can_queue;
  bool worker_available;
  {
    MutexGuard lock(state_mutex_);
    // These entries also coalesce disconnect/timer reads and account for unique
    // devices. Hide history instead of discarding that session bookkeeping.
    for (auto& entry : irk_cache_) {
      entry.in_history = false;
      entry.label.clear();
    }
    if (bond_clear_pending_) return;
    worker_available = bond_clear_task_ != nullptr;
    can_queue = worker_available && host_synced_ && mac_rotation_state_ == MacRotationState::IDLE;
    if (can_queue) {
      bond_clear_pending_ = true;
      bond_clear_host_generation_ = host_generation_;
    }
  }
  if (!worker_available) {
    ESP_LOGW(TAG,
             "Forget All Bonds: history cleared; stored bonds retained because the "
             "bond-clear worker is unavailable");
    return;
  }
  if (!can_queue) {
    ESP_LOGW(TAG,
             "Forget All Bonds: history cleared; BLE is not ready or MAC rotation is active. "
             "Run it again once BLE is ready to clear stored bonds.");
    return;
  }
  notify_bond_clear_();
}

void IRKCaptureComponent::notify_bond_clear_() {
  {
    MutexGuard lock(state_mutex_);
    if (!bond_clear_task_ || !bond_clear_pending_) return;
  }
  // Task notifications do not wait for queue capacity. A reset racing this
  // notification is harmless: the worker rechecks the pending request.
  xTaskNotify(bond_clear_task_, 1, eSetBits);
}

void IRKCaptureComponent::queue_bond_clear_() {
  {
    MutexGuard lock(state_mutex_);
    if (!bond_clear_pending_ || !host_synced_ || host_generation_ != bond_clear_host_generation_)
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
  int stop_rc = BLE_HS_EBUSY;
  {
    MutexGuard lock(state_mutex_);
    if (!bond_clear_pending_) return;  // A loop retry may have queued a duplicate.
  }
  {
    // Do not block the host behind a main-task operation that may need it.
    BleOpGuard ble_lock(ble_op_mutex_, 0);
    if (ble_lock.acquired()) {
      {
        MutexGuard lock(state_mutex_);
        can_clear = host_synced_ && host_generation_ == bond_clear_host_generation_ &&
                    !connected_ && mac_rotation_state_ == MacRotationState::IDLE;
      }
      if (can_clear) {
        stop_rc = ble_gap_adv_stop();
        const bool active = ble_gap_adv_active() != 0;
        MutexGuard lock(state_mutex_);
        advertising_ = active;
        can_clear = !active && host_synced_ && host_generation_ == bond_clear_host_generation_ &&
                    !connected_ && mac_rotation_state_ == MacRotationState::IDLE &&
                    (stop_rc == 0 || stop_rc == BLE_HS_EALREADY || stop_rc == BLE_HS_EINVAL);
      }
    }
  }

  if (can_clear) {
    // Flash work must not hold either component mutex.
    const int rc = ble_store_clear();
    if (rc != 0) {
      ESP_LOGE(TAG, "Forget All Bonds: ble_store_clear failed rc=%d; run it again to retry", rc);
    } else {
      ESP_LOGI(TAG, "All BLE bonds cleared");
    }
  } else {
    ESP_LOGW(TAG,
             "Forget All Bonds: history cleared; stored bonds retained because a peer is "
             "connected or BLE maintenance is busy (stop rc=%d). Run it again once idle.",
             stop_rc);
  }
  {
    MutexGuard lock(state_mutex_);
    bond_clear_pending_ = false;
  }
  // Re-evaluate current intent: an OFF request or capture limit reached while
  // the event was queued must not be undone by restoring an old snapshot.
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

  std::string json;
  json.reserve(cache_copy.size() * 64 + 2);
  json += "[";
  bool first = true;
  for (const auto& entry : cache_copy) {
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
    json += "}";
  }
  json += "]";

  return json;
}

void IRKCaptureComponent::update_status_sensor_(uint32_t now) {
  if (!status_sensor_) return;

  const char* status;
  {
    MutexGuard lock(state_mutex_);
    // Retire expired deadlines permanently: signed wrap-safe comparisons only
    // order timestamps within half the clock period (about 25 days).
    if (status_result_hold_until_ != 0 && deadline_reached(now, status_result_hold_until_)) {
      status_result_hold_until_ = 0;
    }
    const bool current_result =
        connection_generation_ != 0 && last_result_generation_ == connection_generation_;
    if (capture_status_ != CaptureStatus::NONE &&
        (status_result_hold_until_ != 0 || (connected_ && current_result))) {
      status = capture_status_ == CaptureStatus::CAPTURED ? "captured" : "no_irk";
    } else if (connected_ && enc_ready_) {
      // irk_gave_up_ also covers polling timeouts; it is not evidence of a
      // no-IRK outcome. Completed reconnects/timeouts are simply waiting to close.
      status = !current_result && !irk_gave_up_ ? "capturing" : "idle";
    } else if (connected_) {
      status = "pairing";
    } else if (advertising_) {
      status = "advertising";
    } else {
      status = "idle";
    }
  }

  if (last_status_value_ != status) {
    last_status_value_ = status;
    status_sensor_->publish_state(status);
  }
}

}  // namespace irk_capture
}  // namespace esphome

#endif  // USE_ESP32
