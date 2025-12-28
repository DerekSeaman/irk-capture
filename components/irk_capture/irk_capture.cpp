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

#include "esphome/core/application.h"
#include "esphome/core/log.h"

// Some ESP-IDF 5.x package variants omit this prototype from headers; declare
// it explicitly
extern "C" int ble_store_config_init(void);
extern "C" int ble_store_clear(void);

namespace esphome {
namespace irk_capture {

static const char* const TAG = "irk_capture";
static constexpr char VERSION[] = "1.5.0";
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

THREAD SAFETY RULES:
-    ALL reads/writes to shared state MUST use state_mutex_
-    Protected by state_mutex_ (FreeRTOS mutex):
     * timers_.last_peer_id / timers_.enc_peer_id (ble_addr_t multi-word structs)
     * timers_.post_disc_due_ms / timers_.late_enc_due_ms (timer targets)
     * conn_handle_ (connection handle)
     * connected_ (connection state flag)
     * advertising_ (advertising state flag)
     * pairing_start_time_ (pairing timeout tracking)
     * ble_name_, manufacturer_name_ (device names)
     * mac_rotation_state_ (MAC rotation state machine)
     * pending_mac_ (6-byte pre-generated MAC buffer)
     * suppress_next_adv_ (advertising suppression flag)
     * adv_restart_time_ (advertising restart timer)
     * total_captures_ (session IRK counter - NOT atomic)
     * irk_cache_ (deduplication vector - push_back/erase NOT thread-safe)
     * last_publish_time_ (rate limiting timestamp)
-    RAII MutexGuard class ensures exception-safe lock/unlock
-    Mutex MUST be released before calling BLE stack APIs (prevents deadlock)
-    UI updates (publish_state) are safe outside mutex (internally thread-safe)

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
   - Transitions to DISCONNECTING when IRK captured or timeout (45s)

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
  static constexpr uint32_t IRK_MIN_POLL_INTERVAL_MS =
      200;  // Minimum time between IRK polling attempts (prevents spam)
  static constexpr uint32_t SEC_RETRY_DELAY_MS =
      2000;  // Delay before retrying security after encryption failure
  static constexpr uint32_t SEC_TIMEOUT_MS = 20000;  // Timeout for encryption to complete (assumes
                                                     // peer forgot pairing)
  static constexpr uint32_t ADV_SUPPRESS_DURATION_MS =
      2000;  // How long to suppress advertising after IRK capture
  static constexpr uint32_t PAIRING_TOTAL_TIMEOUT_MS = 90000;  // Global pairing timeout (90s max)
  static constexpr uint32_t TIMEOUT_COOLDOWN_MS =
      5000;  // Cooldown after pairing timeout before re-advertising (prevents rapid-fire loop)
  static constexpr uint32_t MIN_REPUBLISH_INTERVAL_MS =
      60000;  // Min time between republishing same IRK (60s)
};

// GATT service and characteristic UUIDs
static constexpr uint16_t UUID_SVC_HEART_RATE = 0x180D;
static constexpr uint16_t UUID_CHR_HEART_RATE_MEASUREMENT = 0x2A37;
static constexpr uint16_t UUID_SVC_DEVICE_INFO = 0x180A;
static constexpr uint16_t UUID_CHR_MANUFACTURER_NAME = 0x2A29;
static constexpr uint16_t UUID_CHR_MODEL_NUMBER = 0x2A24;
static constexpr uint16_t UUID_SVC_BATTERY = 0x180F;
static constexpr uint16_t UUID_CHR_BATTERY_LEVEL = 0x2A19;

// BLE Appearance values
static constexpr uint16_t APPEARANCE_HEART_RATE_SENSOR = 0x0340;

//======================== IRK lifecycle (for readers) ========================
/*
Connect → Initiate security
ENC_CHANGE → immediate IRK read from store; if available: publish & disconnect
(tested working behavior). If ENC fails (status != 0), delete that peer's keys and retry
security once (self-heal). DISCONNECT → immediate store read; schedule delayed
read at +800ms; restart advertising While connected (post ENC) → poll every 1s
starting at +2s, up to 45s, then disconnect when IRK captured All address
reporting uses the peer identity address; IRK hex is reversed for parity with
Arduino output.

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

// Generic hex formatter for byte arrays
static std::string to_hex_str(const uint8_t* data, size_t len, bool reverse = false) {
  std::string out;
  out.reserve(len * 2);
  if (reverse) {
    for (int i = (int) len - 1; i >= 0; --i) {
      uint8_t c = data[i];
      out.push_back(HEX[(c >> 4) & 0xF]);
      out.push_back(HEX[c & 0xF]);
    }
  } else {
    for (size_t i = 0; i < len; ++i) {
      uint8_t c = data[i];
      out.push_back(HEX[(c >> 4) & 0xF]);
      out.push_back(HEX[c & 0xF]);
    }
  }
  return out;
}

// Reverse byte order for hex output (matches Arduino BLE library convention for
// IRK display)
static std::string bytes_to_hex_rev(const uint8_t* data, size_t len) {
  return to_hex_str(data, len, true);
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
- timers_.last_peer_id / timers_.enc_peer_id (ble_addr_t multi-word structs)
- timers_.post_disc_due_ms / timers_.late_enc_due_ms (timer targets)
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
    peer_id = timers_.last_peer_id;
    ESP_LOGD(TAG, "Peer: %s", addr_to_str(peer_id).c_str()); // SLOW - UART
bottleneck!
  }

GOOD (minimal lock hold time):
  ble_addr_t peer_id_copy;
  {
    MutexGuard lock(state_mutex_);
    peer_id_copy = timers_.last_peer_id;  // Fast memory copy
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

static void hr_measurement_sample(uint8_t* buf, size_t* len) {
  buf[0] = 0x00;  // Flags byte: bit0=0 (UINT8 format), bit1-2=0 (sensor contact not supported),
                  //             bit3=0 (no energy expended), bit4=0 (RR interval not present)
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
  // Reject all-zero IRK (invalid)
  bool all_zero = true;
  for (int i = 0; i < 16; i++) {
    if (irk[i] != 0x00) {
      all_zero = false;
      break;
    }
  }
  if (all_zero) {
    ESP_LOGW(TAG, "Rejected all-zero IRK (invalid)");
    return false;
  }

  // Reject all-FF IRK (uninitialized)
  bool all_ff = true;
  for (int i = 0; i < 16; i++) {
    if (irk[i] != 0xFF) {
      all_ff = false;
      break;
    }
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
 * This function modifies irk_cache_, last_publish_time_, and reads advertising_.
 *
 * MEMORY SAFETY: This function prevents duplicate entries in irk_cache_ by
 * checking if the IRK already exists before adding. Even if the same device
 * reconnects 100 times, it will only have ONE entry in the cache (updated
 * in-place). Hard cap at 10 entries prevents unbounded memory growth on
 * ESP32-C3.
 *
 * @param irk_hex IRK in hex string format
 * @param addr MAC address string
 * @param out_should_stop_adv [out] Set to true if caller should stop advertising (limit reached)
 * @return true if should publish, false if duplicate/rate-limited
 */
bool IRKCaptureComponent::should_publish_irk(const std::string& irk_hex, const std::string& addr,
                                             bool& out_should_stop_adv) {
  // PRECONDITION: Caller holds state_mutex_
  out_should_stop_adv = false;  // Default: don't stop advertising
  uint32_t now = now_ms();

  // Check cache for duplicate - prevents memory bloat from repeated connections
  // SAFETY: If found, we update the existing entry in-place (no new allocation)
  for (auto& entry : irk_cache_) {
    if (entry.irk_hex == irk_hex && entry.mac_addr == addr) {
      // Same device reconnecting - update existing entry, don't create new one
      entry.last_seen_ms = now;
      entry.capture_count++;

      // SESSION LIMIT: Prevent unbounded memory growth from background reconnections
      // iOS devices reconnect every 15min → 96 times/day → heap fragmentation
      if (entry.capture_count > 5) {
        ESP_LOGW(TAG,
                 "IRK republish limit reached (%u captures). Device keeps reconnecting - "
                 "unpair from Bluetooth settings to stop.",
                 entry.capture_count);

        // Signal caller to stop advertising (break reconnection loop)
        // THREAD-SAFE: We already hold state_mutex_, so we can read advertising_ directly
        out_should_stop_adv = advertising_;
        return false;  // Don't republish - prevents heap fragmentation
      }

      // Rate limit republishing to Home Assistant (60s minimum)
      if ((now - last_publish_time_) < TimingConfig::MIN_REPUBLISH_INTERVAL_MS) {
        ESP_LOGD(TAG, "Suppressing duplicate IRK (published %u ms ago)", now - last_publish_time_);
        return false;
      }

      ESP_LOGI(TAG, "Re-publishing IRK (capture #%u/5)", entry.capture_count);
      last_publish_time_ = now;
      return true;
    }
  }

  // New IRK - add to cache (max 10 entries, FIFO eviction)
  // SAFETY: Hard cap at 10 entries prevents unbounded memory growth on ESP32-C3
  if (irk_cache_.size() >= 10) {
    // Evict oldest (FIFO). Note: documented behavior; intentional to cap memory
    // and keep UX predictable.
    ESP_LOGD(TAG, "IRK cache full (10 entries), evicting oldest entry");
    irk_cache_.erase(irk_cache_.begin());
  }
  irk_cache_.push_back({ irk_hex, addr, now, now, 1 });
  last_publish_time_ = now;
  ESP_LOGD(TAG, "New IRK added to cache (total: %zu/10)", irk_cache_.size());
  return true;
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
 *
 * This is the single point of IRK output, ensuring consistent formatting across
 * all capture paths. Called from multiple locations: ENC_CHANGE, DISCONNECT,
 * post-disconnect timer, polling loop.
 */
void publish_and_log_irk(IRKCaptureComponent* self, const ble_addr_t& peer_id_addr,
                         const std::string& irk_hex, const char* context_tag) {
  if (!self) return;  // Early return if no component instance

  const std::string addr_str = addr_to_str(peer_id_addr);

  // THREAD-SAFE: Check deduplication AND increment counter under mutex
  // CRITICAL: should_publish_irk() assumes caller holds mutex (non-recursive mutex!)
  // All irk_cache_ operations, counter increments, and advertising checks happen atomically
  uint8_t current_captures;
  bool max_reached = false;
  bool should_publish;
  bool should_stop_adv = false;  // Output from deduplication check
  {
    MutexGuard lock(self->state_mutex_);

    // Deduplication check (modifies irk_cache_ and last_publish_time_)
    // PRECONDITION: We hold state_mutex_ - safe to call should_publish_irk()
    should_publish = self->should_publish_irk(irk_hex, addr_str, should_stop_adv);

    if (should_publish) {
      // Increment counter (read-modify-write race without mutex)
      self->total_captures_++;
      current_captures = self->total_captures_;

      // Check if max captures reached
      if (self->continuous_mode_ && self->max_captures_ > 0 &&
          self->total_captures_ >= self->max_captures_) {
        max_reached = true;
      }
    }
  }  // Release mutex before slow logging/publishing operations

  // Handle auto-stop advertising (outside mutex to avoid deadlock)
  if (should_stop_adv) {
    self->stop_advertising();
    ESP_LOGI(TAG,
             "Auto-stopped advertising due to repeated reconnections. "
             "Toggle 'BLE Advertising' switch to resume.");
  }

  // Skip publishing duplicate (deduplication happened under mutex)
  if (!should_publish) {
    return;
  }

  log_spacer();
  log_banner(context_tag);
  ESP_LOGI(TAG, "Identity Address: %s", addr_str.c_str());
  ESP_LOGI(TAG, "IRK: %s", irk_hex.c_str());
  ESP_LOGI(TAG, "Total captures this session: %u", current_captures);
  if (max_reached) {
    ESP_LOGI(TAG, "Max captures (%u) reached in continuous mode", self->max_captures_);
  }

  log_spacer();
  self->publish_irk_to_sensors(irk_hex, addr_str.c_str());
}

//======================== GATT DB ========================

static uint16_t g_hr_handle;
static uint16_t g_prot_handle;

static struct ble_gatt_chr_def hr_chrs[] = {
  {
      .uuid = &UUID_CHR_HR_MEAS.u,
      .access_cb = chr_read_hr,
      .arg = nullptr,
      // Heart rate characteristic: read requires encryption, notifications supported
      .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ_ENC,
      .val_handle = &g_hr_handle,
  },
  { 0 }
};

static struct ble_gatt_chr_def devinfo_chrs[] = {
  {
      .uuid = &UUID_CHR_MANUF.u,
      .access_cb = chr_read_devinfo,
      .arg = nullptr,  // Will be set to 'this' in register_gatt_services()
      .flags = BLE_GATT_CHR_F_READ_ENC,
  },
  {
      .uuid = &UUID_CHR_MODEL.u,
      .access_cb = chr_read_devinfo,
      .arg = nullptr,  // Will be set to 'this' in register_gatt_services()
      .flags = BLE_GATT_CHR_F_READ_ENC,
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

static struct ble_gatt_chr_def prot_chrs[] = { {
                                                   .uuid = &UUID_CHR_PROT.u,
                                                   .access_cb = chr_read_protected,
                                                   .arg = (void*) "Protected Info",
                                                   .flags = BLE_GATT_CHR_F_READ_ENC,
                                                   .val_handle = &g_prot_handle,
                                               },
                                               { 0 } };

static struct ble_gatt_svc_def gatt_svcs[] = { {
                                                   .type = BLE_GATT_SVC_TYPE_PRIMARY,
                                                   .uuid = &UUID_SVC_HR.u,
                                                   .characteristics = hr_chrs,
                                               },
                                               {
                                                   .type = BLE_GATT_SVC_TYPE_PRIMARY,
                                                   .uuid = &UUID_SVC_DEVINFO.u,
                                                   .characteristics = devinfo_chrs,
                                               },
                                               {
                                                   .type = BLE_GATT_SVC_TYPE_PRIMARY,
                                                   .uuid = &UUID_SVC_BAS.u,
                                                   .characteristics = batt_chrs,
                                               },
                                               {
                                                   .type = BLE_GATT_SVC_TYPE_PRIMARY,
                                                   .uuid = &UUID_SVC_PROT.u,
                                                   .characteristics = prot_chrs,
                                               },
                                               { 0 } };

//======================== Access callbacks ========================

int chr_read_devinfo(uint16_t conn_handle, uint16_t, struct ble_gatt_access_ctxt* ctxt, void* arg) {
  if (!is_encrypted(conn_handle)) return BLE_ATT_ERR_INSUFFICIENT_ENC;

  // THREAD-SAFE: arg points to IRKCaptureComponent instance for both characteristics
  // We determine which characteristic by UUID comparison
  auto* self = static_cast<IRKCaptureComponent*>(arg);

  // Determine which characteristic is being read
  const ble_uuid_t* char_uuid = ctxt->chr->uuid;
  bool is_manufacturer = ble_uuid_cmp(char_uuid, &UUID_CHR_MANUF.u) == 0;

  std::string value_copy;
  {
    MutexGuard lock(self->state_mutex_);
    value_copy = is_manufacturer ? self->manufacturer_name_ : self->ble_name_;
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
int chr_read_protected(uint16_t conn_handle, uint16_t, struct ble_gatt_access_ctxt* ctxt,
                       void* arg) {
  if (!is_encrypted(conn_handle)) return BLE_ATT_ERR_INSUFFICIENT_ENC;
  append_const_string_or_default(ctxt->om, (const char*) arg, "Protected Info");
  return 0;
}

//======================== Store helper ========================

static int read_peer_bond_by_conn(uint16_t conn_handle, struct ble_store_value_sec* out_bond) {
  struct ble_gap_conn_desc desc;
  int rc = ble_gap_conn_find(conn_handle, &desc);
  if (rc != 0) return rc;

  struct ble_store_key_sec key_sec {};
  key_sec.peer_addr = desc.peer_id_addr;

  return ble_store_read_peer_sec(&key_sec, out_bond);
}

//======================== BLE name sanitization ========================

std::string IRKCaptureComponent::sanitize_ble_name(const std::string& name) {
  // Validate and sanitize BLE name for runtime changes from Home Assistant
  std::string sanitized;
  sanitized.reserve(12);  // 12 chars for Samsung S24/S25 compatibility with single-UUID advertising

  // Check for empty string
  if (name.empty()) {
    ESP_LOGW(TAG, "BLE name cannot be empty, using default 'IRK Capture'");
    return "IRK Capture";
  }

  // Sanitize: allow only safe characters (alphanumeric, space, hyphen, underscore)
  for (char c : name) {
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == ' ' ||
        c == '-' || c == '_') {
      sanitized += c;
    } else {
      // Log rejected character
      ESP_LOGW(TAG, "Rejected invalid character in BLE name: 0x%02X ('%c')", (uint8_t) c,
               (c >= 32 && c <= 126) ? c : '?');
    }

    // Enforce 12-byte limit for Samsung S24/S25 compatibility (clean profile with single UUID)
    if (sanitized.length() >= 12) {
      ESP_LOGW(TAG, "BLE name truncated to 12 bytes (Samsung compatibility)");
      break;
    }
  }

  // Final validation: ensure we have at least one character
  if (sanitized.empty()) {
    ESP_LOGW(TAG, "BLE name contained only invalid characters, using default 'IRK Capture'");
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
  // Sanitize and validate user input from Home Assistant
  std::string sanitized = parent_->sanitize_ble_name(value);

  ESP_LOGI(TAG, "BLE name changed to: %s", sanitized.c_str());
  publish_state(sanitized);
  parent_->update_ble_name(sanitized);
}

void IRKCaptureSwitch::write_state(bool state) {
  if (state)
    parent_->start_advertising();
  else
    parent_->stop_advertising();
  publish_state(state);
}

void IRKCaptureButton::press_action() {
  ESP_LOGI(TAG, "Refreshing MAC address...");
  parent_->refresh_mac();
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
int handle_gap_disconnect(IRKCaptureComponent* self, struct ble_gap_event* ev) {
  ESP_LOGI(TAG, "Disconnect reason=%d (0x%02x)", ev->disconnect.reason, ev->disconnect.reason);
  self->on_disconnect();

  // Attempt post-disconnect IRK read using the peer identity address
  struct ble_gap_conn_desc d {};
  if (ble_gap_conn_find(ev->disconnect.conn.conn_handle, &d) == 0) {
    // Thread-safe cache for delayed retry
    {
      MutexGuard lock(self->state_mutex_);
      self->timers_.last_peer_id = d.peer_id_addr;
    }

    struct ble_store_value_sec bond {};
    struct ble_store_key_sec key {};
    key.peer_addr = d.peer_id_addr;
    int rc = ble_store_read_peer_sec(&key, &bond);
    if (rc == BLE_HS_ENOENT) {
      ESP_LOGD(TAG, "No bond for peer (ENOENT)");
    } else if (rc != 0) {
      ESP_LOGW(TAG, "ble_store_read_peer_sec rc=%d", rc);
    } else if (bond.irk_present) {
      std::string irk_hex = bytes_to_hex_rev(bond.irk, sizeof(bond.irk));
      publish_and_log_irk(self, d.peer_id_addr, irk_hex, "DISC_IMMEDIATE");
    } else {
      ESP_LOGD(TAG, "Bond present but no IRK in store post-disconnect");
    }

    // Schedule an extra delayed post-disconnect check (800 ms)
    self->schedule_post_disconnect_check(d.peer_id_addr);
  } else {
    {
      MutexGuard lock(self->state_mutex_);
      memset(&self->timers_.last_peer_id, 0, sizeof(self->timers_.last_peer_id));
    }
    ESP_LOGD(TAG, "Disconnect: conn desc not found; cleared last_peer_id");
  }

  // Thread-safe advertising state update
  {
    MutexGuard lock(self->state_mutex_);
    self->advertising_ = false;
  }

  // Check if we should stop advertising (max captures reached)
  bool should_stop_adv = false;
  if (self->continuous_mode_ && self->max_captures_ > 0 &&
      self->total_captures_ >= self->max_captures_) {
    ESP_LOGI(TAG, "Max captures (%u) reached - stopping advertising", self->max_captures_);
    should_stop_adv = true;
  }

  // Restart advertising logic - THREAD-SAFE: Check suppression flag under mutex
  bool suppressed;
  {
    MutexGuard lock(self->state_mutex_);
    suppressed = self->suppress_next_adv_;
  }

  if (should_stop_adv) {
    // Don't restart - max captures reached
    if (self->advertising_switch_) self->advertising_switch_->publish_state(false);
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
      self->suppress_next_adv_ = false;           // Reset for next time
      self->adv_restart_time_ = now_ms() + 5000;  // Schedule restart
    }

    if (self->advertising_switch_) self->advertising_switch_->publish_state(false);
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

  // Log common encryption failure reasons for debugging (especially Android
  // pairing issues)
  if (ev->enc_change.status != 0) {
    const char* status_desc = "Unknown";
    switch (ev->enc_change.status) {
      case 1:
        status_desc = "Passkey Entry Failed";
        break;
      case 2:
        status_desc = "OOB Not Available";
        break;
      case 3:
        status_desc = "Authentication Requirements";
        break;
      case 4:
        status_desc = "Confirm Value Failed";
        break;
      case 5:
        status_desc = "Pairing Not Supported";
        break;
      case 6:
        status_desc = "Encryption Key Size";
        break;
      case 7:
        status_desc = "Command Not Supported";
        break;
      case 8:
        status_desc = "Unspecified Reason";
        break;
      case 9:
        status_desc = "Repeated Attempts";
        break;
      case 10:
        status_desc = "Invalid Parameters";
        break;
      case 11:
        status_desc = "DHKey Check Failed";
        break;
      case 12:
        status_desc = "Numeric Comparison Failed";
        break;
      case 13:
        status_desc = "BR/EDR Pairing in Progress";
        break;
      case 14:
        status_desc = "Cross-transport Key Derivation";
        break;
      case 1288:
        status_desc = "DHKey Check Failed (NimBLE)";
        break;
    }
    ESP_LOGW(TAG, "ENC_CHANGE failed: %s (status=%d)", status_desc, ev->enc_change.status);
  }

  if (ev->enc_change.status == 0) {
    ESP_LOGI(TAG, "Encryption established; attempting immediate IRK capture");
    self->enc_ready_ = true;
    self->enc_time_ = now_ms();
    self->on_auth_complete(true);

    // Immediate store read using identity address
    struct ble_gap_conn_desc d {};
    if (ble_gap_conn_find(ev->enc_change.conn_handle, &d) == 0) {
      // Thread-safe cache of peer ID for delayed retry
      {
        MutexGuard lock(self->state_mutex_);
        self->timers_.enc_peer_id = d.peer_id_addr;
      }

      struct ble_store_key_sec key {};
      key.peer_addr = d.peer_id_addr;
      struct ble_store_value_sec bond {};
      int rc = ble_store_read_peer_sec(&key, &bond);
      if (rc == BLE_HS_ENOENT) {
        ESP_LOGD(TAG, "No bond for peer yet (ENOENT); scheduling late check");
        self->schedule_late_enc_check(d.peer_id_addr);
      } else if (rc != 0) {
        ESP_LOGW(TAG, "ble_store_read_peer_sec rc=%d; scheduling late check", rc);
        self->schedule_late_enc_check(d.peer_id_addr);
      } else if (bond.irk_present) {
        std::string irk_hex = bytes_to_hex_rev(bond.irk, sizeof(bond.irk));
        publish_and_log_irk(self, d.peer_id_addr, irk_hex, "ENC_CHANGE");
        // Tested working behavior: terminate immediately after successful ENC + IRK capture
        ble_gap_terminate(ev->enc_change.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
      } else {
        ESP_LOGD(TAG, "Bond present but no IRK yet; scheduling late check");
        self->schedule_late_enc_check(d.peer_id_addr);
      }
    }
  } else {
    // Encryption failed - clear all bonds and terminate
    ESP_LOGW(TAG, "ENC_CHANGE failed status=%d; clearing all bonds", ev->enc_change.status);
    ble_store_clear();  // Clear everything to force fresh pairing
    ble_gap_terminate(ev->enc_change.conn_handle, BLE_ERR_REM_USER_CONN_TERM);

    // For DHKey failures (status=1288), stop advertising briefly to force peer to reset
    // THREAD-SAFE: Suppression flag write must be protected
    if (ev->enc_change.status == 1288) {
      ESP_LOGW(TAG, "DHKey failure detected - suppressing advertising to force peer reset");
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
 * @return BLE_GAP_REPEAT_PAIRING_RETRY (instructs NimBLE to delete bond and
 * retry pairing)
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
             "Repeat pairing from %02X:%02X:%02X:%02X:%02X:%02X (clearing all "
             "bond data)",
             peer->val[5], peer->val[4], peer->val[3], peer->val[2], peer->val[1], peer->val[0]);
    // Delete ALL stored data for this peer to allow fresh pairing
    ble_store_util_delete_peer(peer);
  } else {
    ESP_LOGW(TAG, "Repeat pairing: conn desc not found rc=%d", rc);
  }
  return BLE_GAP_REPEAT_PAIRING_RETRY;  // Tell NimBLE to retry pairing with
                                        // clean slate
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

    case 14:  // BLE_GAP_EVENT_L2CAP_UPDATE_REQ
      // Accept connection parameter updates
      return 0;

    case 16:  // BLE_GAP_EVENT_IDENTITY_RESOLVED
      // Identity resolved successfully (IRK working!)
      ESP_LOGD(TAG, "Peer identity resolved using IRK");
      return 0;

    case 18:  // BLE_GAP_EVENT_PHY_UPDATE_COMPLETE
      // PHY layer updated (normal)
      return 0;

    case 27:  // BLE_GAP_EVENT_AUTHORIZE
      // Authorization event (allow by returning 0)
      return 0;

    case 34:  // BLE_GAP_EVENT_SUBRATE_CHANGE
      // BLE 5.2+ subrate change (normal)
      return 0;

    case 38:  // BLE_GAP_EVENT_VS_HCI
      // Vendor-specific HCI event (can ignore)
      return 0;

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
    // Continue anyway - component will work but may have race conditions
  }

  // Clear in-memory IRK cache for fresh session (complements ble_store_clear()
  // which is called later during BLE stack initialization)
  irk_cache_.clear();
  irk_cache_.reserve(
      10);  // Pre-allocate capacity to prevent runtime reallocations/heap fragmentation
  total_captures_ = 0;
  last_publish_time_ = 0;

  this->setup_ble();

  if (start_on_boot_) {
    this->start_advertising();
  }
  if (ble_name_text_) {
    ble_name_text_->publish_state(ble_name_);
  }
}

void IRKCaptureComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "IRK Capture:");
  vTaskDelay(pdMS_TO_TICKS(5));

  // THREAD-SAFE: Copy name and advertising state before logging
  std::string name_copy;
  bool adv_state;
  {
    MutexGuard lock(state_mutex_);
    name_copy = ble_name_;
    adv_state = advertising_;
  }

  ESP_LOGCONFIG(TAG, "  BLE Name: %s", name_copy.c_str());
  vTaskDelay(pdMS_TO_TICKS(5));
  ESP_LOGCONFIG(TAG, "  Advertising: %s", adv_state ? "YES" : "NO");
}

void IRKCaptureComponent::loop() {
  const uint32_t now = now_ms();
  if (now - last_loop_ < TimingConfig::LOOP_MIN_INTERVAL_MS) return;
  last_loop_ = now;

  // Timers for IRK checks
  handle_post_disconnect_timer(now);
  handle_late_enc_timer(now);

  // MAC rotation state machine: handle completion when radio is idle
  // THREAD-SAFE: Check state under mutex
  MacRotationState rotation_state;
  uint16_t conn_handle_copy;
  uint8_t mac_copy[6];
  {
    MutexGuard lock(state_mutex_);
    rotation_state = mac_rotation_state_;
    conn_handle_copy = conn_handle_;
    std::memcpy(mac_copy, pending_mac_, sizeof(mac_copy));
  }

  if (rotation_state == MacRotationState::READY_TO_ROTATE) {
    // Double-check we're truly idle (paranoid safety check)
    if (conn_handle_copy != BLE_HS_CONN_HANDLE_NONE) {
      ESP_LOGW(TAG, "MAC rotation: connection still active (handle=%u), waiting...",
               conn_handle_copy);
      return;  // Wait for disconnect callback to advance state
    }

    ESP_LOGI(TAG, "MAC rotation: radio idle, performing MAC change");

    // Clear all bonds since MAC change invalidates them
    ESP_LOGI(TAG, "Clearing all bond data before MAC refresh");
    ble_store_clear();

    // Reset suppression flags since we're starting fresh with new MAC
    // THREAD-SAFE: Write to shared state under mutex
    {
      MutexGuard lock(state_mutex_);
      suppress_next_adv_ = false;
      adv_restart_time_ = 0;
    }

    // Log previous MAC before change
    log_mac("Previous");

    // Attempt to set the pre-generated MAC address (using local copy to avoid race)
    int rc = ble_hs_id_set_rnd(mac_copy);
    if (rc == 0) {
      ESP_LOGI(TAG, "New MAC (set_rnd): %02X:%02X:%02X:%02X:%02X:%02X", mac_copy[5], mac_copy[4],
               mac_copy[3], mac_copy[2], mac_copy[1], mac_copy[0]);

      // Re-infer own address type for next adv start
      uint8_t own_addr_type;
      int rc2 = ble_hs_id_infer_auto(0, &own_addr_type);
      ESP_LOGI(TAG, "Own address type after set_rnd: %u (0=PUBLIC,1=RANDOM)", own_addr_type);
      if (rc2 != 0) {
        ESP_LOGW(TAG, "ble_hs_id_infer_auto rc=%d after set_rnd", rc2);
      }

      log_mac("Effective");

      // Reset pairing state for next session
      enc_ready_ = false;
      enc_time_ = 0;

      // Advance to completion state - THREAD-SAFE
      {
        MutexGuard lock(state_mutex_);
        mac_rotation_state_ = MacRotationState::ROTATION_COMPLETE;
      }
      ESP_LOGI(TAG, "MAC rotation complete, will restart advertising");

    } else if (rc == BLE_HS_EINVAL) {
      // Host not ready yet - retry on next loop iteration
      ESP_LOGW(TAG, "ble_hs_id_set_rnd EINVAL - host not ready, will retry");
      // Stay in READY_TO_ROTATE state to retry next loop (no state change needed)
    } else {
      ESP_LOGE(TAG, "ble_hs_id_set_rnd failed rc=%d - aborting MAC rotation", rc);
      // Abort rotation - THREAD-SAFE
      {
        MutexGuard lock(state_mutex_);
        mac_rotation_state_ = MacRotationState::IDLE;
      }
      // Restart advertising with old MAC
      start_advertising();
    }
    return;  // Skip rest of loop processing this iteration
  }

  // Restart advertising after successful MAC rotation
  if (rotation_state == MacRotationState::ROTATION_COMPLETE) {
    {
      MutexGuard lock(state_mutex_);
      mac_rotation_state_ = MacRotationState::IDLE;
    }
    ESP_LOGI(TAG, "Restarting advertising with new MAC");
    start_advertising();
    return;  // Skip rest of loop processing this iteration
  }

  // Auto-restart advertising if suppressed and timer expired
  bool should_restart_adv = false;
  {
    MutexGuard lock(state_mutex_);
    if (!advertising_ && adv_restart_time_ != 0 && now >= adv_restart_time_) {
      adv_restart_time_ = 0;
      should_restart_adv = true;
    }
  }
  if (should_restart_adv) {
    ESP_LOGI(TAG, "Auto-restarting advertising after suppression timeout");
    start_advertising();
  }

  // Pairing robustness (single retry)
  retry_security_if_needed(now);

  // Global pairing timeout (90s max) - thread-safe check
  bool should_timeout = false;
  uint16_t timeout_conn_handle;
  {
    MutexGuard lock(state_mutex_);
    if (connected_ && pairing_start_time_ != 0) {
      uint32_t elapsed = now - pairing_start_time_;
      if (elapsed > TimingConfig::PAIRING_TOTAL_TIMEOUT_MS) {
        should_timeout = true;
        timeout_conn_handle = conn_handle_;
        pairing_start_time_ = 0;
      }
    }
  }

  if (should_timeout) {
    ESP_LOGW(TAG, "Pairing timeout after 90+ seconds - resetting connection");
    // Zombie protection: Only terminate if connection still exists
    struct ble_gap_conn_desc d {};
    if (ble_gap_conn_find(timeout_conn_handle, &d) == 0) {
      // Connection still alive - clean up bond and terminate
      ble_store_util_delete_peer(&d.peer_id_addr);
      ble_gap_terminate(timeout_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    } else {
      // Connection already dead (zombie) - just reset local state
      ESP_LOGD(TAG, "Connection already terminated, cleaning up local state");
      MutexGuard lock(state_mutex_);
      connected_ = false;
      conn_handle_ = BLE_HS_CONN_HANDLE_NONE;
    }

    // Cooldown timer: Prevent rapid-fire reconnection loop from failing device
    // Gives "bad" device time to move away or stop attempting connection
    // THREAD-SAFE: Write to shared state under mutex
    {
      MutexGuard lock(state_mutex_);
      adv_restart_time_ = now + TimingConfig::TIMEOUT_COOLDOWN_MS;
    }
    ESP_LOGI(TAG, "Cooldown: advertising will restart in %u seconds",
             TimingConfig::TIMEOUT_COOLDOWN_MS / 1000);
  }

  // THREAD-SAFE: Check connection state before proceeding with connection-specific work
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
  auto err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "NVS full or version mismatch - erasing");
    nvs_flash_erase();
    nvs_flash_init();
  }

  // NVS health check - verify storage actually works
  // All errors are logged; cleanup is guarded to avoid handle leaks
  nvs_handle_t nvs_test_handle;
  err = nvs_open("irk_test", NVS_READWRITE, &nvs_test_handle);
  if (err == ESP_OK) {
    uint32_t test_val = 0xDEADBEEF;
    esp_err_t werr = nvs_set_u32(nvs_test_handle, "test", test_val);
    if (werr == ESP_OK) {
      esp_err_t cerr = nvs_commit(nvs_test_handle);
      if (cerr != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed (err=%d) - bond storage may not work!", cerr);
      } else {
        ESP_LOGI(TAG, "NVS health check passed");
      }
    } else {
      ESP_LOGE(TAG, "NVS set_u32 failed (err=%d) - continuing", werr);
    }
    nvs_close(nvs_test_handle);

    // Clean up test namespace (best-effort, errors logged)
    esp_err_t oerr = nvs_open("irk_test", NVS_READWRITE, &nvs_test_handle);
    if (oerr == ESP_OK) {
      esp_err_t eerr = nvs_erase_all(nvs_test_handle);
      if (eerr != ESP_OK) {
        ESP_LOGW(TAG, "NVS erase_all failed (err=%d) - continuing", eerr);
      } else {
        esp_err_t c2 = nvs_commit(nvs_test_handle);
        if (c2 != ESP_OK) {
          ESP_LOGW(TAG, "NVS commit after erase failed (err=%d) - continuing", c2);
        }
      }
      nvs_close(nvs_test_handle);
    } else {
      ESP_LOGW(TAG, "NVS reopen for cleanup failed (err=%d) - continuing", oerr);
    }
  } else {
    ESP_LOGE(TAG,
             "NVS open failed (err=%d) - IRK capture will fail! Erasing and "
             "retrying...",
             err);
    nvs_flash_erase();
    nvs_flash_init();
  }

  // NimBLE host
  nimble_port_init();

  // Security (set once here; no later re-asserts)
  ble_hs_cfg.reset_cb = [](int reason) { ESP_LOGW(TAG, "NimBLE reset reason=%d", reason); };
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
  };

  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_mitm = 0;
  ble_hs_cfg.sm_sc = 1;                              // Secure Connections enabled
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;  // Just Works pairing (no PIN)
  ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  log_sm_config();

  // Key-value store for bonding/keys
  ble_store_config_init();

  // Clear all bonds on boot for a "clean slate" - prevents bond table from
  // filling up and ensures privacy (no old IRKs persist across reboots or
  // device ownership changes)
  ble_store_clear();
  ESP_LOGI(TAG, "Bond table cleared on boot - fresh pairing session guaranteed");

  // GAP/GATT and name
  ble_svc_gap_init();
  ble_svc_gatt_init();
  ble_svc_gap_device_name_set(ble_name_.c_str());

  // Register services
  this->register_gatt_services();

  // Host task
  nimble_port_freertos_init([](void*) {
    nimble_port_run();
    nimble_port_freertos_deinit();
    vTaskDelete(NULL);
  });

  // Seed initial static random address at boot (matches Arduino BLE library
  // behavior) Why: Provides stable identity before user-triggered refresh_mac
  // rotations
  uint8_t rnd[6];
  esp_fill_random(rnd, sizeof(rnd));
  rnd[0] |= 0xC0;  // static random
  rnd[0] &= 0xFE;
  int rc = ble_hs_id_set_rnd(rnd);
  if (rc == 0) {
    ESP_LOGI(TAG, "Initial MAC (set_rnd): %02X:%02X:%02X:%02X:%02X:%02X", rnd[5], rnd[4], rnd[3],
             rnd[2], rnd[1], rnd[0]);
    log_mac("Effective");
  }

  host_synced_ = true;
}

void IRKCaptureComponent::register_gatt_services() {
  // THREAD-SAFE: Pass 'this' pointer to BOTH DevInfo characteristics
  // Callback will determine which field to read based on UUID
  devinfo_chrs[0].arg = (void*) this;  // Manufacturer Name
  devinfo_chrs[1].arg = (void*) this;  // Model Number

  int rc = ble_gatts_count_cfg(gatt_svcs);
  if (rc == 0) rc = ble_gatts_add_svcs(gatt_svcs);
  if (rc != 0) {
    ESP_LOGE(TAG, "GATT registration failed rc=%d", rc);
    return;
  }
  hr_char_handle_ = g_hr_handle;
  prot_char_handle_ = g_prot_handle;

  ESP_LOGD(TAG, "HR handle=%u, Protected handle=%u", hr_char_handle_, prot_char_handle_);
}

//======================== Advertising ========================

void IRKCaptureComponent::start_advertising() {
  if (!host_synced_) {
    ESP_LOGW(TAG, "Host not synced; cannot advertise");
    return;
  }

  // THREAD-SAFE: Copy BLE name to local variable before using it
  // Prevents data race with update_ble_name() which can reallocate ble_name_
  std::string name_copy;
  {
    MutexGuard lock(state_mutex_);
    name_copy = ble_name_;
  }  // Lock released - safe to call BLE stack functions

  ble_hs_adv_fields fields {};
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.name = (uint8_t*) name_copy.c_str();
  fields.name_len = (uint8_t) name_copy.size();
  fields.name_is_complete = 1;

  // Appearance: Heart Rate Sensor
  fields.appearance = APPEARANCE_HEART_RATE_SENSOR;
  fields.appearance_is_present = 1;

  // Advertise ONLY Heart Rate service for Samsung S24/S25 compatibility
  // Battery and Device Info services remain available in GATT server after connection
  static ble_uuid16_t hr_uuid = BLE_UUID16_INIT(UUID_SVC_HEART_RATE);
  fields.uuids16 = &hr_uuid;
  fields.num_uuids16 = 1;  // Single service = clean "fitness device" profile
  fields.uuids16_is_complete = 1;

  int rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_adv_set_fields rc=%d", rc);
    return;
  }

  ble_gap_adv_params advp {};
  advp.conn_mode = BLE_GAP_CONN_MODE_UND;
  advp.disc_mode = BLE_GAP_DISC_MODE_GEN;

  uint8_t own_addr_type;
  rc = ble_hs_id_infer_auto(0, &own_addr_type);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_hs_id_infer_auto rc=%d", rc);
    return;
  }

  rc = ble_gap_adv_start(own_addr_type, nullptr, BLE_HS_FOREVER, &advp,
                         IRKCaptureComponent::gap_event_handler, this);

  // Thread-safe state update (mutex released before BLE stack call above)
  bool adv_success;
  {
    MutexGuard lock(state_mutex_);
    advertising_ = (rc == 0);
    adv_success = advertising_;
  }

  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_adv_start rc=%d", rc);
  } else {
    // UI update outside lock (publish_state is internally thread-safe)
    if (advertising_switch_) advertising_switch_->publish_state(true);
    ESP_LOGD(TAG, "Advertising as '%s' (Heart Rate Sensor)", ble_name_.c_str());
  }
}

void IRKCaptureComponent::stop_advertising() {
  ble_gap_adv_stop();

  // Thread-safe state update (after BLE stack call)
  {
    MutexGuard lock(state_mutex_);
    advertising_ = false;
  }

  // UI update outside lock (publish_state is internally thread-safe)
  if (advertising_switch_) advertising_switch_->publish_state(false);
  ESP_LOGD(TAG, "Advertising stopped");
}

bool IRKCaptureComponent::is_advertising() {
  MutexGuard lock(state_mutex_);
  return advertising_;
}

//======================== MAC refresh (event-driven, non-blocking) ========================

void IRKCaptureComponent::refresh_mac() {
  ESP_LOGI(TAG, "MAC rotation requested (non-blocking event-driven)");

  // Pre-generate the new MAC address before taking mutex (esp_fill_random is slow)
  uint8_t temp_mac[6];
  bool mac_generated = false;
  for (int tries = 0; tries < 10; ++tries) {
    esp_fill_random(temp_mac, sizeof(temp_mac));
    // Force static-random: top two bits 11, LSB bit0 = 0 (unicast)
    temp_mac[0] |= 0xC0;
    temp_mac[0] &= 0xFE;

    // Validate: reject all-zero MAC
    bool all_zero = true;
    for (int i = 0; i < 6; ++i) {
      if (temp_mac[i] != 0x00) {
        all_zero = false;
        break;
      }
    }
    if (!all_zero) {
      ESP_LOGD(TAG, "Pre-generated MAC: %02X:%02X:%02X:%02X:%02X:%02X", temp_mac[5], temp_mac[4],
               temp_mac[3], temp_mac[2], temp_mac[1], temp_mac[0]);
      mac_generated = true;
      break;
    }
  }

  // THREAD-SAFE: Atomically update state machine and pending MAC buffer
  bool should_stop_adv;
  uint16_t conn_handle_copy;
  {
    MutexGuard lock(state_mutex_);

    // Set rotation state and commit pre-generated MAC to shared buffer
    mac_rotation_state_ = MacRotationState::REQUESTED;
    if (mac_generated) {
      std::memcpy(pending_mac_, temp_mac, sizeof(pending_mac_));
    }

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
    ble_gap_terminate(conn_handle_copy, BLE_ERR_REM_USER_CONN_TERM);
    // on_disconnect() will advance the state machine to READY_TO_ROTATE
  } else {
    // No connection - safe to rotate immediately
    ESP_LOGD(TAG, "No active connection, ready to rotate MAC");
    MutexGuard lock(state_mutex_);
    mac_rotation_state_ = MacRotationState::READY_TO_ROTATE;
    // loop() will handle the actual rotation
  }
}

//======================== BLE name update ========================

void IRKCaptureComponent::update_ble_name(const std::string& name) {
  // THREAD-SAFE: Update name with mutex protection since GATT callback may read it
  {
    MutexGuard lock(state_mutex_);
    ble_name_ = name;
  }

  // Update GAP device name
  ble_svc_gap_device_name_set(name.c_str());

  // NOTE: devinfo_chrs[1].arg already points to 'this' (set in register_gatt_services)
  // GATT callback will read updated ble_name_ with mutex protection

  // Initiate non-blocking MAC rotation (event-driven state machine)
  // Sequence: refresh_mac() → on_disconnect() → loop() → start_advertising()
  // The advertising will restart automatically with the new name after MAC rotation completes
  this->refresh_mac();
}

//======================== GAP helpers ========================

void IRKCaptureComponent::on_connect(uint16_t conn_handle) {
  // Thread-safe connection state update
  {
    MutexGuard lock(state_mutex_);
    conn_handle_ = conn_handle;
    connected_ = true;
    pairing_start_time_ = now_ms();
  }

  enc_ready_ = false;
  enc_time_ = 0;

  // Reset loop-helper state (single retry model)
  sec_retry_done_ = false;
  sec_init_time_ms_ = 0;
  irk_gave_up_ = false;
  irk_last_try_ms_ = 0;

  // THREAD-SAFE: Copy state for logging (avoid holding mutex during UART logging)
  bool adv_state;
  {
    MutexGuard lock(state_mutex_);
    adv_state = advertising_;
  }

  // Compact summary to increase chance at least one key line survives under log
  // pressure
  ESP_LOGI(TAG, "Conn start: handle=%u enc_ready=%d adv=%d", conn_handle, (int) enc_ready_,
           (int) adv_state);

  ESP_LOGI(TAG, "Connected; handle=%u, initiating security", conn_handle);
  log_conn_desc(conn_handle);
  log_sm_config();

  // Cache peer identity address for delayed post-disconnect checks
  struct ble_gap_conn_desc d;
  if (ble_gap_conn_find(conn_handle, &d) == 0) {
    // Thread-safe peer ID snapshot
    {
      MutexGuard lock(state_mutex_);
      timers_.last_peer_id = d.peer_id_addr;
    }

    // Check for bond state mismatch: peer thinks it's unbonded but we have bond
    // data
    if (!d.sec_state.bonded) {
      struct ble_store_key_sec key {};
      key.peer_addr = d.peer_id_addr;
      struct ble_store_value_sec bond {};
      int rc = ble_store_read_peer_sec(&key, &bond);
      if (rc == BLE_HS_ENOENT) {
        ESP_LOGD(TAG, "Peer unbonded and no cached bond (ENOENT) - will pair fresh");
      } else if (rc != 0) {
        ESP_LOGW(TAG, "ble_store_read_peer_sec rc=%d during bond mismatch check", rc);
      } else if (bond.irk_present) {
        // Re-publish the IRK we already have
        std::string irk_hex = bytes_to_hex_rev(bond.irk, sizeof(bond.irk));
        char addr_str[18];
        snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X", d.peer_id_addr.val[5],
                 d.peer_id_addr.val[4], d.peer_id_addr.val[3], d.peer_id_addr.val[2],
                 d.peer_id_addr.val[1], d.peer_id_addr.val[0]);
        publish_irk_to_sensors(irk_hex, addr_str);
        ESP_LOGI(TAG, "Re-published existing IRK for already-paired device");
        // Set flag to prevent immediate re-advertising (break the reconnect loop)
        // THREAD-SAFE: Suppression flag write must be protected
        {
          MutexGuard lock(state_mutex_);
          suppress_next_adv_ = true;
        }
        // Disconnect since we already have what we need
        // THREAD-SAFE: Use function parameter (already set under mutex at line 1771)
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;  // Don't initiate security, we're done
      } else {
        // No IRK in bond - delete and try fresh pairing
        ESP_LOGW(TAG,
                 "Peer unbonded but we have cached bond without IRK. Clearing "
                 "to force fresh pairing.");
        ble_store_util_delete_peer(&d.peer_id_addr);
      }
    }
  } else {
    {
      MutexGuard lock(state_mutex_);
      memset(&timers_.last_peer_id, 0, sizeof(timers_.last_peer_id));
    }
    ESP_LOGD(TAG, "on_connect: ble_gap_conn_find failed; cleared cached last_peer_id");
  }

  // Proactively initiate pairing; peer should show pairing dialog now
  // THREAD-SAFE: Use function parameter (already set under mutex at line 1771)
  int rc = ble_gap_security_initiate(conn_handle);
  if (rc == BLE_HS_EBUSY) {
    // Peer is already initiating security - skip our retry to avoid conflicts
    ESP_LOGD(TAG, "Peer already initiating security (EBUSY); skipping retry");
    sec_retry_done_ = true;  // Skip the 2-second retry since peer is handling it
  } else if (rc != 0 && rc != BLE_HS_EALREADY) {
    ESP_LOGW(TAG, "ble_gap_security_initiate rc=%d", rc);
  }
}

void IRKCaptureComponent::on_disconnect() {
  // Thread-safe disconnection state update and MAC rotation handoff
  {
    MutexGuard lock(state_mutex_);
    connected_ = false;
    conn_handle_ = BLE_HS_CONN_HANDLE_NONE;
    pairing_start_time_ = 0;

    // MAC rotation handoff: advance state if rotation is pending (CRITICAL: must be inside mutex)
    // Now that we're disconnected, the radio is idle and safe to rotate
    if (mac_rotation_state_ == MacRotationState::REQUESTED) {
      ESP_LOGD(TAG, "MAC rotation: connection closed, advancing to READY_TO_ROTATE");
      mac_rotation_state_ = MacRotationState::READY_TO_ROTATE;
      // loop() will perform the actual MAC rotation on next iteration
    }
  }

  enc_ready_ = false;
  enc_time_ = 0;

  // Reset loop-helper state
  sec_retry_done_ = false;
  sec_init_time_ms_ = 0;
  irk_gave_up_ = false;
  irk_last_try_ms_ = 0;

  ESP_LOGI(TAG, "Disconnected");
}

void IRKCaptureComponent::on_auth_complete(bool) {
  // IRK retrieved in loop via try_get_irk
}

//======================== IRK extraction ========================

bool IRKCaptureComponent::try_get_irk(uint16_t conn_handle, uint8_t irk_out[16],
                                      ble_addr_t& peer_id_out) {
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
    ESP_LOGW(TAG, "Bond present but no IRK (peer did not distribute ID key)");
    ESP_LOGW(TAG, "Peer key distribution may be incomplete or unsupported by device");
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

void IRKCaptureComponent::schedule_post_disconnect_check(const ble_addr_t& peer_id) {
  MutexGuard lock(state_mutex_);
  timers_.last_peer_id = peer_id;
  timers_.post_disc_due_ms = now_ms() + TimingConfig::POST_DISC_DELAY_MS;
}

void IRKCaptureComponent::schedule_late_enc_check(const ble_addr_t& peer_id) {
  MutexGuard lock(state_mutex_);
  timers_.enc_peer_id = peer_id;
  timers_.late_enc_due_ms = now_ms() + TimingConfig::ENC_LATE_READ_DELAY_MS;
}

void IRKCaptureComponent::handle_post_disconnect_timer(uint32_t now) {
  // Thread-safe timer check and peer ID snapshot
  ble_addr_t peer_id;
  {
    MutexGuard lock(state_mutex_);
    if (!timers_.post_disc_due_ms || now < timers_.post_disc_due_ms) return;
    timers_.post_disc_due_ms = 0;  // consume
    peer_id = timers_.last_peer_id;
  }

  if (addr_is_zero(peer_id)) {
    ESP_LOGD(TAG, "Post-disc timer fired but last_peer_id is zero (skipping)");
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
  } else if (bond.irk_present) {
    std::string irk_hex = bytes_to_hex_rev(bond.irk, sizeof(bond.irk));
    publish_and_log_irk(this, peer_id, irk_hex, "DISC_DELAYED");
  } else {
    ESP_LOGD(TAG, "Bond present but no IRK - post-disc delayed check");
  }
}

void IRKCaptureComponent::handle_late_enc_timer(uint32_t now) {
  // Thread-safe timer check and peer ID snapshot
  ble_addr_t peer_id;
  {
    MutexGuard lock(state_mutex_);
    if (!timers_.late_enc_due_ms || now < timers_.late_enc_due_ms) return;
    timers_.late_enc_due_ms = 0;
    peer_id = timers_.enc_peer_id;
  }

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
  } else if (bond.irk_present) {
    std::string irk_hex = bytes_to_hex_rev(bond.irk, sizeof(bond.irk));
    publish_and_log_irk(this, peer_id, irk_hex, "ENC_LATE");

    // Tested working behavior: terminate after late capture if still connected
    // THREAD-SAFE: Check connection state before terminating
    bool is_connected;
    uint16_t conn_handle_copy;
    {
      MutexGuard lock(state_mutex_);
      is_connected = connected_;
      conn_handle_copy = conn_handle_;
    }
    if (is_connected && conn_handle_copy != BLE_HS_CONN_HANDLE_NONE) {
      ble_gap_terminate(conn_handle_copy, BLE_ERR_REM_USER_CONN_TERM);
    }
  } else {
    ESP_LOGD(TAG, "Bond present but no IRK - late ENC check");
  }
}

//======================== Loop helpers ========================

void IRKCaptureComponent::retry_security_if_needed(uint32_t now) {
  // THREAD-SAFE: Check connection state first
  bool is_connected;
  uint16_t conn_handle_copy;
  {
    MutexGuard lock(state_mutex_);
    is_connected = connected_;
    conn_handle_copy = conn_handle_;
  }

  // Tested working behavior: single retry after SEC_RETRY_DELAY_MS from initial initiate
  if (is_connected && !enc_ready_) {
    if (sec_init_time_ms_ == 0) sec_init_time_ms_ = now;

    // Retry security after configured delay
    if (!sec_retry_done_ && (now - sec_init_time_ms_) > TimingConfig::SEC_RETRY_DELAY_MS) {
      uint32_t elapsed = now - sec_init_time_ms_;
      ESP_LOGI(TAG, "Retrying security initiate after %u ms", elapsed);
      int rc = ble_gap_security_initiate(conn_handle_copy);
      ESP_LOGW(TAG, "Retry security initiate rc=%d", rc);
      sec_retry_done_ = true;
    }

    // If encryption still hasn't completed after timeout, assume peer forgot
    // pairing Delete our bond and disconnect to force fresh pairing on
    // reconnect
    if ((now - sec_init_time_ms_) > TimingConfig::SEC_TIMEOUT_MS) {
      struct ble_gap_conn_desc d {};
      if (ble_gap_conn_find(conn_handle_copy, &d) == 0) {
        ESP_LOGW(TAG,
                 "Encryption timeout after %u ms; clearing bond for %s to "
                 "force fresh pairing.",
                 TimingConfig::SEC_TIMEOUT_MS, addr_to_str(d.peer_id_addr).c_str());
        ble_store_util_delete_peer(&d.peer_id_addr);
      } else {
        ESP_LOGW(TAG,
                 "Encryption timeout after %u ms; conn desc not found during "
                 "cleanup.",
                 TimingConfig::SEC_TIMEOUT_MS);
      }
      // Disconnect to trigger fresh pairing on next connection
      // CRITICAL: If termination fails, the NimBLE stack may be wedged - force local cleanup
      // to prevent "zombie connection" state where UI shows connected but no data flows
      int rc = ble_gap_terminate(conn_handle_copy, BLE_ERR_REM_USER_CONN_TERM);
      if (rc != 0) {
        ESP_LOGE(TAG,
                 "CRITICAL: ble_gap_terminate failed (rc=%d) - NimBLE stack may be wedged. "
                 "Forcing local state reset to prevent zombie connection.",
                 rc);
        // Force local cleanup so we don't stay stuck in CONNECTED state forever
        MutexGuard lock(state_mutex_);
        connected_ = false;
        conn_handle_ = BLE_HS_CONN_HANDLE_NONE;
      }
    }
  } else if (!is_connected) {
    sec_retry_done_ = false;
    sec_init_time_ms_ = 0;
  }
}

void IRKCaptureComponent::notify_hr_if_due(uint32_t now) {
  if (hr_char_handle_ == 0) return;
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
  int rc = ble_gatts_notify_custom(conn_handle_copy, hr_char_handle_, om);
  if (rc != 0) {
    ESP_LOGD(TAG, "notify rc=%d", rc);
  }
}

void IRKCaptureComponent::poll_irk_if_due(uint32_t now) {
  // Attempt IRK retrieval after encryption + delay (allow store write)
  if (!enc_ready_ || irk_gave_up_) return;
  if ((now - enc_time_) < TimingConfig::ENC_TO_FIRST_TRY_DELAY_MS) return;
  if ((now - irk_last_try_ms_) < TimingConfig::ENC_TRY_INTERVAL_MS) return;

  irk_last_try_ms_ = now;

  // THREAD-SAFE: Copy connection handle before BLE stack calls
  uint16_t conn_handle_copy;
  {
    MutexGuard lock(state_mutex_);
    conn_handle_copy = conn_handle_;
  }

  uint8_t irk_bytes[16];
  ble_addr_t peer_id;
  if (try_get_irk(conn_handle_copy, irk_bytes, peer_id)) {
    std::string irk_hex = bytes_to_hex_rev(irk_bytes, sizeof(irk_bytes));
    publish_and_log_irk(this, peer_id, irk_hex, "POLL_CONNECTED");
    ble_gap_terminate(conn_handle_copy, BLE_ERR_REM_USER_CONN_TERM);
    irk_gave_up_ = true;
  } else {
    if ((now - enc_time_) > TimingConfig::ENC_GIVE_UP_AFTER_MS) {
      ESP_LOGW(TAG, "IRK not found after %u ms post-encryption",
               TimingConfig::ENC_GIVE_UP_AFTER_MS);
      irk_gave_up_ = true;
    }
  }
}

//======================== Public publish utility ========================

void IRKCaptureComponent::publish_irk_to_sensors(const std::string& irk_hex, const char* addr_str) {
  if (irk_sensor_) irk_sensor_->publish_state(irk_hex);
  if (address_sensor_) address_sensor_->publish_state(addr_str);
}

}  // namespace irk_capture
}  // namespace esphome

#endif  // USE_ESP32
