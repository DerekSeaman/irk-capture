#pragma once

#include <array>
#include <atomic>
#include <string>
#include <vector>

#include "esphome/components/button/button.h"
#include "esphome/components/select/select.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text/text.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "identity_name.h"

// ESP32-only component - requires Bluetooth hardware
#ifdef USE_ESP32
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_hs.h>
#include <host/ble_store.h>
#include <host/ble_uuid.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>

#include "rpa_check.h"
#else
#error \
    "IRK Capture component requires ESP32 platform with Bluetooth support (ESP32/ESP32-C3/ESP32-S3/ESP32-C6)"
#endif

namespace esphome {
namespace irk_capture {

class IRKCaptureComponent;

// Latest history/bond-clear request. A terminal result remains available until
// another request starts so HTTP clients can observe asynchronous completion.
enum class BondClearState : uint8_t { IDLE, PENDING, BUSY, FAILED, CANCELLED, SUCCEEDED };

// The name a state has in the wizard's JSON. The wizard page compares these
// strings, so they are part of its HTTP contract.
inline const char* bond_clear_state_name(BondClearState state) {
  switch (state) {
    case BondClearState::PENDING:
      return "pending";
    case BondClearState::BUSY:
      return "busy";
    case BondClearState::FAILED:
      return "failed";
    case BondClearState::CANCELLED:
      return "cancelled";
    case BondClearState::SUCCEEDED:
      return "succeeded";
    case BondClearState::IDLE:
      break;
  }
  return "idle";
}

struct BondClearResult {
  uint32_t operation_id { 0 };
  BondClearState state { BondClearState::IDLE };
  std::string reason;
  bool history_cleared { false };
};

// Outcome of the latest address rotation (Refresh BLE Identity, or the one a
// name change triggers). A rotation runs after its request has returned, so
// this is the only way a caller learns whether it took effect.
enum class IdentityRefreshState : uint8_t { IDLE, PENDING, SUCCEEDED, FAILED, PARTIAL };

// The name a state has in the wizard's JSON; part of its HTTP contract.
inline const char* identity_refresh_state_name(IdentityRefreshState state) {
  switch (state) {
    case IdentityRefreshState::PENDING:
      return "pending";
    case IdentityRefreshState::SUCCEEDED:
      return "succeeded";
    case IdentityRefreshState::FAILED:
      return "failed";
    case IdentityRefreshState::PARTIAL:
      return "partial";
    case IdentityRefreshState::IDLE:
      break;
  }
  return "idle";
}

struct IdentityRefreshResult {
  uint32_t operation_id { 0 };
  IdentityRefreshState state { IdentityRefreshState::IDLE };
  std::string cleanup_reason;
};

// BLE advertising profile options
enum class BLEProfile : uint8_t {
  HEART_SENSOR = 0,  // Heart Rate Sensor
  KEYBOARD = 1,      // Logitech K380 Keyboard
};

// Select for BLE profile
class IRKCaptureSelect : public select::Select {
 public:
  void set_parent(IRKCaptureComponent* parent) {
    parent_ = parent;
  }
  void control(const std::string& value) override;

 protected:
  IRKCaptureComponent* parent_ { nullptr };
};

// Text input for BLE name
class IRKCaptureText : public text::Text {
 public:
  void set_parent(IRKCaptureComponent* parent) {
    parent_ = parent;
  }
  void control(const std::string& value) override;

 protected:
  IRKCaptureComponent* parent_ { nullptr };
};

// Text sensor for IRK Capture outputs (IRK, addresses, check, status)
class IRKCaptureTextSensor : public text_sensor::TextSensor {};

// Switch for advertising control
class IRKCaptureSwitch : public switch_::Switch {
 public:
  void set_parent(IRKCaptureComponent* parent) {
    parent_ = parent;
  }
  void write_state(bool state) override;

 protected:
  IRKCaptureComponent* parent_ { nullptr };
};

// Button to refresh BLE identity (rotate MAC and adopt a generated name)
class IRKCaptureButton : public button::Button {
 public:
  void set_parent(IRKCaptureComponent* parent) {
    parent_ = parent;
  }
  void press_action() override;

 protected:
  IRKCaptureComponent* parent_ { nullptr };
};

// Button that clears capture history and requests a bond-store wipe without
// rotating the advertised address.
class IRKCaptureForgetBondsButton : public button::Button {
 public:
  void set_parent(IRKCaptureComponent* parent) {
    parent_ = parent;
  }
  void press_action() override;

 protected:
  IRKCaptureComponent* parent_ { nullptr };
};

// Switch: stop advertising automatically once a capture publishes, instead of
// continuing to hunt for more unique devices this session.
class IRKCaptureStopAfterCaptureSwitch : public switch_::Switch {
 public:
  void set_parent(IRKCaptureComponent* parent) {
    parent_ = parent;
  }
  void write_state(bool state) override;

 protected:
  IRKCaptureComponent* parent_ { nullptr };
};

// Text input: consume-once label for the next device added or restored to visible history.
class IRKCaptureLabelText : public text::Text {
 public:
  void set_parent(IRKCaptureComponent* parent) {
    parent_ = parent;
  }
  void control(const std::string& value) override;

 protected:
  IRKCaptureComponent* parent_ { nullptr };
};

// Copy this provenance with each store read and fallback timer. A later
// connection must not change whether an older result came from a re-pair.
enum class CaptureOrigin : uint8_t { UNKNOWN, FRESH, BONDED, REPAIR };

// What a stored bond holds for a peer; see read_peer_irk_().
enum class BondRead : uint8_t { NO_BOND, READ_ERROR, IRK, NO_IRK, INVALID_IRK };

// Outcome of set_next_capture_label(). stored_label is the label now held:
// the sanitized value when accepted, the unchanged label when refused.
struct LabelUpdateResult {
  bool accepted { false };
  std::string stored_label;
};

// Last successful genuine capture, retained after transient status and entity
// updates. boot_id distinguishes a reboot from sequence rollover.
struct CaptureResult {
  uint32_t boot_id { 0 };
  uint32_t sequence { 0 };
  std::string irk;
  std::string device_mac;
  IrkCheck verification { IrkCheck::UNCHECKED };
  // The connection that produced this result. In the baseline begin_capture()
  // returns, it is instead the newest connection when the capture started: a
  // result from that connection or an earlier one belongs to an earlier run.
  uint32_t connection_generation { 0 };
};

// What record_observation_locked_() decided and why. Filled in under state_mutex_ and
// logged by the caller once the lock is released.
struct PublishDecision {
  enum class Reason : uint8_t {
    DUPLICATE_PATH,    // Another read path already handled this connection
    STALE_CONNECTION,  // Delayed read from a connection older than the latest seen
    KEY_CHANGED,       // A known identity presented a different IRK
    PAIRED_AGAIN,      // Explicit pairing or re-pair with the same IRK
    RECONNECT_LIMIT,   // A bonded peer keeps reconnecting
    RATE_LIMITED,      // Bonded reconnect inside the republish interval
    RECONNECT,         // Bonded reconnect that is republished
    NEW_DEVICE,        // First capture of this identity
  };
  Reason reason { Reason::DUPLICATE_PATH };
  bool publish { false };
  bool stop_adv { false };            // Caller should hold advertising off
  bool limit_just_reached { false };  // True only on the first over-limit reconnect
  bool evicted_oldest { false };      // The cache was full; its oldest entry was dropped
  uint16_t reconnect_count { 0 };
  uint32_t since_published_ms { 0 };
  size_t cache_size { 0 };
  size_t cache_limit { 0 };
  std::string label;  // Label this publication consumed, if any
};

// The session values a status page shows, copied under a single lock.
struct SessionState {
  // Left empty, with history_changed false, when the caller already holds
  // the history for history_version.
  std::string history_json { "[]" };
  uint32_t history_version { 0 };
  bool history_changed { true };
  std::string next_capture_label;
  std::string advertised_name;
  BLEProfile profile { BLEProfile::KEYBOARD };
  bool advertising_requested { false };
  bool stop_after_capture { false };
  CaptureResult capture_result;
  BondClearResult bond_clear;
  IdentityRefreshResult identity_refresh;
};

// The controls a status request overlays on the wizard's periodic snapshot,
// read under one lock so they agree with each other.
struct LiveControls {
  std::string next_capture_label;
  BLEProfile profile { BLEProfile::KEYBOARD };
  bool advertising_requested { false };
  bool stop_after_capture { false };
};

// GAP event handlers (irk_capture_gap.cpp) and IRK output helper (irk_capture_history.cpp).
int handle_gap_connect(class IRKCaptureComponent* self, struct ble_gap_event* ev);
int handle_gap_disconnect(class IRKCaptureComponent* self, struct ble_gap_event* ev);
int handle_gap_enc_change(class IRKCaptureComponent* self, struct ble_gap_event* ev);
int handle_gap_repeat_pairing(class IRKCaptureComponent* self, struct ble_gap_event* ev);
void publish_and_log_irk(class IRKCaptureComponent* self, const ble_addr_t& peer_id_addr,
                         const std::string& irk_hex, const char* context_tag,
                         uint32_t connection_generation,
                         CaptureOrigin origin = CaptureOrigin::UNKNOWN,
                         IrkCheck check = IrkCheck::UNCHECKED, bool host_task = false);

// Main component
class IRKCaptureComponent : public Component {
 public:
  // Component API
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override {
    return -200.0f;
  }
  // Arguments: IRK, identity MAC, and irk_check_name() of the address check.
  void add_on_capture_trigger(Trigger<std::string, std::string, std::string>* trigger) {
    capture_triggers_.push_back(trigger);
  }

  // GAP event handlers and IRK output helper.
  friend int handle_gap_connect(IRKCaptureComponent* self, struct ble_gap_event* ev);
  friend int handle_gap_disconnect(IRKCaptureComponent* self, struct ble_gap_event* ev);
  friend int handle_gap_enc_change(IRKCaptureComponent* self, struct ble_gap_event* ev);
  friend int handle_gap_repeat_pairing(IRKCaptureComponent* self, struct ble_gap_event* ev);
  friend void publish_and_log_irk(IRKCaptureComponent* self, const ble_addr_t& peer_id_addr,
                                  const std::string& irk_hex, const char* context_tag,
                                  uint32_t connection_generation, CaptureOrigin origin,
                                  IrkCheck check, bool host_task);

  // The Device Information read callback copies names under state_mutex_.
  friend int chr_read_devinfo(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt* ctxt, void* arg);

  // Configuration setters
  void set_ble_name(const std::string& name) {
    ble_name_ = name;
  }
  void set_start_on_boot(bool start) {
    advertising_requested_ = start;
  }
  void set_continuous_mode(bool enable) {
    continuous_mode_ = enable;
  }
  void set_max_captures(uint8_t max) {
    max_captures_ = max;
  }
  void set_irk_sensor(text_sensor::TextSensor* sensor) {
    irk_sensor_ = sensor;
  }
  void set_address_sensor(text_sensor::TextSensor* sensor) {
    address_sensor_ = sensor;
  }
  // irk_check_name() of the latest result: verified, unverified or mismatch.
  void set_irk_check_sensor(text_sensor::TextSensor* sensor) {
    irk_check_sensor_ = sensor;
  }
  void set_effective_mac_sensor(text_sensor::TextSensor* sensor) {
    effective_mac_sensor_ = sensor;
  }
  void set_advertising_switch(IRKCaptureSwitch* sw) {
    advertising_switch_ = sw;
    if (sw) sw->set_parent(this);
  }
  void set_refresh_identity_button(IRKCaptureButton* btn) {
    refresh_identity_button_ = btn;
    if (btn) btn->set_parent(this);
  }
  void set_ble_name_text(IRKCaptureText* txt) {
    ble_name_text_ = txt;
    if (txt) txt->set_parent(this);
  }
  void set_ble_profile_select(IRKCaptureSelect* sel) {
    ble_profile_select_ = sel;
    if (sel) sel->set_parent(this);
  }
  void set_status_sensor(text_sensor::TextSensor* sensor) {
    status_sensor_ = sensor;
  }
  void set_forget_bonds_button(IRKCaptureForgetBondsButton* btn) {
    forget_bonds_button_ = btn;
    if (btn) btn->set_parent(this);
  }
  void set_stop_after_capture_switch(IRKCaptureStopAfterCaptureSwitch* sw) {
    stop_after_capture_switch_ = sw;
    if (sw) sw->set_parent(this);
  }
  void set_next_capture_label_text(IRKCaptureLabelText* txt) {
    next_capture_label_text_ = txt;
    if (txt) txt->set_parent(this);
  }

  // Profile management
  void set_ble_profile(BLEProfile profile);
  BLEProfile get_ble_profile();

  // Public actions
  // Returns false when the device rejected the change (name left unchanged).
  bool update_ble_name(const std::string& name);
  std::string get_ble_name();
  // host_task: the caller is the NimBLE host task, which never waits for
  // ble_op_mutex_. A busy lock then hands the start to loop() through
  // adv_restart_time_, keeping any hold-off deadline already set.
  void start_advertising(bool host_task = false);
  // True only after verifying that GAP advertising is inactive under the BLE lock.
  // host_task as for start_advertising(). hold_off is the adv_restart_time_
  // deadline of the hold-off this stop serves, or 0 for a plain stop. Such a
  // stop is made only while that hold-off is still pending (see
  // hold_off_pending_locked_()), so it cannot turn off a newer start.
  bool stop_advertising(bool host_task = false, uint32_t hold_off = 0);
  void set_advertising_requested(bool requested);
  CaptureResult get_capture_result();
  // Main-task wizard action: claim a baseline and request advertising in the
  // same state transaction, then let the normal BLE start gates run. It is
  // arm_capture() followed by start_advertising().
  CaptureResult begin_capture();
  // The state transaction alone: takes only state_mutex_ and makes no BLE
  // call, so a caller may hold its own lock across it (the wizard holds
  // snapshot_mutex_, giving the order snapshot_mutex_ -> state_mutex_).
  // Follow it with start_advertising() once that lock is released.
  CaptureResult arm_capture();
  // Wizard cancellation can run on the HTTP task. BLE termination is queued
  // on the host task and the wizard defers the advertising stop to the main task.
  void cancel_capture();
  // Rotates the address and adopts a name carrying the profile and the new
  // address's low two octets ("IRK HR 7F3A", "IRK KB 7F3A") until the next
  // reboot. Both halves change together because rotating the address alone
  // leaves the device invisible to a phone that has cached the old name.
  void refresh_identity();
  bool is_advertising();            // Thread-safe check of actual advertising state
  bool is_advertising_requested();  // Thread-safe check of the user's desired state
  void on_ble_host_synced();        // Called when NimBLE host is ready

  // GAP events
  void on_connect(uint16_t conn_handle);
  bool on_disconnect(uint16_t conn_handle);
  // GAP event handler (static trampoline)
  static int gap_event_handler(struct ble_gap_event* event, void* arg);

  // Sensor publishing helper
  void publish_irk_to_sensors(const std::string& irk_hex, const std::string& addr_str,
                              uint32_t connection_generation = 0, bool capture_event = false,
                              bool stop_after_result = false, IrkCheck check = IrkCheck::UNCHECKED);
  void publish_effective_mac();

  // Wizard-facing runtime controls and session snapshots.
  void set_stop_after_capture(bool enabled);
  bool get_stop_after_capture();
  std::string get_next_capture_label();
  // Longest capture label kept; text.py gives the Home Assistant entity the
  // same limit.
  static constexpr size_t CAPTURE_LABEL_MAX_LEN = 24;
  // Sanitizes, trims, stores, and returns the label for the next device added
  // or restored to visible history (consumed once). Keeps [A-Za-z0-9 _-'] up
  // to CAPTURE_LABEL_MAX_LEN chars; "" clears it; unusable/space-only text is
  // rejected and the current label kept.
  LabelUpdateResult set_next_capture_label(const std::string& value);
  // Clears visible history, retaining deduplication/capture accounting. The
  // NimBLE task clears stored bonds only when no peer is connected.
  void forget_all_bonds();
  BondClearResult request_bond_clear();
  BondClearResult get_bond_clear_result();
  // Session captures as JSON [{mac, irk, label, reconnects, verification}].
  // Served over HTTP rather than an HA entity (HA caps entity states at 255 chars).
  std::string build_history_json();
  // Thread-safe snapshot for a status page, including visible capture history
  // JSON. Pass the history_version of the last state taken to skip copying and
  // reformatting a history that has not changed since; 0 always builds it.
  SessionState get_session_state(uint32_t known_history_version = 0);
  LiveControls get_live_controls();

 protected:
  // Configuration/state
  std::string ble_name_ { DEFAULT_BLE_NAME };
  // Set by refresh_identity() and consumed when the rotation commits: the
  // name carries the address that actually took effect, which is only known
  // once ble_hs_id_set_rnd() has succeeded.
  bool identity_refresh_pending_ { false };
  // A generated name follows subsequent address changes, including host
  // recovery. An accepted manual name leaves this mode, even if unchanged.
  bool generated_name_active_ { false };
  // Same-value manual accepts supersede older GAP writes without another rotation.
  uint32_t ble_name_generation_ { 0 };
  // Set when a custom or generated name overrides the Keyboard default
  // ("Logitech K380"); resets on reboot or when "Logitech K380" is re-applied.
  bool keyboard_name_custom_ { false };
  std::string manufacturer_name_ { "ESP32" };  // BLE Device Info manufacturer
  bool continuous_mode_ { true };              // Keep advertising after captures
  uint8_t max_captures_ { 10 };                // Max unique devices (0=unlimited)
  text_sensor::TextSensor* irk_sensor_ { nullptr };
  text_sensor::TextSensor* address_sensor_ { nullptr };
  text_sensor::TextSensor* irk_check_sensor_ { nullptr };
  text_sensor::TextSensor* effective_mac_sensor_ { nullptr };
  IRKCaptureSwitch* advertising_switch_ { nullptr };
  IRKCaptureButton* refresh_identity_button_ { nullptr };
  IRKCaptureText* ble_name_text_ { nullptr };
  IRKCaptureSelect* ble_profile_select_ { nullptr };
  // The profile this boot registered in GATT and advertises. A change takes
  // effect only at the next boot, so a saved request waits in next_profile_.
  BLEProfile ble_profile_ { BLEProfile::KEYBOARD };   // Default to Keyboard profile
  BLEProfile next_profile_ { BLEProfile::KEYBOARD };  // Valid while profile_change_saved_
  bool profile_change_saved_ { false };               // Saved for the next boot
  BLEProfile selected_profile_() const {              // Caller holds state_mutex_
    return profile_change_saved_ ? next_profile_ : ble_profile_;
  }

  // Connection state
  uint16_t conn_handle_ { BLE_HS_CONN_HANDLE_NONE };
  bool advertising_ { false };           // Actual NimBLE advertising state
  bool advertising_requested_ { true };  // Persistent user intent across connections
  uint32_t last_loop_ { 0 };
  uint32_t last_notify_ { 0 };
  bool connected_ { false };
  uint32_t connection_generation_ { 0 };  // Monotonic ID for coalescing capture paths
  uint32_t cancelled_capture_generation_ { 0 };
  bool capture_stop_requested_ { false };
  bool capture_cancelled_locked_(uint32_t generation) const {
    return generation != 0 && cancelled_capture_generation_ != 0 &&
           static_cast<int32_t>(generation - cancelled_capture_generation_) <= 0;
  }
  CaptureOrigin connection_origin_ { CaptureOrigin::UNKNOWN };

  // Security/pairing state
  bool enc_ready_ { false };
  uint32_t enc_time_ { 0 };
  bool sec_retry_done_ { false };
  uint32_t sec_init_time_ms_ { 0 };
  uint32_t adv_restart_time_ { 0 };  // When to restart advertising after a hold-off; 0 = none
  // When to retry an unsuccessful stop (lock busy or radio still active); 0 = none.
  uint32_t adv_stop_retry_time_ { 0 };
  // A hold-off stop the host task could not make because ble_op_mutex_ was
  // busy: the adv_restart_time_ deadline of that hold-off, or 0. loop() makes
  // the stop while advertising is requested only as long as that hold-off is
  // still pending; a newer start, OFF or reset supersedes it.
  uint32_t adv_hold_off_stop_ { 0 };
  // Caller holds state_mutex_. True while the hold-off whose restart deadline
  // is hold_off still stands: advertising is requested and nothing has
  // restarted it or replaced its deadline since.
  bool hold_off_pending_locked_(uint32_t hold_off) const {
    return hold_off != 0 && advertising_requested_ && adv_restart_time_ == hold_off;
  }
  // When the Bluetooth host began to be unsynced (boot or a host reset); 0 once
  // it syncs. Lets a host that never comes up be reported instead of ignored.
  uint32_t host_wait_since_ { 0 };
  uint32_t host_wait_log_time_ { 0 };  // Last stall report; 0 = not reported yet
  uint8_t advertising_start_attempts_ { 0 };
  uint32_t advertising_failure_log_time_ { 0 };
  bool random_address_ready_ { false };
  uint32_t host_generation_ { 0 };  // Invalidates results from calls interrupted by a host reset

  // IRK polling state
  bool irk_gave_up_ { false };
  bool irk_read_error_logged_ { false };  // One bond-read warning per connection
  uint32_t irk_last_try_ms_ { 0 };

  // MAC rotation state machine (non-blocking event-driven implementation)
  enum class MacRotationState {
    IDLE,            // No MAC rotation pending
    REQUESTED,       // Rotation requested, waiting for disconnect
    READY_TO_ROTATE  // Disconnected, ready to perform rotation in loop()
  };
  MacRotationState mac_rotation_state_ { MacRotationState::IDLE };
  uint8_t pending_mac_[6] { 0 };            // Pre-generated MAC for rotation
  uint8_t mac_rotation_retries_ { 0 };      // Retry counter for MAC rotation
  uint32_t mac_rotation_ready_time_ { 0 };  // Earliest next attempt (settle delay or retry backoff)
  uint32_t mac_rotation_generation_ { 0 };  // Invalidates work after reset or a new request
  bool mac_rotation_cleanup_pending_ { false };  // A host-task cleanup was requested
  bool mac_rotation_prepared_ { false };         // Advertising stopped and bond cleanup attempted
  bool mac_rotation_cleanup_failed_ { false };

  // IRK capture tracking with deduplication and rate limiting
  struct IRKCacheEntry {
    std::string irk_hex;
    std::string mac_addr;
    uint8_t addr_type { 0 };
    uint64_t last_published_ms { 0 };  // 64-bit so the republish interval never wraps
    uint32_t last_observed_generation { 0 };
    uint16_t reconnect_count { 0 };
    bool reconnect_limit_reported { false };
    std::string label;         // Optional user label for the visible history entry
    bool in_history { true };  // History can be cleared without losing deduplication
    // Best address check seen for this key; see record_observation_locked_().
    IrkCheck verification { IrkCheck::UNCHECKED };
  };
  std::vector<IRKCacheEntry> irk_cache_;  // Deduplication cache
  // Caller holds state_mutex_. Every change the history shows (a row added,
  // removed, hidden or restored, or a field format_history_json_() writes) must
  // call this so get_session_state() rebuilds the history. Bookkeeping it does
  // not show, and hidden rows, need not. Never 0, which callers pass to mean
  // "no history yet".
  uint32_t history_version_ { 1 };
  void history_changed_locked_() {
    if (++history_version_ == 0) history_version_ = 1;
  }
  // Caller holds state_mutex_. The rows the history shows, copied so they can
  // be formatted after the lock is released. Hidden rows are left behind.
  std::vector<IRKCacheEntry> visible_history_locked_() const;
  // Most entries irk_cache_ holds before the oldest is evicted. The floor is
  // well above the default capture limit on purpose: advertising can be turned
  // back on after the limit is reached, and devices captured then must not
  // push earlier ones out of the history. An evicted device that returns is
  // treated as new. Unlimited mode (max_captures 0) uses the floor.
  static constexpr size_t MIN_CACHE_ENTRIES = 32;
  size_t cache_limit_() const {
    return max_captures_ > MIN_CACHE_ENTRIES ? max_captures_ : MIN_CACHE_ENTRIES;
  }
  uint32_t capture_events_ { 0 };  // IRK publications this session
  uint32_t unique_devices_ { 0 };  // New identity addresses this session
  // Caller holds state_mutex_. True once this session's captures should end
  // advertising: any capture in single mode, or max_captures unique devices
  // in continuous mode (0 = unlimited). stop_after_capture_ is separate.
  bool capture_limit_reached_locked_() const {
    if (!continuous_mode_) return capture_events_ > 0;
    return max_captures_ > 0 && unique_devices_ >= max_captures_;
  }
  uint32_t pairing_start_time_ { 0 };  // Global pairing timeout
  CaptureResult capture_result_;
  IdentityRefreshResult identity_refresh_result_;
  uint32_t failed_generation_ { 0 };  // Coalesces failure reporting per connection
  // The connection whose encryption failed or timed out. Its bond is stale or
  // being deleted, so no fallback read may publish it. A failure from polling that
  // gave up is different: a late bond read is how that connection recovers.
  uint32_t enc_failed_generation_ { 0 };
  uint32_t capture_result_generation_ { 0 };  // Coalesces accepted successful results
  // Triggers are registered before setup and invoked only on the main task.
  std::vector<Trigger<std::string, std::string, std::string>*> capture_triggers_;
  // Keep event delivery separate from the latest displayed result. A delayed
  // main loop must not collapse several accepted captures into one callback.
  struct CaptureEvent {
    std::string irk;
    std::string mac;
    IrkCheck verification { IrkCheck::UNCHECKED };
  };
  static constexpr size_t CAPTURE_EVENT_CAPACITY = 32;
  std::vector<CaptureEvent> pending_capture_events_;
  // Overflow preserves queued events and is reported by the main task.
  uint32_t pending_capture_event_drops_ { 0 };
  // Everything one flush_pending_publishes_() delivers, taken in one lock.
  struct PendingPublishBatch {
    bool adv_pub { false }, adv_val { false };
    bool irk_pub { false };
    bool effmac_pub { false };
    bool stop_after_pub { false }, stop_after_val { false };
    bool label_pub { false };
    bool name_pub { false };
    std::string irk_hex, irk_addr, effmac, label_val, name_val;
    IrkCheck irk_check { IrkCheck::UNCHECKED };
    const char* status { nullptr };  // Null when no Status sensor is configured
    std::vector<CaptureEvent> capture_events;
    uint32_t dropped_capture_events { 0 };
  };

  // A worker queues wake-ups onto NimBLE's task without blocking the main task
  // on queue capacity. Bond deletion and completion disconnects stay serialized
  // with pairing callbacks.
  // The worker/event live for the component's lifetime; only pending state is
  // cancelled on host reset. Pending bond clearing gates advertising and MAC rotation.
  struct ble_npl_event bond_clear_event_ {};
  TaskHandle_t bond_clear_task_ { nullptr };  // Initialized once in setup()
  bool bond_clear_pending_ { false };
  uint32_t bond_clear_host_generation_ { 0 };
  BondClearResult bond_clear_result_;
  // Deferred disconnects (post-capture and MAC rotation) run on the host task
  // so handle reuse cannot interleave with the generation check.
  uint16_t pending_terminate_handle_ { BLE_HS_CONN_HANDLE_NONE };
  uint32_t pending_terminate_generation_ { 0 };
  const char* pending_terminate_context_ { nullptr };  // String literal only
  bool pending_terminate_for_rotation_ { false };
  uint8_t pending_terminate_retries_ { 0 };      // Failed attempts of a rotation disconnect
  uint32_t pending_terminate_retry_time_ { 0 };  // Earliest retry after a transient failure
  bool pending_terminate_busy_logged_ { false };
  // A link nothing tracks (a failed CONNECT left up, or a second connection)
  // that the host task could not close because ble_op_mutex_ was busy. With
  // one connection configured, a newer such link replaces an older one.
  uint16_t untracked_terminate_handle_ { BLE_HS_CONN_HANDLE_NONE };
  const char* untracked_terminate_context_ { nullptr };  // String literal only
  bool untracked_terminate_reboot_ { false };            // Reboot if it cannot be closed

  // Wizard-facing session state (see README). The entity pointers are set
  // once before setup(); the rest is protected by state_mutex_ unless noted.
  text_sensor::TextSensor* status_sensor_ { nullptr };
  IRKCaptureForgetBondsButton* forget_bonds_button_ { nullptr };
  IRKCaptureStopAfterCaptureSwitch* stop_after_capture_switch_ { nullptr };
  IRKCaptureLabelText* next_capture_label_text_ { nullptr };

  // Auto-off advertising after a genuine capture (excludes bonded reconnects).
  bool stop_after_capture_ { false };
  // Consumed by the next device added or restored to visible history.
  std::string next_capture_label_;
  // Caller holds state_mutex_. Clears the label and stages the same for its
  // entity, which would otherwise keep showing a label that is no longer armed.
  void consume_capture_label_() {
    next_capture_label_.clear();
    pending_label_val_.clear();
    pending_label_pub_ = true;
  }
  // Avoids redundant status publishes. Main task only; not locked.
  std::string last_status_value_;
  // CAPTURED/NO_IRK belong to last_result_generation_ (held while that link
  // stays up or during the hold window). FAILED belongs to failed_generation_
  // (encryption failure, poll give-up, or timeout) and is shown for the hold
  // period only. NONE also covers ordinary bonded reconnects.
  enum class CaptureStatus { NONE, CAPTURED, NO_IRK, FAILED };
  CaptureStatus capture_status_ { CaptureStatus::NONE };
  uint32_t status_result_hold_until_ { 0 };

  // Deferred entity publishing: non-main tasks stage values under state_mutex_
  // and loop() drains them via flush_pending_publishes_().
  bool pending_adv_pub_ { false };
  bool pending_irk_pub_ { false };
  std::string pending_irk_hex_;
  std::string pending_irk_addr_;
  IrkCheck pending_irk_check_ { IrkCheck::UNCHECKED };
  uint32_t last_result_generation_ { 0 };  // Orders completed pairing outcomes across delayed reads
  bool pending_effmac_pub_ { false };
  std::string pending_effmac_;
  // Staged config-entity publishes (mutated from HA, httpd, or NimBLE tasks;
  // drained on the main task).
  bool pending_stop_after_capture_pub_ { false };
  bool pending_label_pub_ { false };
  std::string pending_label_val_;
  bool pending_ble_name_pub_ { false };

  // Host state — written by NimBLE reset/sync callbacks and read by the ESPHome
  // main task. Atomic storage provides cross-core visibility without requiring
  // state_mutex_.
  std::atomic<bool> host_synced_ { false };

  // Delayed bond-store reads are queued per peer so a later connection cannot
  // overwrite an earlier peer's pending check. The queue is fixed-size to avoid
  // runtime heap allocation on embedded targets.
  static constexpr size_t PEER_TIMER_CAPACITY = 8;
  struct PeerTimer {
    uint32_t due_ms { 0 };
    ble_addr_t peer_id {};
    uint32_t connection_generation { 0 };
    bool pairing_completed { false };
    CaptureOrigin origin { CaptureOrigin::UNKNOWN };
    // The address the peer connected from, for checking the IRK this timer finds.
    bool has_connection_addr { false };
    ble_addr_t connection_addr {};
  };
  std::array<PeerTimer, PEER_TIMER_CAPACITY> post_disc_timers_ {};
  std::array<PeerTimer, PEER_TIMER_CAPACITY> late_enc_timers_ {};

  // Both deadlines terminate the same connection and share one retry budget.
  // Once started, recovery continues until disconnect or host reset, even if
  // encryption completes while the asynchronous termination is pending.
  enum class TimeoutReason { NONE, ENCRYPTION, PAIRING };
  struct ConnectionTimeout {
    TimeoutReason reason { TimeoutReason::NONE };
    uint8_t attempts { 0 };
    uint32_t retry_ms { 0 };
    bool bond_cleared { false };
  } connection_timeout_;

  // Protects all mutable shared state (connection/pairing/timer/cache/
  // rotation/bond-clear/termination state and pending_* staging fields).
  // Exceptions:
  // - Main-task only: last_loop_, last_notify_, last_status_value_
  // - Atomic: host_synced_
  // - Fixed before the host starts: entity pointers, capture_triggers_,
  //   continuous_mode_, max_captures_, bond_clear_task_
  // - NPL manages bond_clear_event_ queue state. Its callback and argument
  //   are set before the host starts.
  SemaphoreHandle_t state_mutex_ { nullptr };

  // Serializes BLE host control calls across NimBLE and ESPHome tasks
  // (advertising start/stop, disconnects, GAP name updates, MAC changes).
  SemaphoreHandle_t ble_op_mutex_ { nullptr };

  // Internal helpers
  void set_advertising_intent_locked_(bool requested);
  // connection_addr_out, when given, receives the address the peer connected from.
  bool try_get_irk(uint16_t conn_handle, uint8_t irk_out[16], ble_addr_t& peer_id_out,
                   uint32_t connection_generation, CaptureOrigin origin = CaptureOrigin::UNKNOWN,
                   ble_addr_t* connection_addr_out = nullptr);
  void setup_ble();
  void setup_ble_worker_();
  bool register_gatt_services();
  std::string sanitize_ble_name(const std::string& name);
  std::string advertised_name_();                   // Caller holds state_mutex_
  void commit_generated_name_(const uint8_t* mac);  // Caller holds state_mutex_
  // A name accepted for a rotation, with the generations sampled when it was
  // accepted. The rotation is refused if any of them has moved on since.
  struct NameChange {
    const std::string& name;
    uint32_t host_generation;
    uint32_t rotation_generation;
    uint32_t name_generation;
  };
  bool request_mac_rotation_(bool refresh_name, const NameChange* name_change = nullptr);
  // Caller holds state_mutex_.
  bool rotation_current_locked_(uint32_t generation, bool require_idle_link = true) const;
  void abort_rotation_locked_();
  bool adv_start_current_locked_(uint32_t host_generation, uint32_t rotation_generation) const;
  void handle_advertising_failure_(int rc, uint32_t host_generation, const char* operation);

  // IRK validation and deduplication helpers
  bool is_valid_irk(const uint8_t irk[16]);
  // Reads and classifies the peer's stored bond. `bond` is filled for every
  // outcome except NO_BOND and READ_ERROR; rc_out receives the store's result.
  BondRead read_peer_irk_(const ble_addr_t& peer_id, struct ble_store_value_sec& bond,
                          int* rc_out = nullptr);
  // connection_addr is the address the peer connected from, or null if unknown.
  void publish_no_irk_(const ble_addr_t& peer_id, uint32_t connection_generation,
                       bool pairing_completed, CaptureOrigin origin = CaptureOrigin::UNKNOWN,
                       const ble_addr_t* connection_addr = nullptr);
  // Ends a link whose pairing completed without an IRK, if it is still the
  // connection the result was read for. Safe from the NimBLE and main tasks.
  void close_after_no_irk_(uint16_t conn_handle, uint32_t connection_generation,
                           const char* context);
  // Queues a generation-checked disconnect on the host task without waiting.
  void terminate_bounded_(uint16_t conn_handle, uint32_t generation, const char* context,
                          bool for_rotation = false);
  void handle_pending_termination_();  // NimBLE event task only
  // Disconnects a link the component does not track, without waiting for
  // ble_op_mutex_: when it is busy, the worker callback retries. NimBLE host
  // task only.
  void terminate_untracked_(uint16_t conn_handle, bool reboot_on_failure, const char* context);
  void handle_untracked_termination_();  // NimBLE event task only
  void finish_untracked_terminate_(uint16_t conn_handle, int rc, bool reboot_on_failure,
                                   const char* context);
  // Shows Status "failed" for a pairing that ended without a result.
  void report_capture_failure_(uint32_t connection_generation);
  void report_host_stall_(uint32_t now);  // Main task only
  // Shows Status "failed" when setup() gives up. Main task, setup() only.
  void publish_setup_failure_();
  // Caller holds state_mutex_. Updates the deduplication cache for one
  // observation and returns whether (and why) to publish it.
  PublishDecision record_observation_locked_(const std::string& irk_hex, const std::string& addr,
                                             uint8_t addr_type, uint32_t connection_generation,
                                             bool force_pairing_publish,
                                             IrkCheck check = IrkCheck::UNCHECKED);
  // Caller holds state_mutex_.
  void restore_capture_history_(IRKCacheEntry& entry, PublishDecision& out_decision);
  static std::string format_history_json_(const std::vector<IRKCacheEntry>& cache);
  void notify_bond_clear_();    // Nonblocking maintenance worker wake-up
  void queue_bond_clear_();     // Worker task only
  void handle_forget_bonds_();  // NimBLE event-queue callback only

  // Timer handlers
  bool enqueue_peer_timer_(std::array<PeerTimer, PEER_TIMER_CAPACITY>& timers,
                           const ble_addr_t& peer_id, uint32_t connection_generation,
                           uint32_t delay_ms, bool pairing_completed = false,
                           CaptureOrigin origin = CaptureOrigin::UNKNOWN,
                           const ble_addr_t* connection_addr = nullptr);
  void schedule_post_disconnect_check(const ble_addr_t& peer_id, uint32_t connection_generation,
                                      bool pairing_completed = false,
                                      CaptureOrigin origin = CaptureOrigin::UNKNOWN,
                                      const ble_addr_t* connection_addr = nullptr);
  void schedule_late_enc_check(const ble_addr_t& peer_id, uint32_t connection_generation,
                               CaptureOrigin origin = CaptureOrigin::UNKNOWN,
                               const ble_addr_t* connection_addr = nullptr);
  void run_peer_timer_(std::array<PeerTimer, PEER_TIMER_CAPACITY>& timers, uint32_t now,
                       const char* context_tag);
  void cancel_peer_timers_(const ble_addr_t& peer_id,
                           uint32_t connection_generation);  // Caller holds state_mutex_

  // Loop helpers
  void handle_mac_rotation_(uint32_t now);
  void handle_rotation_cleanup_();  // NimBLE event task only
  void finish_rotation_cleanup_locked_(const char* failure_reason);
  void retry_security_if_needed(uint32_t now);
  bool handle_connection_timeout_(uint32_t now);
  // Caller holds state_mutex_. Reset is also used by connect/host reset;
  // finish additionally applies the timeout cooldown and MAC rotation handoff.
  void reset_connection_state_();
  void finish_disconnect_(uint32_t now);
  void notify_hr_if_due(uint32_t now);
  void poll_irk_if_due(uint32_t now);

  // Deferred entity publishing (see pending_* fields). Staged from any context,
  // drained only on the ESPHome main task in loop().
  // Takes every staged publish, clearing its pending_* flag, under one lock.
  PendingPublishBatch take_pending_publishes_();
  // Main task only. Publish the IRK, identity MAC, and IRK check sensors.
  void publish_result_entities_(const std::string& irk, const std::string& mac, IrkCheck check);
  // Main task only. Publish Status when it changes.
  void publish_status_if_changed_(const char* status);
  void flush_pending_publishes_();
  // Reboots from the main loop. Safe to call from any task, including after
  // the component has been marked failed.
  void request_reboot_();

  // Caller holds state_mutex_ while snapshotting the pending result values.
  const char* status_value_locked_(uint32_t now);
};

}  // namespace irk_capture
}  // namespace esphome
