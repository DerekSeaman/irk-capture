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
#include "esphome/core/component.h"

// ESP32-only component - requires Bluetooth hardware
#ifdef USE_ESP32
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_hs.h>
#include <host/ble_store.h>
#include <host/ble_uuid.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
#else
#error \
    "IRK Capture component requires ESP32 platform with Bluetooth support (ESP32/ESP32-C3/ESP32-S3/ESP32-C6)"
#endif

namespace esphome {
namespace irk_capture {

class IRKCaptureComponent;

// BLE advertising profile options
enum class BLEProfile : uint8_t {
  HEART_SENSOR = 0,  // Heart Rate Sensor
  KEYBOARD = 1,      // Logitech K380 Keyboard
};

// Select for BLE profile
class IRKCaptureSelect : public select::Select, public Component {
 public:
  void set_parent(IRKCaptureComponent* parent) {
    parent_ = parent;
  }
  void control(const std::string& value) override;
  void dump_config() override;

 protected:
  IRKCaptureComponent* parent_ { nullptr };
};

// Text input for BLE name
class IRKCaptureText : public text::Text, public Component {
 public:
  void set_parent(IRKCaptureComponent* parent) {
    parent_ = parent;
  }
  void control(const std::string& value) override;
  void dump_config() override;

 protected:
  IRKCaptureComponent* parent_ { nullptr };
};

// Text sensor for IRK/Address output
class IRKCaptureTextSensor : public text_sensor::TextSensor, public Component {
 public:
  void setup() override {
  }
  void dump_config() override;
};

// Switch for advertising control
class IRKCaptureSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(IRKCaptureComponent* parent) {
    parent_ = parent;
  }
  void write_state(bool state) override;
  void dump_config() override;

 protected:
  IRKCaptureComponent* parent_ { nullptr };
};

// Button for new MAC
class IRKCaptureButton : public button::Button, public Component {
 public:
  void set_parent(IRKCaptureComponent* parent) {
    parent_ = parent;
  }
  void press_action() override;
  void dump_config() override;

 protected:
  IRKCaptureComponent* parent_ { nullptr };
};

// Free-function helpers (external linkage). Definitions live in the .cpp.
int handle_gap_connect(class IRKCaptureComponent* self, struct ble_gap_event* ev);
int handle_gap_disconnect(class IRKCaptureComponent* self, struct ble_gap_event* ev);
int handle_gap_enc_change(class IRKCaptureComponent* self, struct ble_gap_event* ev);
int handle_gap_repeat_pairing(class IRKCaptureComponent* self, struct ble_gap_event* ev);
void publish_and_log_irk(class IRKCaptureComponent* self, const ble_addr_t& peer_id_addr,
                         const std::string& irk_hex, const char* context_tag,
                         uint32_t connection_generation);

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

  // Friend classes for UI components (access protected members)
  friend class IRKCaptureText;
  friend class IRKCaptureSwitch;
  friend class IRKCaptureButton;
  friend class IRKCaptureSelect;

  // Friend functions for GAP event handlers and helpers (access private
  // members)
  friend int handle_gap_connect(IRKCaptureComponent* self, struct ble_gap_event* ev);
  friend int handle_gap_disconnect(IRKCaptureComponent* self, struct ble_gap_event* ev);
  friend int handle_gap_enc_change(IRKCaptureComponent* self, struct ble_gap_event* ev);
  friend int handle_gap_repeat_pairing(IRKCaptureComponent* self, struct ble_gap_event* ev);
  friend void publish_and_log_irk(IRKCaptureComponent* self, const ble_addr_t& peer_id_addr,
                                  const std::string& irk_hex, const char* context_tag,
                                  uint32_t connection_generation);

  // Friend declarations for GATT callback functions (access protected members)
  friend int chr_read_devinfo(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt* ctxt, void* arg);
  friend int chr_read_batt(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt* ctxt, void* arg);
  friend int chr_read_hr(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt* ctxt, void* arg);
  friend int chr_read_protected(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt* ctxt, void* arg);

  // Configuration setters
  void set_ble_name(const std::string& name) {
    ble_name_ = name;
  }
  void set_start_on_boot(bool start) {
    start_on_boot_ = start;
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
  void set_effective_mac_sensor(text_sensor::TextSensor* sensor) {
    effective_mac_sensor_ = sensor;
  }
  void set_advertising_switch(IRKCaptureSwitch* sw) {
    advertising_switch_ = sw;
    if (sw) sw->set_parent(this);
  }
  void set_new_mac_button(IRKCaptureButton* btn) {
    new_mac_button_ = btn;
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

  // Profile management
  void set_ble_profile(BLEProfile profile);
  BLEProfile get_ble_profile();

  // Public actions
  void update_ble_name(const std::string& name);
  std::string get_ble_name();
  void start_advertising();
  void stop_advertising();
  void set_advertising_requested(bool requested);
  void refresh_mac();
  bool is_advertising();            // Thread-safe check of actual advertising state
  bool is_advertising_requested();  // Thread-safe check of the user's desired state
  void on_ble_host_synced();        // Called when NimBLE host is ready

  // GAP events
  void on_connect(uint16_t conn_handle);
  bool on_disconnect(uint16_t conn_handle);
  // GAP event handler (static trampoline)
  static int gap_event_handler(struct ble_gap_event* event, void* arg);

  // Sensor publishing helper
  void publish_irk_to_sensors(const std::string& irk_hex, const char* addr_str);
  void publish_effective_mac();

 protected:
  // Configuration/state
  std::string ble_name_ { "IRK Capture" };
  std::string manufacturer_name_ { "ESPresense" };  // BLE Device Info manufacturer
  bool start_on_boot_ { true };
  bool continuous_mode_ { true };  // Keep advertising after captures
  uint8_t max_captures_ { 10 };    // Max unique devices (0=unlimited)
  text_sensor::TextSensor* irk_sensor_ { nullptr };
  text_sensor::TextSensor* address_sensor_ { nullptr };
  text_sensor::TextSensor* effective_mac_sensor_ { nullptr };
  IRKCaptureSwitch* advertising_switch_ { nullptr };
  IRKCaptureButton* new_mac_button_ { nullptr };
  IRKCaptureText* ble_name_text_ { nullptr };
  IRKCaptureSelect* ble_profile_select_ { nullptr };
  BLEProfile ble_profile_ { BLEProfile::KEYBOARD };  // Default to Keyboard profile

  // Connection state
  uint16_t conn_handle_ { BLE_HS_CONN_HANDLE_NONE };
  uint16_t hr_char_handle_ { 0 };
  uint16_t prot_char_handle_ { 0 };
  bool advertising_ { false };           // Actual NimBLE advertising state
  bool advertising_requested_ { true };  // Persistent user intent across connections
  uint32_t last_loop_ { 0 };
  uint32_t last_notify_ { 0 };
  bool connected_ { false };
  uint32_t connection_generation_ { 0 };  // Monotonic ID for coalescing capture paths
  uint32_t pairing_generation_ { 0 };     // Generation that started without a cached bond
  uint32_t repair_generation_ { 0 };      // Generation authorized by REPEAT_PAIRING

  // Security/pairing state
  bool enc_ready_ { false };
  uint32_t enc_time_ { 0 };
  bool sec_retry_done_ { false };
  uint32_t sec_init_time_ms_ { 0 };
  bool sec_timeout_terminate_pending_ { false };
  bool sec_timeout_bond_cleared_ { false };
  uint8_t sec_timeout_terminate_attempts_ { 0 };
  uint32_t sec_timeout_terminate_retry_ms_ { 0 };
  bool suppress_next_adv_ { false };  // Prevent immediate re-advertising after IRK re-publish
  uint32_t adv_restart_time_ { 0 };   // Time to auto-restart advertising after suppression

  // IRK polling state
  bool irk_gave_up_ { false };
  uint32_t irk_last_try_ms_ { 0 };

  // MAC rotation state machine (non-blocking event-driven implementation)
  enum class MacRotationState {
    IDLE,              // No MAC rotation pending
    REQUESTED,         // refresh_mac() called, waiting for disconnect
    READY_TO_ROTATE,   // Disconnected, ready to perform rotation in loop()
    ROTATION_COMPLETE  // Rotation done, ready to restart advertising
  };
  MacRotationState mac_rotation_state_ { MacRotationState::IDLE };
  uint8_t pending_mac_[6] { 0 };            // Pre-generated MAC for rotation
  uint8_t mac_rotation_retries_ { 0 };      // Retry counter for MAC rotation
  uint32_t mac_rotation_ready_time_ { 0 };  // Time when rotation can start (after settling delay)

  // IRK capture tracking with deduplication and rate limiting
  struct IRKCacheEntry {
    std::string irk_hex;
    std::string mac_addr;
    uint8_t addr_type;
    uint32_t first_seen_ms;
    uint32_t last_seen_ms;
    uint32_t last_published_ms;
    uint32_t last_observed_generation;
    uint16_t reconnect_count;
    bool reconnect_limit_reported;
  };
  std::vector<IRKCacheEntry> irk_cache_;  // Deduplication cache
  uint32_t capture_events_ { 0 };         // IRK publications this session
  uint32_t unique_devices_ { 0 };         // New identity addresses this session
  uint32_t pairing_start_time_ { 0 };     // Global pairing timeout

  // Deferred entity publishing. ESPHome entity publish_state() is not safe to
  // call from the NimBLE task, so BLE-context code stages values here (under
  // state_mutex_) and the main-task loop() drains them via
  // flush_pending_publishes_().
  bool pending_adv_pub_ { false };
  bool pending_adv_val_ { false };
  bool pending_irk_pub_ { false };
  std::string pending_irk_hex_;
  std::string pending_irk_addr_;
  bool pending_effmac_pub_ { false };
  std::string pending_effmac_;

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
  };
  std::array<PeerTimer, PEER_TIMER_CAPACITY> post_disc_timers_ {};
  std::array<PeerTimer, PEER_TIMER_CAPACITY> late_enc_timers_ {};

  // Bounded retry state for the global pairing-timeout termination path. Lets us
  // retry ble_gap_terminate() without falsely reporting the connection closed
  // while NimBLE still holds it.
  uint8_t timeout_terminate_attempts_ { 0 };
  uint32_t timeout_terminate_retry_ms_ { 0 };
  bool timeout_terminate_pending_ { false };
  bool timeout_bond_cleared_ { false };
  bool timeout_cooldown_on_disconnect_ { false };

  // FreeRTOS mutex for thread-safe access to shared state
  // Protects: timer queues, conn_handle_, connected_, advertising_,
  //           advertising_requested_,
  //           pairing_start_time_,
  //           ble_name_, manufacturer_name_, mac_rotation_state_, pending_mac_,
  //           suppress_next_adv_, adv_restart_time_, capture_events_,
  //           unique_devices_, irk_cache_ (deduplication state),
  //           connection_generation_, pairing_generation_, repair_generation_,
  //           enc_ready_, enc_time_, sec_retry_done_, sec_init_time_ms_,
  //           security/global timeout termination retry state,
  //           irk_gave_up_, irk_last_try_ms_ (pairing/polling state),
  //           pending_adv_/pending_irk_/pending_effmac_ (deferred-publish queue)
  SemaphoreHandle_t state_mutex_ { nullptr };

  // Serializes BLE control operations that can be called from both NimBLE and
  // ESPHome contexts (start/stop advertising, device-name updates, MAC
  // changes).
  SemaphoreHandle_t ble_op_mutex_ { nullptr };

  // Internal helpers
  bool try_get_irk(uint16_t conn_handle, uint8_t irk_out[16], ble_addr_t& peer_id_out);
  void setup_ble();
  bool register_gatt_services();
  std::string sanitize_ble_name(const std::string& name);

  // IRK validation and deduplication helpers
  bool is_valid_irk(const uint8_t irk[16]);
  bool should_publish_irk(const std::string& irk_hex, const std::string& addr, uint8_t addr_type,
                          uint32_t connection_generation, bool force_pairing_publish,
                          bool& out_should_stop_adv, bool& out_is_new_device,
                          bool& out_limit_just_reached);

  // Timer handlers
  void schedule_post_disconnect_check(const ble_addr_t& peer_id, uint32_t connection_generation);
  void schedule_late_enc_check(const ble_addr_t& peer_id, uint32_t connection_generation);
  void handle_post_disconnect_timer(uint32_t now);
  void handle_late_enc_timer(uint32_t now);

  // Loop helpers
  void retry_security_if_needed(uint32_t now);
  void notify_hr_if_due(uint32_t now);
  void poll_irk_if_due(uint32_t now);

  // Deferred entity publishing (see pending_* fields). Staged from any context,
  // drained only on the ESPHome main task in loop().
  void stage_advertising_publish_(bool value);
  void flush_pending_publishes_();
};

}  // namespace irk_capture
}  // namespace esphome
