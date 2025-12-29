#pragma once

#include "esphome/components/button/button.h"
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
  void setup() override {}
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
                         const std::string& irk_hex, const char* context_tag);

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

  // Friend functions for GAP event handlers and helpers (access private members)
  friend int handle_gap_connect(IRKCaptureComponent* self, struct ble_gap_event* ev);
  friend int handle_gap_disconnect(IRKCaptureComponent* self, struct ble_gap_event* ev);
  friend int handle_gap_enc_change(IRKCaptureComponent* self, struct ble_gap_event* ev);
  friend int handle_gap_repeat_pairing(IRKCaptureComponent* self, struct ble_gap_event* ev);
  friend void publish_and_log_irk(IRKCaptureComponent* self, const ble_addr_t& peer_id_addr,
                                  const std::string& irk_hex, const char* context_tag);

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

  // Public actions
  void update_ble_name(const std::string& name);
  std::string get_ble_name() {
    return ble_name_;
  }
  void start_advertising();
  void stop_advertising();
  void refresh_mac();
  bool is_advertising();  // Thread-safe check of advertising state

  // GAP events
  void on_connect(uint16_t conn_handle);
  void on_disconnect();
  void on_auth_complete(bool encrypted);

  // GAP event handler (static trampoline)
  static int gap_event_handler(struct ble_gap_event* event, void* arg);

  // Sensor publishing helper
  void publish_irk_to_sensors(const std::string& irk_hex, const char* addr_str);

 protected:
  // Configuration/state
  std::string ble_name_ { "IRK Capture" };
  std::string manufacturer_name_ { "ESPresense" };  // BLE Device Info manufacturer
  bool start_on_boot_ { true };
  bool continuous_mode_ { true };  // Keep advertising after captures
  uint8_t max_captures_ { 10 };    // Max captures (0=unlimited)
  text_sensor::TextSensor* irk_sensor_ { nullptr };
  text_sensor::TextSensor* address_sensor_ { nullptr };
  IRKCaptureSwitch* advertising_switch_ { nullptr };
  IRKCaptureButton* new_mac_button_ { nullptr };
  IRKCaptureText* ble_name_text_ { nullptr };

  // Connection state
  uint16_t conn_handle_ { BLE_HS_CONN_HANDLE_NONE };
  uint16_t hr_char_handle_ { 0 };
  uint16_t prot_char_handle_ { 0 };
  bool advertising_ { false };
  uint32_t last_loop_ { 0 };
  uint32_t last_notify_ { 0 };
  bool connected_ { false };

  // Security/pairing state
  bool enc_ready_ { false };
  uint32_t enc_time_ { 0 };
  bool sec_retry_done_ { false };
  uint32_t sec_init_time_ms_ { 0 };
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
    uint32_t first_seen_ms;
    uint32_t last_seen_ms;
    uint8_t capture_count;
  };
  std::vector<IRKCacheEntry> irk_cache_;  // Deduplication cache
  uint8_t total_captures_ { 0 };          // Total IRKs captured this session
  uint32_t last_publish_time_ { 0 };      // Last IRK publish timestamp
  uint32_t pairing_start_time_ { 0 };     // Global pairing timeout

  // Host state (ESPHome-managed host; this is a soft flag)
  bool host_synced_ { false };

  // Timer targets and cached peer ids
  struct Timers {
    uint32_t post_disc_due_ms { 0 };
    uint32_t late_enc_due_ms { 0 };
    ble_addr_t last_peer_id {};
    ble_addr_t enc_peer_id {};
  } timers_ {};

  // FreeRTOS mutex for thread-safe access to shared state
  // Protects: timers_, conn_handle_, advertising_, pairing_start_time_, ble_name_,
  //           manufacturer_name_, mac_rotation_state_, pending_mac_,
  //           suppress_next_adv_, adv_restart_time_, total_captures_,
  //           irk_cache_, last_publish_time_ (deduplication state)
  SemaphoreHandle_t state_mutex_ { nullptr };

  // Internal helpers
  bool try_get_irk(uint16_t conn_handle, uint8_t irk_out[16], ble_addr_t& peer_id_out);
  void setup_ble();
  void register_gatt_services();
  std::string sanitize_ble_name(const std::string& name);

  // IRK validation and deduplication helpers
  bool is_valid_irk(const uint8_t irk[16]);
  bool should_publish_irk(const std::string& irk_hex, const std::string& addr,
                          bool& out_should_stop_adv);

  // Timer handlers
  void schedule_post_disconnect_check(const ble_addr_t& peer_id);
  void schedule_late_enc_check(const ble_addr_t& peer_id);
  void handle_post_disconnect_timer(uint32_t now);
  void handle_late_enc_timer(uint32_t now);

  // Loop helpers
  void retry_security_if_needed(uint32_t now);
  void notify_hr_if_due(uint32_t now);
  void poll_irk_if_due(uint32_t now);
};

}  // namespace irk_capture
}  // namespace esphome
