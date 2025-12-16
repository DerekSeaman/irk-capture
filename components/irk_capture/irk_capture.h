#pragma once

#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/button/button.h"
#include "esphome/components/text/text.h"

// ESP-IDF NimBLE headers (C API)
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>

#include <host/ble_hs.h>
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_uuid.h>
#include <host/ble_store.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>

namespace esphome {
namespace irk_capture {

/*
Behavior parity checklist (v1.0 → v1.2):
-  Same security init timing and single retry
-  Same ENC_CHANGE immediate IRK read + optional terminate
-  Same post-disconnect immediate and delayed read
-  Same late ENC (+5s) read
-  Same periodic poll for IRK while connected
-  Same IRK hex order (reverse) and use of peer identity address
-  Same advertising payload and MAC refresh behavior (blocking)
*/

// Forward-declare main component for child entities
class IRKCaptureComponent;

// Text input for BLE name
class IRKCaptureText : public text::Text, public Component {
 public:
    void set_parent(IRKCaptureComponent *parent) { parent_ = parent; }
    void control(const std::string &value) override;
 protected:
    IRKCaptureComponent *parent_{nullptr};
};

// Text sensor for IRK/Address output
class IRKCaptureTextSensor : public text_sensor::TextSensor, public Component {
 public:
    void setup() override {}
    void dump_config() override {}
};

// Switch for advertising control
class IRKCaptureSwitch : public switch_::Switch, public Component {
 public:
    void set_parent(IRKCaptureComponent *parent) { parent_ = parent; }
    void write_state(bool state) override;
 protected:
    IRKCaptureComponent *parent_{nullptr};
};

// Button for new MAC
class IRKCaptureButton : public button::Button, public Component {
 public:
    void set_parent(IRKCaptureComponent *parent) { parent_ = parent; }
    void press_action() override;
 protected:
    IRKCaptureComponent *parent_{nullptr};
};

// Main component
class IRKCaptureComponent : public Component {
 public:
    // Component lifecycle
    void setup() override;
    void loop() override;
    void dump_config() override;
    float get_setup_priority() const override { return -200.0f; }

    // Configuration
    void set_ble_name(const std::string &name) { ble_name_ = name; }
    void set_start_on_boot(bool start) { start_on_boot_ = start; }

    // Entities
    void set_irk_sensor(text_sensor::TextSensor *sensor) { irk_sensor_ = sensor; }
    void set_address_sensor(text_sensor::TextSensor *sensor) { address_sensor_ = sensor; }
    void set_advertising_switch(IRKCaptureSwitch *sw) {
        advertising_switch_ = sw;
        sw->set_parent(this);
    }
    void set_new_mac_button(IRKCaptureButton *btn) {
        new_mac_button_ = btn;
        btn->set_parent(this);
    }
    void set_ble_name_text(IRKCaptureText *txt) {
        ble_name_text_ = txt;
        txt->set_parent(this);
    }

    // Public API
    void update_ble_name(const std::string &name);
    std::string get_ble_name() { return ble_name_; }
    void start_advertising();
    void stop_advertising();
    void refresh_mac();  // blocking (v1.0 behavior retained)
    bool is_advertising() { return advertising_; }

    // Compatibility stubs (logic remains in GAP callbacks)
    void on_connect(uint16_t conn_handle);
    void on_disconnect();
    void on_auth_complete(bool encrypted);

    // Centralized publish utility
    void publish_irk_to_sensors(const std::string &irk_hex, const char *addr_str);

    // GAP event handler (static trampoline)
    static int gap_event_handler(struct ble_gap_event *event, void *arg);

 protected:
    // Readability: timing constants
    static constexpr uint32_t LOOP_MIN_INTERVAL_MS      = 50;
    static constexpr uint32_t HR_NOTIFY_INTERVAL_MS     = 250;
    static constexpr uint32_t POST_DISC_DELAY_MS        = 800;
    static constexpr uint32_t ENC_LATE_READ_DELAY_MS    = 5000;
    static constexpr uint32_t ENC_TO_FIRST_TRY_DELAY_MS = 2000;
    static constexpr uint32_t ENC_TRY_INTERVAL_MS       = 1000;
    static constexpr uint32_t ENC_GIVE_UP_AFTER_MS      = 45000;

    // Configuration
    std::string ble_name_{"IRK Capture"};
    bool start_on_boot_{true};

    // Entities
    text_sensor::TextSensor *irk_sensor_{nullptr};
    text_sensor::TextSensor *address_sensor_{nullptr};
    IRKCaptureSwitch *advertising_switch_{nullptr};
    IRKCaptureButton *new_mac_button_{nullptr};
    IRKCaptureText *ble_name_text_{nullptr};

    // BLE handles/state
    uint16_t conn_handle_{BLE_HS_CONN_HANDLE_NONE};
    uint16_t hr_char_handle_{0};
    uint16_t prot_char_handle_{0};
    bool advertising_{false};
    bool host_synced_{false};
    bool connected_{false};

    // Timing/state
    uint32_t last_loop_{0};
    uint32_t last_notify_{0};
    bool enc_ready_{false};
    uint32_t enc_time_{0};

    // Loop-helper state (moved from function-statics for clarity)
    bool sec_retry_done_{false};
    uint32_t sec_init_time_ms_{0};

    bool irk_gave_up_{false};
    uint32_t irk_last_try_ms_{0};

    // Timer bundle to replace file-scope globals
    struct Timers {
        uint32_t post_disc_due_ms{0};
        ble_addr_t last_peer_id{};
        uint32_t late_enc_due_ms{0};
        ble_addr_t enc_peer_id{};
    } timers_;

    // BLE setup and GATT
    void setup_ble();
    void register_gatt_services();

    // IRK helpers
    bool try_get_irk(uint16_t conn_handle, std::string &irk, std::string &addr);

    // Timer helpers (readability only)
    void schedule_post_disconnect_check(const ble_addr_t &peer_id);
    void schedule_late_enc_check(const ble_addr_t &peer_id);
    void handle_post_disconnect_timer(uint32_t now_ms);
    void handle_late_enc_timer(uint32_t now_ms);

    // Loop helpers (readability only; same behavior)
    void retry_security_if_needed(uint32_t now_ms);
    void notify_hr_if_due(uint32_t now_ms);
    void poll_irk_if_due(uint32_t now_ms);
};

}  // namespace irk_capture
}  // namespace esphome
