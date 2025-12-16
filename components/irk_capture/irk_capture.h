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
    void setup() override;
    void loop() override;
    void dump_config() override;
    float get_setup_priority() const override { return -200.0f; }

    void set_ble_name(const std::string &name) { ble_name_ = name; }
    void set_start_on_boot(bool start) { start_on_boot_ = start; }
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

    void update_ble_name(const std::string &name);
    std::string get_ble_name() { return ble_name_; }
    void start_advertising();
    void stop_advertising();
    void refresh_mac();
    bool is_advertising() { return advertising_; }

    void on_connect(uint16_t conn_handle);
    void on_disconnect();
    void on_auth_complete(bool encrypted);

    // Publish IRK to sensors (called by centralized helper)
    void publish_irk_to_sensors(const std::string &irk_hex, const char *addr_str);

    // GAP event handler (static trampoline)
    static int gap_event_handler(struct ble_gap_event *event, void *arg);

 protected:
    // Configuration
    std::string ble_name_{"IRK Capture"};
    bool start_on_boot_{true};

    // Entities
    text_sensor::TextSensor *irk_sensor_{nullptr};
    text_sensor::TextSensor *address_sensor_{nullptr};
    IRKCaptureSwitch *advertising_switch_{nullptr};
    IRKCaptureButton *new_mac_button_{nullptr};
    IRKCaptureText *ble_name_text_{nullptr};

    // State
    uint16_t conn_handle_{BLE_HS_CONN_HANDLE_NONE};
    uint16_t hr_char_handle_{0};
    uint16_t prot_char_handle_{0};   // Protected Info characteristic handle
    bool advertising_{false};
    uint32_t last_loop_{0};
    uint32_t last_notify_{0};
    bool connected_{false};
    bool host_synced_{false};

    // Pairing/IRK timing control
    bool enc_ready_{false};
    uint32_t enc_time_{0};

    // Internals
    bool try_get_irk(uint16_t conn_handle, std::string &irk, std::string &addr);
    void setup_ble();
    void register_gatt_services();
};

}  // namespace irk_capture
}  // namespace esphome
