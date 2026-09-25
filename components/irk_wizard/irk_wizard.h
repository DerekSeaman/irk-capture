#pragma once

#include <atomic>
#include <string>

#include "esphome/components/irk_capture/irk_capture.h"
#include "esphome/components/text/text.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "wizard_util.h"

#ifdef USE_ESP32
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#else
#error "IRK Wizard requires ESP32 (esp_http_server)"
#endif

namespace esphome {
namespace irk_wizard {

// A point-in-time copy of everything the wizard UI needs to render. Built on
// the ESPHome main task (see loop()) and read by esp_http_server's own
// request-handling task, so it is copied under snapshot_mutex_ on both ends
// rather than handing the httpd task raw TextSensor pointers to read from a
// different task than the one that publishes them.
struct WizardSnapshot {
  std::string status { "idle" };
  std::string history_json { "[]" };
  std::string irk;
  std::string device_mac;
  std::string effective_mac;
  std::string ble_name;
  // What the phone will actually show in its Bluetooth list. Differs from
  // ble_name in the Keyboard profile, which poses as a Logitech K380.
  std::string advertised_name;
  std::string next_capture_label;
  std::string profile { "Heart Sensor" };
  bool advertising { false };
  bool stop_after_capture { false };
};

class IRKWizardComponent : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override {
    return setup_priority::AFTER_WIFI;
  }

  void set_irk_capture(irk_capture::IRKCaptureComponent* irk) {
    irk_capture_ = irk;
  }
  void set_status_sensor(text_sensor::TextSensor* s) {
    status_sensor_ = s;
  }
  void set_irk_sensor(text_sensor::TextSensor* s) {
    irk_sensor_ = s;
  }
  void set_device_mac_sensor(text_sensor::TextSensor* s) {
    device_mac_sensor_ = s;
  }
  void set_effective_mac_sensor(text_sensor::TextSensor* s) {
    effective_mac_sensor_ = s;
  }
  // Gives the device a name and address the phone has not seen. Callable from
  // any task; the work itself is deferred to the main loop.
  void fresh_identity();
  void set_port(uint16_t port) {
    port_ = port;
  }
  // Optional HTTP Basic auth. The expected header value is precomputed once
  // here so request handling is a constant-time string compare with no
  // base64 decoding (and no credentials living in RAM in plaintext beyond
  // this encoded form).
  void set_auth(const std::string& username, const std::string& password);

  irk_capture::IRKCaptureComponent* irk_capture() {
    return irk_capture_;
  }
  // Gzipped page, generated from wizard_page.html at build time.
  void set_page(const uint8_t* data, size_t size) {
    page_data_ = data;
    page_size_ = size;
  }
  const uint8_t* page_data() const {
    return page_data_;
  }
  size_t page_size() const {
    return page_size_;
  }
  AuthThrottle& auth_throttle() {
    return auth_throttle_;
  }
  const std::string& expected_auth() const {
    return expected_auth_;
  }
  // Handlers call this so an idle device can back off its snapshot refresh
  // (and the state_mutex_ traffic that comes with it).
  void note_request() {
    last_request_ms_.store(millis(), std::memory_order_relaxed);
  }

  // Thread-safe: called from httpd request handlers (their own FreeRTOS
  // task), never from loop().
  WizardSnapshot get_snapshot();

 protected:
  irk_capture::IRKCaptureComponent* irk_capture_ { nullptr };
  text_sensor::TextSensor* status_sensor_ { nullptr };
  text_sensor::TextSensor* irk_sensor_ { nullptr };
  text_sensor::TextSensor* device_mac_sensor_ { nullptr };
  text_sensor::TextSensor* effective_mac_sensor_ { nullptr };
  uint16_t port_ { 8080 };
  const uint8_t* page_data_ { nullptr };
  size_t page_size_ { 0 };
  AuthThrottle auth_throttle_;
  std::string expected_auth_;  // "Basic <base64>", empty = auth disabled
  std::atomic<uint32_t> last_request_ms_ { 0 };

  httpd_handle_t server_ { nullptr };
  SemaphoreHandle_t snapshot_mutex_ { nullptr };
  WizardSnapshot snapshot_;
  uint32_t last_snapshot_ms_ { 0 };
  uint32_t last_start_attempt_ms_ { 0 };
  uint8_t start_attempts_ { 0 };

  bool start_server_();
  // Main-task-only: reads TextSensor::state (safe here; publish_state() for
  // all of these sensors also only ever runs on the main task) and stores
  // the result under snapshot_mutex_.
  void refresh_snapshot_();
};

}  // namespace irk_wizard
}  // namespace esphome
