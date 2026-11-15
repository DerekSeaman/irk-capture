#pragma once

#include <atomic>
#include <memory>
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

enum class CaptureStartState : uint8_t { IDLE, PENDING, READY, CANCELLED, FAILED };

// The name a state has in /api/status. The wizard page compares these strings.
inline const char* capture_start_state_name(CaptureStartState state) {
  switch (state) {
    case CaptureStartState::PENDING:
      return "pending";
    case CaptureStartState::READY:
      return "ready";
    case CaptureStartState::CANCELLED:
      return "cancelled";
    case CaptureStartState::FAILED:
      return "failed";
    case CaptureStartState::IDLE:
      break;
  }
  return "idle";
}

struct CaptureStart {
  uint32_t operation_id { 0 };
  CaptureStartState state { CaptureStartState::IDLE };
  uint32_t boot_id { 0 };
  uint32_t sequence { 0 };
};

// A point-in-time copy of everything the wizard UI needs to render. Built on
// the ESPHome main task (see loop()) and read by esp_http_server's own
// request-handling task, so it is copied under snapshot_mutex_ on both ends
// rather than handing the httpd task raw TextSensor pointers to read from a
// different task than the one that publishes them.
struct WizardSnapshot {
  std::string status { "idle" };
  // Shared so that copying a snapshot for a request does not copy the history,
  // its only large value. Null until the first refresh; read it as "[]".
  std::shared_ptr<const std::string> history_json;
  std::string irk;
  std::string device_mac;
  std::string effective_mac;
  // What the phone shows in its Bluetooth list: the Keyboard profile's
  // "Logitech K380", or whatever a rename or an identity refresh replaced it
  // with since boot.
  std::string advertised_name;
  std::string next_capture_label;
  std::string profile { "Heart Sensor" };
  bool advertising { false };
  bool stop_after_capture { false };
  irk_capture::CaptureResult capture_result;
  irk_capture::BondClearResult bond_clear;
  irk_capture::IdentityRefreshResult identity_refresh;
  CaptureStart capture_start;
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
  // Switches the BLE profile, which reboots the device. Callable from any
  // task; set_ble_profile() publishes entities directly, so the call itself
  // is deferred to the main loop.
  void set_profile(bool keyboard);
  // Reserve before acknowledging a change; retain the reservation until reboot
  // or a failed save so another HTTP request cannot overtake deferred work.
  bool try_begin_profile_change() {
    bool expected = false;
    return profile_change_pending_.compare_exchange_strong(expected, true);
  }
  void release_profile_change() {
    profile_change_pending_.store(false);
  }
  // Radio changes are serialized with Home Assistant controls on the main task.
  void set_advertising(bool on);
  // Returns a request ID immediately; /api/status acknowledges the authoritative
  // baseline once the main task has begun the attempt.
  CaptureStart start_capture();
  // Reboots the ESP32 shortly after the request has answered. Callable from
  // any task; the reboot itself runs on the main loop.
  void reboot();
  void set_port(uint16_t port) {
    port_ = port;
  }
  // HTTP Basic auth, required by the config schema. The expected header value
  // is precomputed once here so request handling is a constant-time string
  // compare with no base64 decoding. This is not secrecy at rest: base64 is
  // reversible, and the username and password are compiled into the firmware
  // image as the arguments to this call.
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
  ClientThrottles& client_throttles() {
    return client_throttles_;
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
  ClientThrottles client_throttles_;
  std::string expected_auth_;  // "Basic <base64>"; empty refuses every request
  std::atomic<uint32_t> last_request_ms_ { 0 };
  std::atomic<bool> profile_change_pending_ { false };

  httpd_handle_t server_ { nullptr };
  SemaphoreHandle_t snapshot_mutex_ { nullptr };
  WizardSnapshot snapshot_;
  CaptureStart capture_start_;  // Protected by snapshot_mutex_, including HTTP writes
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
