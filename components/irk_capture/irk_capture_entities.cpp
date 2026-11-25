// Home Assistant entity glue: the switch, button, text, and select classes
// that forward commands to IRKCaptureComponent.

#include "irk_capture_internal.h"

// ESP32-only implementation - requires Bluetooth hardware
#ifdef USE_ESP32

namespace esphome {
namespace irk_capture {

//======================== Entity impls ========================

// True when an entity may forward a command to its parent. A component that
// failed setup no longer runs loop(), so a command would be accepted and then
// nothing would happen; say why instead, once per press.
static bool parent_usable(IRKCaptureComponent* parent, const char* control) {
  if (parent == nullptr) {
    ESP_LOGE(TAG, "%s is not attached to an irk_capture component", control);
    return false;
  }
  if (parent->is_failed()) {
    ESP_LOGE(TAG, "%s ignored: IRK Capture failed to start. See the log from boot, then reboot.",
             control);
    return false;
  }
  return true;
}

void IRKCaptureText::control(const std::string& value) {
  if (!parent_usable(parent_, "BLE Device Name")) return;
  // Normalization and publication belong to the parent so HA and other
  // callers share the same ordering with generated identity names.
  if (!parent_->update_ble_name(value)) {
    const std::string current = parent_->get_ble_name();
    ESP_LOGW(TAG, "BLE name update rejected; keeping '%s'", current.c_str());
    return;
  }
  ESP_LOGI(TAG, "BLE name accepted: %s", parent_->get_ble_name().c_str());
}

void IRKCaptureSwitch::write_state(bool state) {
  if (!parent_usable(parent_, "BLE Advertising")) {
    publish_state(false);  // Nothing is advertising, whatever was asked for.
    return;
  }
  parent_->set_advertising_requested(state);
}

void IRKCaptureButton::press_action() {
  if (!parent_usable(parent_, "Refresh BLE Identity")) return;
  ESP_LOGI(TAG, "BLE identity refresh requested from Home Assistant");
  parent_->refresh_identity();
}

void IRKCaptureForgetBondsButton::press_action() {
  if (!parent_usable(parent_, "Forget All Bonds")) return;
  ESP_LOGI(TAG, "Forget All Bonds requested from Home Assistant");
  parent_->forget_all_bonds();
}

void IRKCaptureStopAfterCaptureSwitch::write_state(bool state) {
  if (!parent_usable(parent_, "Stop Advertising After Capture")) return;
  // set_stop_after_capture() stages the entity publish; publishing here as
  // well would send every change twice.
  parent_->set_stop_after_capture(state);
}

void IRKCaptureLabelText::control(const std::string& value) {
  if (!parent_usable(parent_, "Next Capture Label")) return;
  // Stages the publish of the accepted (sanitized) value; see above.
  parent_->set_next_capture_label(value);
}

void IRKCaptureSelect::control(const std::string& value) {
  // A failed component still accepts a profile change: with a GATT table that
  // would not register, switching profile and rebooting is the way out.
  if (parent_ == nullptr) {
    ESP_LOGE(TAG, "BLE Profile is not attached to an irk_capture component");
    return;
  }
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

}  // namespace irk_capture
}  // namespace esphome

#endif  // USE_ESP32
