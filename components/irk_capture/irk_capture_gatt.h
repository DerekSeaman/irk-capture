#pragma once

// Private GATT UUIDs shared only by registration, advertising and GAP logging.
#include "irk_capture.h"

#ifdef USE_ESP32
namespace esphome {
namespace irk_capture {

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

}  // namespace irk_capture
}  // namespace esphome
#endif
