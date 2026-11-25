#pragma once

#include <cstdint>
#include <string>

namespace esphome {
namespace irk_capture {

// Custom/generated name cap: Samsung One UI 7 drops longer names from its
// Bluetooth list when the advertisement carries a single service UUID.
static constexpr size_t BLE_NAME_MAX_LEN = 12;
// 13 bytes; Keyboard puts the name in the scan response (29-byte budget),
// not the single-UUID advertisement packet.
static constexpr char KEYBOARD_DEFAULT_NAME[] = "Logitech K380";
// Used when no usable name is configured.
static constexpr char DEFAULT_BLE_NAME[] = "IRK Capture";

// Generated name for "Refresh BLE Identity", e.g. "IRK HR 7F3A": 2-char
// profile abbrev plus the active address's low two octets (matching the
// Effective MAC suffix). Rotating the name with the address prevents iOS
// from hiding a device whose previous name it has cached.
inline std::string identity_name(const char* abbrev, uint8_t hi, uint8_t lo) {
  static const char* const HEXU = "0123456789ABCDEF";
  // "IRK " + 2 + " " + 4 hex == 11. Clamping the abbreviation keeps the
  // 12-byte guarantee true for any caller rather than only the two profiles
  // that exist today.
  std::string name = "IRK ";
  for (size_t i = 0; abbrev != nullptr && abbrev[i] != '\0' && i < 2; i++) name += abbrev[i];
  name += ' ';
  name += HEXU[(hi >> 4) & 0x0F];
  name += HEXU[hi & 0x0F];
  name += HEXU[(lo >> 4) & 0x0F];
  name += HEXU[lo & 0x0F];
  return name;
}

}  // namespace irk_capture
}  // namespace esphome
