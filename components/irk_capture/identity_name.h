#pragma once

#include <cstdint>
#include <string>

namespace esphome {
namespace irk_capture {

// Maximum advertised name length. A Samsung Galaxy running One UI 7 drops a
// longer name from its Bluetooth list when the advertisement carries a single
// service UUID, so sanitize_ble_name() truncates to this and the names built
// here stay inside it by construction.
static constexpr size_t BLE_NAME_MAX_LEN = 12;

// The name a "Refresh BLE Identity" press adopts, e.g. "IRK HR 7F3A".
//
// 'abbrev' names the active profile, so the phone's Bluetooth list says which
// one is advertising rather than making the user remember. The suffix is the
// low two octets of the address actually in use, which is what the Effective
// MAC sensor ends with - a user comparing the two can see they match.
//
// Every press produces a name the phone has not seen before, which is the part
// that matters: iOS hides an accessory whose name it already knows even on a
// fresh address, so rotating the address alone leaves the device invisible.
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
