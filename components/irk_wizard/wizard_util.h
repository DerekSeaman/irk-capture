#pragma once

// Pure helpers for the wizard's HTTP layer: base64, a constant-time compare,
// and just enough JSON handling for the short single-field bodies the page
// sends. Deliberately free of ESP-IDF and ESPHome includes so the same code
// can be compiled and exercised on a host (see the workflow's host test job).

#include <cstddef>
#include <cstdint>
#include <string>

namespace esphome {
namespace irk_wizard {

inline std::string base64_encode(const std::string& in) {
  static const char* const TBL = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((in.size() + 2) / 3) * 4);
  size_t i = 0;
  while (i + 2 < in.size()) {
    uint32_t n = ((uint32_t) (uint8_t) in[i] << 16) | ((uint32_t) (uint8_t) in[i + 1] << 8) |
                 (uint32_t) (uint8_t) in[i + 2];
    out += TBL[(n >> 18) & 63];
    out += TBL[(n >> 12) & 63];
    out += TBL[(n >> 6) & 63];
    out += TBL[n & 63];
    i += 3;
  }
  if (i < in.size()) {
    const bool two = (i + 1 < in.size());
    uint32_t n = (uint32_t) (uint8_t) in[i] << 16;
    if (two) n |= (uint32_t) (uint8_t) in[i + 1] << 8;
    out += TBL[(n >> 18) & 63];
    out += TBL[(n >> 12) & 63];
    out += two ? TBL[(n >> 6) & 63] : '=';
    out += '=';
  }
  return out;
}

// Length-independent compare so a wrong password can't be recovered by
// timing how far the comparison got.
inline bool secure_equals(const std::string& a, const std::string& b) {
  if (a.size() != b.size()) return false;
  uint8_t diff = 0;
  for (size_t i = 0; i < a.size(); i++) diff |= (uint8_t) (a[i] ^ b[i]);
  return diff == 0;
}

// True when value is "application/json", optionally followed by parameters
// (";charset=utf-8"). A prefix match would also accept "application/jsonx".
inline bool is_json_content_type(const char* value) {
  if (value == nullptr) return false;
  static const char* const WANT = "application/json";
  size_t i = 0;
  for (; WANT[i] != '\0'; i++) {
    char c = value[i];
    if (c >= 'A' && c <= 'Z') c = (char) (c - 'A' + 'a');
    if (c != WANT[i]) return false;
  }
  // Only a parameter separator or trailing whitespace may follow.
  while (value[i] == ' ' || value[i] == '\t') i++;
  return value[i] == '\0' || value[i] == ';';
}

// A name a phone has not seen before. iOS in particular hides an accessory
// whose name it already knows, even when the address is new, so "try again"
// needs a different name rather than just a different MAC. Fixed at 12
// bytes: irk_capture caps BLE names there so Samsung phones can see them.
inline std::string fresh_ble_name(uint32_t rnd) {
  static const char* const HEXU = "0123456789ABCDEF";
  std::string name = "IRK Cap ";
  for (int shift = 12; shift >= 0; shift -= 4) name += HEXU[(rnd >> shift) & 0xF];
  return name;
}

// Slows down password guessing: after MAX_FAILURES bad attempts in a row,
// everything is refused for LOCKOUT_MS. A success clears the count.
struct AuthThrottle {
  static constexpr uint8_t MAX_FAILURES = 5;
  static constexpr uint32_t LOCKOUT_MS = 30000;

  uint8_t failures { 0 };
  bool lock_active { false };
  uint32_t locked_at { 0 };

  // Wraparound-safe: compares elapsed time, never absolute timestamps.
  bool locked(uint32_t now) {
    if (lock_active && (uint32_t) (now - locked_at) >= LOCKOUT_MS) {
      lock_active = false;
      failures = 0;
    }
    return lock_active;
  }
  void fail(uint32_t now) {
    if (failures < MAX_FAILURES) failures++;
    if (failures >= MAX_FAILURES) {
      lock_active = true;
      locked_at = now;
    }
  }
  void ok() {
    failures = 0;
    lock_active = false;
  }
};

inline bool json_find_value_start(const std::string& body, const char* key, size_t& pos) {
  std::string needle = std::string("\"") + key + "\"";
  size_t key_pos = body.find(needle);
  if (key_pos == std::string::npos) return false;
  size_t colon = body.find(':', key_pos + needle.size());
  if (colon == std::string::npos) return false;
  pos = colon + 1;
  while (pos < body.size() && (body[pos] == ' ' || body[pos] == '\t')) pos++;
  return pos < body.size();
}

inline bool json_extract_bool(const std::string& body, const char* key, bool& out) {
  size_t pos;
  if (!json_find_value_start(body, key, pos)) return false;
  if (body.compare(pos, 4, "true") == 0) {
    out = true;
    return true;
  }
  if (body.compare(pos, 5, "false") == 0) {
    out = false;
    return true;
  }
  return false;
}

// Extracts a quoted string value and unescapes \" and \\ only - the two
// escapes JSON.stringify() can produce for the short, sanitized strings the
// wizard's own page sends. Values are re-sanitized on the irk_capture side
// before they are stored or echoed back.
inline bool json_extract_string(const std::string& body, const char* key, std::string& out) {
  size_t pos;
  if (!json_find_value_start(body, key, pos)) return false;
  if (pos >= body.size() || body[pos] != '"') return false;
  pos++;
  std::string result;
  while (pos < body.size() && body[pos] != '"') {
    if (body[pos] == '\\' && pos + 1 < body.size()) {
      pos++;
      result += body[pos];
    } else {
      result += body[pos];
    }
    pos++;
  }
  out = result;
  return true;
}

inline std::string json_escape(const std::string& in) {
  std::string out;
  out.reserve(in.size() + 4);
  for (char c : in) {
    if (c == '"' || c == '\\') out += '\\';
    out += c;
  }
  return out;
}

}  // namespace irk_wizard
}  // namespace esphome
