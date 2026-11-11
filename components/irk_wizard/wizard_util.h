#pragma once

// Pure helpers for the wizard's HTTP layer: base64, a constant-time compare,
// and just enough JSON handling for the short, flat bodies the page sends.
// Deliberately free of ESP-IDF and ESPHome includes so the same code can be
// compiled and exercised on a host (see the workflow's host test job).

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

// Maps a profile name from the page to whether it is the Keyboard profile.
// Anything else is rejected rather than defaulted: a profile switch reboots the
// device and discards the capture session, so a typo, or a newer page talking
// to an older device, must never switch profiles by falling through.
inline bool parse_profile_name(const std::string& name, bool& keyboard) {
  if (name == "Keyboard") {
    keyboard = true;
    return true;
  }
  if (name == "Heart Sensor") {
    keyboard = false;
    return true;
  }
  return false;
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

//======================== Minimal JSON ========================
// The page sends one small, flat JSON object per request, and these helpers
// read one top-level member of it. They walk the object instead of searching
// the text, so a key name inside another member's value is never mistaken for
// a member. Anything malformed before or at the requested value is rejected
// rather than guessed at: a wrong guess here becomes a wrong command.

inline void json_skip_ws(const std::string& body, size_t& pos) {
  while (pos < body.size() &&
         (body[pos] == ' ' || body[pos] == '\t' || body[pos] == '\r' || body[pos] == '\n')) {
    pos++;
  }
}

inline bool json_read_hex4(const std::string& body, size_t pos, uint32_t& value) {
  if (pos + 4 > body.size()) return false;
  value = 0;
  for (size_t i = 0; i < 4; i++) {
    const char c = body[pos + i];
    uint32_t digit;
    if (c >= '0' && c <= '9') {
      digit = (uint32_t) (c - '0');
    } else if (c >= 'a' && c <= 'f') {
      digit = (uint32_t) (c - 'a') + 10;
    } else if (c >= 'A' && c <= 'F') {
      digit = (uint32_t) (c - 'A') + 10;
    } else {
      return false;
    }
    value = (value << 4) | digit;
  }
  return true;
}

inline void json_append_utf8(uint32_t code_point, std::string& out) {
  if (code_point < 0x80) {
    out += (char) code_point;
  } else if (code_point < 0x800) {
    out += (char) (0xC0 | (code_point >> 6));
    out += (char) (0x80 | (code_point & 0x3F));
  } else if (code_point < 0x10000) {
    out += (char) (0xE0 | (code_point >> 12));
    out += (char) (0x80 | ((code_point >> 6) & 0x3F));
    out += (char) (0x80 | (code_point & 0x3F));
  } else {
    out += (char) (0xF0 | (code_point >> 18));
    out += (char) (0x80 | ((code_point >> 12) & 0x3F));
    out += (char) (0x80 | ((code_point >> 6) & 0x3F));
    out += (char) (0x80 | (code_point & 0x3F));
  }
}

// Reads the string whose opening quote is at pos and leaves pos just after its
// closing quote. Pass a null out to skip the string. Every JSON escape is
// decoded; reducing an escape to its letter would turn a pasted tab ("\t")
// into a stray "t". Returns false for a string that never closes, an unknown
// escape, a lone surrogate or a raw control character.
inline bool json_read_string(const std::string& body, size_t& pos, std::string* out) {
  if (pos >= body.size() || body[pos] != '"') return false;
  pos++;
  while (pos < body.size()) {
    const char c = body[pos++];
    if (c == '"') return true;
    if ((unsigned char) c < 0x20) return false;
    if (c != '\\') {
      if (out != nullptr) *out += c;
      continue;
    }
    if (pos >= body.size()) return false;
    const char escape = body[pos++];
    char decoded = '\0';
    switch (escape) {
      case '"':
      case '\\':
      case '/':
        decoded = escape;
        break;
      case 'b':
        decoded = '\b';
        break;
      case 'f':
        decoded = '\f';
        break;
      case 'n':
        decoded = '\n';
        break;
      case 'r':
        decoded = '\r';
        break;
      case 't':
        decoded = '\t';
        break;
      case 'u': {
        uint32_t code_point;
        if (!json_read_hex4(body, pos, code_point)) return false;
        pos += 4;
        if (code_point >= 0xD800 && code_point <= 0xDBFF) {
          // A high surrogate is only half a character; its low half must follow.
          uint32_t low;
          if (pos + 6 > body.size() || body[pos] != '\\' || body[pos + 1] != 'u' ||
              !json_read_hex4(body, pos + 2, low) || low < 0xDC00 || low > 0xDFFF) {
            return false;
          }
          pos += 6;
          code_point = 0x10000 + ((code_point - 0xD800) << 10) + (low - 0xDC00);
        } else if (code_point >= 0xDC00 && code_point <= 0xDFFF) {
          return false;
        }
        if (out != nullptr) json_append_utf8(code_point, *out);
        continue;
      }
      default:
        return false;
    }
    if (out != nullptr) *out += decoded;
  }
  return false;  // No closing quote.
}

// Skips the value at pos: a string, a nested object or array, or a bare scalar.
inline bool json_skip_value(const std::string& body, size_t& pos) {
  if (pos >= body.size()) return false;
  if (body[pos] == '"') return json_read_string(body, pos, nullptr);
  if (body[pos] == '{' || body[pos] == '[') {
    int depth = 0;
    while (pos < body.size()) {
      const char c = body[pos];
      if (c == '"') {
        if (!json_read_string(body, pos, nullptr)) return false;
        continue;
      }
      pos++;
      if (c == '{' || c == '[') {
        depth++;
      } else if ((c == '}' || c == ']') && --depth == 0) {
        return true;
      }
    }
    return false;
  }
  const size_t start = pos;
  while (pos < body.size() && body[pos] != ',' && body[pos] != '}' && body[pos] != ' ' &&
         body[pos] != '\t' && body[pos] != '\r' && body[pos] != '\n') {
    pos++;
  }
  return pos > start;
}

// Finds the top-level member named key and sets pos to the start of its value.
// The first member with that name wins.
inline bool json_find_value_start(const std::string& body, const char* key, size_t& pos) {
  size_t at = 0;
  json_skip_ws(body, at);
  if (at >= body.size() || body[at] != '{') return false;
  at++;
  for (;;) {
    json_skip_ws(body, at);
    std::string name;
    if (!json_read_string(body, at, &name)) return false;  // Also the closing brace.
    json_skip_ws(body, at);
    if (at >= body.size() || body[at] != ':') return false;
    at++;
    json_skip_ws(body, at);
    if (at >= body.size()) return false;
    if (name == key) {
      pos = at;
      return true;
    }
    if (!json_skip_value(body, at)) return false;
    json_skip_ws(body, at);
    if (at >= body.size() || body[at] != ',') return false;
    at++;
  }
}

// True when a value that ends at pos is followed by the next member or by the
// end of the object. Without this, "true1" would be read as true.
inline bool json_value_ends(const std::string& body, size_t pos) {
  json_skip_ws(body, pos);
  return pos < body.size() && (body[pos] == ',' || body[pos] == '}');
}

inline bool json_extract_bool(const std::string& body, const char* key, bool& out) {
  size_t pos;
  if (!json_find_value_start(body, key, pos)) return false;
  if (body.compare(pos, 4, "true") == 0 && json_value_ends(body, pos + 4)) {
    out = true;
    return true;
  }
  if (body.compare(pos, 5, "false") == 0 && json_value_ends(body, pos + 5)) {
    out = false;
    return true;
  }
  return false;
}

// Extracts a complete, quoted string value. Values are re-sanitized on the
// irk_capture side before they are stored or echoed back. out is left
// untouched unless the whole value is valid.
inline bool json_extract_string(const std::string& body, const char* key, std::string& out) {
  size_t pos;
  if (!json_find_value_start(body, key, pos)) return false;
  std::string result;
  if (!json_read_string(body, pos, &result) || !json_value_ends(body, pos)) return false;
  out = result;
  return true;
}

// Escapes a value for a JSON string. Control characters are escaped as well:
// one raw control character makes the whole status response unparseable, which
// the page can only report as the device being unreachable.
inline std::string json_escape(const std::string& in) {
  static const char* const HEX_DIGITS = "0123456789abcdef";
  std::string out;
  out.reserve(in.size() + 4);
  for (char c : in) {
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if ((unsigned char) c < 0x20) {
      out += "\\u00";
      out += HEX_DIGITS[(c >> 4) & 0x0F];
      out += HEX_DIGITS[c & 0x0F];
    } else {
      out += c;
    }
  }
  return out;
}

}  // namespace irk_wizard
}  // namespace esphome
