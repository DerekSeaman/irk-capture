#pragma once

// Pure helpers for the wizard's HTTP layer: base64, credential comparison,
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

// Accumulate differences across the full expected credential length without
// stopping at the first mismatch. Fold a length mismatch into the volatile
// accumulator and use a safe fallback byte past the supplied input's end.
inline bool secure_equals(const std::string& a, const std::string& b) {
  volatile uint8_t diff = (a.size() != b.size()) ? 1 : 0;
  static const uint8_t kZero = 0;
  const uint8_t* const ap = a.empty() ? &kZero : reinterpret_cast<const uint8_t*>(a.data());
  const size_t alen = a.size();
  for (size_t i = 0; i < b.size(); i++) {
    const uint8_t av = ap[i < alen ? i : 0];
    diff = static_cast<uint8_t>(diff | static_cast<uint8_t>(av ^ static_cast<uint8_t>(b[i])));
  }
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

// Parses "Keyboard" or "Heart Sensor"; rejects unknown values instead of
// defaulting because a wrong profile switch triggers a reboot and loses the session.
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

// Per-client throttle table so one guessing host locks out only itself.
// Full table evicts the least recently seen idle client; recent failures keep
// their slot for one lockout window. If all slots have recent failures, extra
// clients share the overflow throttle.
struct ClientThrottles {
  static constexpr size_t SLOTS = 4;

  struct Slot {
    bool used { false };
    uint32_t key { 0 };
    uint32_t last_seen { 0 };
    AuthThrottle throttle;
  };
  Slot slots[SLOTS];
  AuthThrottle overflow;

  AuthThrottle& for_client(uint32_t key, uint32_t now) {
    Slot* target = nullptr;
    for (auto& slot : slots) {
      if (slot.used && slot.key == key) {
        slot.last_seen = now;
        return slot.throttle;
      }
      if (!slot.used && target == nullptr) target = &slot;
    }
    if (target == nullptr) {
      for (auto& slot : slots) {
        if (slot.throttle.locked(now)) continue;
        if (slot.throttle.failures != 0) {
          if ((uint32_t) (now - slot.last_seen) < AuthThrottle::LOCKOUT_MS) continue;
          slot.throttle.ok();
        }
        // Wraparound-safe "seen longer ago".
        if (target == nullptr ||
            (uint32_t) (now - slot.last_seen) > (uint32_t) (now - target->last_seen)) {
          target = &slot;
        }
      }
    }
    if (target == nullptr) return overflow;
    *target = Slot {};
    target->used = true;
    target->key = key;
    target->last_seen = now;
    return target->throttle;
  }
};

//======================== Minimal JSON ========================
// Validate the entire JSON object, then find the requested top-level member.
// Lookup ignores key-like strings inside values and rejects duplicates of
// the requested key.

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

// Parses a JSON string at pos (leaving pos past the closing quote); pass
// out=nullptr to skip. Decodes all JSON escapes (including surrogate pairs)
// and rejects unclosed strings, unknown escapes, lone surrogates, and <0x20.
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

// Validates and skips one complete JSON value. Bound nesting independently of
// the HTTP body limit so malformed input cannot exhaust the server task stack.
inline bool json_skip_value(const std::string& body, size_t& pos, unsigned depth = 0) {
  if (pos >= body.size() || depth > 16) return false;
  if (body[pos] == '"') return json_read_string(body, pos, nullptr);
  if (body[pos] == '{' || body[pos] == '[') {
    const bool object = body[pos++] == '{';
    const char closing = object ? '}' : ']';
    json_skip_ws(body, pos);
    if (pos < body.size() && body[pos] == closing) {
      ++pos;
      return true;
    }
    for (;;) {
      if (object) {
        if (!json_read_string(body, pos, nullptr)) return false;
        json_skip_ws(body, pos);
        if (pos >= body.size() || body[pos++] != ':') return false;
        json_skip_ws(body, pos);
      }
      if (!json_skip_value(body, pos, depth + 1)) return false;
      json_skip_ws(body, pos);
      if (pos >= body.size()) return false;
      if (body[pos] == closing) {
        ++pos;
        return true;
      }
      if (body[pos++] != ',') return false;
      json_skip_ws(body, pos);
    }
  }
  for (const char* literal : { "true", "false", "null" }) {
    const size_t size = std::char_traits<char>::length(literal);
    if (body.compare(pos, size, literal) == 0) {
      pos += size;
      return true;
    }
  }
  // JSON number: optional sign, integer, optional fraction and exponent.
  if (body[pos] == '-') ++pos;
  if (pos >= body.size()) return false;
  if (body[pos] == '0')
    ++pos;
  else {
    if (body[pos] < '1' || body[pos] > '9') return false;
    do {
      ++pos;
    } while (pos < body.size() && body[pos] >= '0' && body[pos] <= '9');
  }
  if (pos < body.size() && body[pos] == '.') {
    const size_t first = ++pos;
    while (pos < body.size() && body[pos] >= '0' && body[pos] <= '9') ++pos;
    if (pos == first) return false;
  }
  if (pos < body.size() && (body[pos] == 'e' || body[pos] == 'E')) {
    ++pos;
    if (pos < body.size() && (body[pos] == '+' || body[pos] == '-')) ++pos;
    const size_t first = pos;
    while (pos < body.size() && body[pos] >= '0' && body[pos] <= '9') ++pos;
    if (pos == first) return false;
  }
  return true;
}

// Validate the entire object before returning a command member. A valid prefix
// must never execute a truncated document. Duplicate command keys are ambiguous.
inline bool json_find_value_start(const std::string& body, const char* key, size_t& pos) {
  size_t end = 0;
  json_skip_ws(body, end);
  if (end >= body.size() || body[end] != '{') return false;
  size_t at = end + 1;
  if (!json_skip_value(body, end)) return false;
  json_skip_ws(body, end);
  if (end != body.size()) return false;
  bool found = false;
  for (;;) {
    json_skip_ws(body, at);
    if (body[at] == '}') return found;
    std::string name;
    if (!json_read_string(body, at, &name)) return false;
    json_skip_ws(body, at);
    ++at;  // Colon, already validated above.
    json_skip_ws(body, at);
    if (name == key) {
      if (found) return false;
      found = true;
      pos = at;
    }
    if (!json_skip_value(body, at)) return false;
    json_skip_ws(body, at);
    if (body[at] == '}') return found;
    ++at;  // Comma, already validated above.
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

// Extracts a complete quoted string for key; leaves out untouched on failure.
// Callers validate/sanitize the decoded value before use.
inline bool json_extract_string(const std::string& body, const char* key, std::string& out) {
  size_t pos;
  if (!json_find_value_start(body, key, pos)) return false;
  std::string result;
  if (!json_read_string(body, pos, &result) || !json_value_ends(body, pos)) return false;
  out = result;
  return true;
}

// Escapes quotes, backslashes, and control chars (<0x20) so a raw control
// character cannot make a JSON response unparseable.
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
