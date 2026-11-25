#pragma once
// Checks a captured IRK against the address the peer connected from, and names
// the kind of address that was. Include after the NimBLE host headers, which
// define ble_addr_t and the BLE_ADDR_* types.

#include <cstdint>

namespace esphome {
namespace irk_capture {

// The kinds of Bluetooth device address (Core Spec Vol 6, Part B, 1.3). A random
// address is told apart by the top two bits of its most significant byte, which
// NimBLE stores last.
enum class AddressKind : uint8_t {
  UNKNOWN,                 // No address, or a type this component does not know
  PUBLIC,                  // Fixed, assigned by the manufacturer
  STATIC_RANDOM,           // Fixed until the device resets or chooses otherwise
  RESOLVABLE_PRIVATE,      // Rotates; derived from the device's IRK
  NON_RESOLVABLE_PRIVATE,  // Rotates; no key can follow it
};

inline AddressKind classify_address(const ble_addr_t& addr) {
  if (addr.type == BLE_ADDR_PUBLIC || addr.type == BLE_ADDR_PUBLIC_ID) return AddressKind::PUBLIC;
  if (addr.type != BLE_ADDR_RANDOM && addr.type != BLE_ADDR_RANDOM_ID) return AddressKind::UNKNOWN;
  switch (addr.val[5] >> 6) {
    case 0x3:
      return AddressKind::STATIC_RANDOM;
    case 0x1:
      return AddressKind::RESOLVABLE_PRIVATE;
    case 0x0:
      return AddressKind::NON_RESOLVABLE_PRIVATE;
    default:
      return AddressKind::UNKNOWN;  // 0b10 is reserved
  }
}

// Whether a captured IRK generated the address the peer connected from.
enum class IrkCheck : uint8_t {
  UNCHECKED,       // The connection address is unknown, or the AES step failed
  NOT_RESOLVABLE,  // The peer connected from an address no IRK generates
  VERIFIED,        // The IRK resolves the connection address
  MISMATCH,        // The address is resolvable, but not with this IRK
};

// String form for irk_check_sensor_, on_capture, and wizard JSON (part of the
// external contract). Only a resolved RPA is "verified"; unchecked is not a failure.
inline const char* irk_check_name(IrkCheck check) {
  switch (check) {
    case IrkCheck::VERIFIED:
      return "verified";
    case IrkCheck::MISMATCH:
      return "mismatch";
    case IrkCheck::UNCHECKED:
    case IrkCheck::NOT_RESOLVABLE:
      break;
  }
  return "unverified";
}

// AES-128 on one block, with key, input and output most significant byte
// first: the Core spec's e() function. Returns false if encryption failed.
// Defined in irk_capture_session.cpp.
bool aes128_encrypt_be(const uint8_t key[16], const uint8_t plaintext[16], uint8_t out[16]);

// Resolves the peer's connection address with a captured IRK, using the random
// address hash function ah() (Core Spec Vol 3, Part H, 2.2.2). `irk` is the key
// as NimBLE stores it, least significant byte first, so in the reverse order of
// its hex form. `connection_addr` may be null when the address is not known.
//
// An RPA is prand (the top 24 bits) followed by hash (the low 24 bits), where
// hash = ah(IRK, prand) = e(IRK, padding || prand) mod 2^24. NimBLE stores the
// address least significant byte first: hash is val[0..2] and prand val[3..5].
inline IrkCheck check_irk_against_address(const uint8_t irk[16],
                                          const ble_addr_t* connection_addr) {
  if (connection_addr == nullptr) return IrkCheck::UNCHECKED;
  const AddressKind kind = classify_address(*connection_addr);
  if (kind == AddressKind::UNKNOWN) return IrkCheck::UNCHECKED;
  if (kind != AddressKind::RESOLVABLE_PRIVATE) return IrkCheck::NOT_RESOLVABLE;

  const uint8_t* addr = connection_addr->val;
  uint8_t key[16];
  for (int i = 0; i < 16; ++i) key[i] = irk[15 - i];
  uint8_t block[16] {};  // 104 bits of zero padding, then prand
  block[13] = addr[5];
  block[14] = addr[4];
  block[15] = addr[3];
  uint8_t out[16];
  if (!aes128_encrypt_be(key, block, out)) return IrkCheck::UNCHECKED;
  const bool match = out[13] == addr[2] && out[14] == addr[1] && out[15] == addr[0];
  return match ? IrkCheck::VERIFIED : IrkCheck::MISMATCH;
}

}  // namespace irk_capture
}  // namespace esphome
