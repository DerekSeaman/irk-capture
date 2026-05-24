# Release Notes v1.5.14

This release note reflects changes from `v1.5.13` to `v1.5.14`.

## Bug Fix

- **Fixed pairing failure with Samsung Galaxy Watch (Wear OS 5)**: Despite the v1.5.13 fix suppressing duplicate Security Request PDUs, Galaxy Watch Wear OS 5 still failed with SMP error 0x07 "Command Not Supported" approximately 6 seconds into the pairing handshake. Root cause: NimBLE was negotiating LE Secure Connections (SC) pairing, and the Galaxy Watch — despite advertising SC support in its Pairing Request — rejected the SC-specific Pairing Public Key exchange (opcode 0x0C) with a Pairing Failed response. This is a Samsung firmware bug.
  - Changed `sm_sc` from `1` to `0` at runtime, switching from Secure Connections to legacy pairing universally. Legacy pairing still distributes the ID key (IRK) in the key distribution phase, so IRK capture is fully preserved.
  - Apple (iOS/watchOS) and Android devices negotiate legacy pairing correctly when the peripheral advertises `sc=0`. IRK capture on previously working devices is unaffected.
  - `CONFIG_BT_NIMBLE_SM_SC` remains compiled in (`y`) so NimBLE can handle SC PDUs from peers that require it.

## Files Updated

- `components/irk_capture/irk_capture.cpp`
- `ESPHome Devices/irk-capture-base.yaml`
- `ESPHome Devices/irk-capture-full.yaml`
- `RELEASE_NOTES_v1.5.14.md`
