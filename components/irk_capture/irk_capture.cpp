#include "irk_capture.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#include <nvs_flash.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <host/ble_store.h>
#include <store/config/ble_store_config.h>

// Some ESP-IDF 5.x package variants omit this prototype from headers; declare it explicitly.
extern "C" int ble_store_config_init(void);

namespace esphome {
namespace irk_capture {

static const char *const TAG = "irk_capture";
static constexpr char VERSION[] = "1.2";
static constexpr char HEX[] = "0123456789abcdef";

// Timing configuration (all values in ms)
struct TimingConfig {
    static constexpr uint32_t LOOP_MIN_INTERVAL_MS = 50;
    static constexpr uint32_t HR_NOTIFY_INTERVAL_MS = 1000;
    static constexpr uint32_t ENC_TO_FIRST_TRY_DELAY_MS = 1000;
    static constexpr uint32_t ENC_TRY_INTERVAL_MS = 1000;
    static constexpr uint32_t ENC_GIVE_UP_AFTER_MS = 45000;
    static constexpr uint32_t POST_DISC_DELAY_MS = 800;
    static constexpr uint32_t ENC_LATE_READ_DELAY_MS = 5000;
};

// GATT service and characteristic UUIDs
static constexpr uint16_t UUID_SVC_HEART_RATE = 0x180D;
static constexpr uint16_t UUID_CHR_HEART_RATE_MEASUREMENT = 0x2A37;
static constexpr uint16_t UUID_SVC_DEVICE_INFO = 0x180A;
static constexpr uint16_t UUID_CHR_MANUFACTURER_NAME = 0x2A29;
static constexpr uint16_t UUID_CHR_MODEL_NUMBER = 0x2A24;
static constexpr uint16_t UUID_SVC_BATTERY = 0x180F;
static constexpr uint16_t UUID_CHR_BATTERY_LEVEL = 0x2A19;

// BLE Appearance values
static constexpr uint16_t APPEARANCE_HEART_RATE_SENSOR = 0x0340;

//======================== IRK lifecycle (for readers) ========================
/*
Connect → Initiate security
ENC_CHANGE → immediate IRK read from store; if available: publish & disconnect (1.0 behavior).
             If ENC fails (status != 0), delete that peer's keys and retry security once (self-heal).
DISCONNECT → immediate store read; schedule delayed read at +800ms; restart advertising
While connected (post ENC) → poll every 1s starting at +2s, up to 45s, then disconnect when IRK captured
All address reporting uses the peer identity address; IRK hex is reversed for parity with Arduino output.

Why this lifecycle: Maintains compatibility with v1.0 behavior (immediate disconnect after capture)
while adding robustness for timing variations across different BLE peer implementations.
*/

//======================== UUIDs ========================

static const ble_uuid16_t UUID_SVC_HR       = BLE_UUID16_INIT(UUID_SVC_HEART_RATE);
static const ble_uuid16_t UUID_CHR_HR_MEAS  = BLE_UUID16_INIT(UUID_CHR_HEART_RATE_MEASUREMENT);

static const ble_uuid16_t UUID_SVC_DEVINFO  = BLE_UUID16_INIT(UUID_SVC_DEVICE_INFO);
static const ble_uuid16_t UUID_CHR_MANUF    = BLE_UUID16_INIT(UUID_CHR_MANUFACTURER_NAME);
static const ble_uuid16_t UUID_CHR_MODEL    = BLE_UUID16_INIT(UUID_CHR_MODEL_NUMBER);

static const ble_uuid16_t UUID_SVC_BAS      = BLE_UUID16_INIT(UUID_SVC_BATTERY);
static const ble_uuid16_t UUID_CHR_BATT_LVL = BLE_UUID16_INIT(UUID_CHR_BATTERY_LEVEL);

// Optional protected service/characteristic to force pairing via READ_ENC
static const ble_uuid128_t UUID_SVC_PROT = BLE_UUID128_INIT(
    0x12,0x34,0x56,0x78, 0x90,0xAB,0xCD,0xEF, 0xFE,0xDC,0xBA,0x09, 0x87,0x65,0x43,0x21);
static const ble_uuid128_t UUID_CHR_PROT = BLE_UUID128_INIT(
    0x21,0x43,0x65,0x87, 0x09,0xBA,0xDC,0xFE, 0xEF,0xCD,0xAB,0x90, 0x78,0x56,0x34,0x12);

//======================== Forward decls ========================

static int chr_read_devinfo(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg);
static int chr_read_batt(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg);
static int chr_read_hr(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg);
static int chr_read_protected(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

// GAP event handlers (forward decls)
class IRKCaptureComponent;
static int handle_gap_connect(IRKCaptureComponent *self, struct ble_gap_event *ev);
static int handle_gap_disconnect(IRKCaptureComponent *self, struct ble_gap_event *ev);
static int handle_gap_enc_change(IRKCaptureComponent *self, struct ble_gap_event *ev);
static int handle_gap_repeat_pairing(IRKCaptureComponent *self, struct ble_gap_event *ev);

//======================== Small utilities (readability only) ========================

static inline uint32_t now_ms() { return (uint32_t)(esp_timer_get_time() / 1000ULL); }

// Generic hex formatter for byte arrays
static std::string to_hex_str(const uint8_t *data, size_t len, bool reverse = false) {
    std::string out;
    out.reserve(len * 2);
    if (reverse) {
        for (int i = (int)len - 1; i >= 0; --i) {
            uint8_t c = data[i];
            out.push_back(HEX[(c >> 4) & 0xF]);
            out.push_back(HEX[c & 0xF]);
        }
    } else {
        for (size_t i = 0; i < len; ++i) {
            uint8_t c = data[i];
            out.push_back(HEX[(c >> 4) & 0xF]);
            out.push_back(HEX[c & 0xF]);
        }
    }
    return out;
}

// Reverse byte order for hex output (matches Arduino BLE library convention for IRK display)
static std::string bytes_to_hex_rev(const uint8_t *data, size_t len) {
    return to_hex_str(data, len, true);
}

static std::string addr_to_str(const ble_addr_t &a) {
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             a.val[5], a.val[4], a.val[3], a.val[2], a.val[1], a.val[0]);
    return std::string(buf);
}

static bool addr_is_zero(const ble_addr_t &a) {
    return a.type == 0 && a.val[0] == 0 && a.val[1] == 0 && a.val[2] == 0 &&
           a.val[3] == 0 && a.val[4] == 0 && a.val[5] == 0;
}

static bool is_encrypted(uint16_t conn_handle) {
    struct ble_gap_conn_desc d;
    if (ble_gap_conn_find(conn_handle, &d) == 0) {
        return d.sec_state.encrypted;
    }
    return false;
}

static void hr_measurement_sample(uint8_t *buf, size_t *len) {
    buf[0] = 0x00;  // flags: HR value format UINT8, no sensor contact, no energy expended
    buf[1] = (uint8_t)(60 + (esp_random() % 40));  // 60-99 bpm
    *len = 2;
}

static void log_conn_desc(uint16_t conn_handle) {
    struct ble_gap_conn_desc d;
    if (ble_gap_conn_find(conn_handle, &d) == 0) {
        // Security state
        ESP_LOGI(TAG, "sec: enc=%d bonded=%d auth=%d key_size=%u",
                 d.sec_state.encrypted, d.sec_state.bonded, d.sec_state.authenticated, d.sec_state.key_size);

        // Addresses
        ESP_LOGI(TAG, "peer ota=%s type=%d", addr_to_str(d.peer_ota_addr).c_str(), d.peer_ota_addr.type);
        ESP_LOGI(TAG, "peer id =%s type=%d", addr_to_str(d.peer_id_addr).c_str(), d.peer_id_addr.type);

        // Connection parameters (helpful for Android watch debugging)
        ESP_LOGD(TAG, "conn params: interval=%u latency=%u timeout=%u supervision_timeout=%u",
                 d.conn_itvl, d.conn_latency, d.supervision_timeout, d.supervision_timeout);

        // Role and features
        ESP_LOGD(TAG, "role=%s our_ota=%s",
                 d.role == BLE_GAP_ROLE_MASTER ? "master" : "slave",
                 addr_to_str(d.our_ota_addr).c_str());
    }
}

static void log_sm_config() {
    ESP_LOGI(TAG, "SM config: bonding=%d mitm=%d sc=%d io_cap=%d our_key_dist=0x%02X their_key_dist=0x%02X",
             (int)ble_hs_cfg.sm_bonding, (int)ble_hs_cfg.sm_mitm, (int)ble_hs_cfg.sm_sc, (int)ble_hs_cfg.sm_io_cap,
             (unsigned)ble_hs_cfg.sm_our_key_dist, (unsigned)ble_hs_cfg.sm_their_key_dist);
}

static bool get_own_addr(uint8_t out_mac[6], uint8_t *out_type = nullptr) {
    uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) return false;

    ble_addr_t addr{};
    rc = ble_hs_id_copy_addr(own_addr_type, addr.val, nullptr);
    if (rc != 0) return false;

    for (int i = 0; i < 6; ++i) out_mac[i] = addr.val[i];
    if (out_type) *out_type = own_addr_type;
    return true;
}

static void log_mac(const char *prefix) {
    uint8_t mac[6];
    uint8_t type;
    if (get_own_addr(mac, &type)) {
        ble_addr_t addr{};
        for (int i = 0; i < 6; ++i) addr.val[i] = mac[i];
        ESP_LOGI(TAG, "%s MAC: %s (type=%u)", prefix, addr_to_str(addr).c_str(), type);
    } else {
        ESP_LOGW(TAG, "%s MAC: unknown (could not read)", prefix);
    }
}

// Helper to safely append const string or default value
static int append_const_string_or_default(struct os_mbuf *om, const char *str, const char *default_str) {
    const char *val = str ? str : default_str;
    return os_mbuf_append(om, val, strlen(val));
}

static void log_spacer() { ESP_LOGI(TAG, " "); }
static void log_banner(const char *context_tag) {
    ESP_LOGI(TAG, "*** IRK CAPTURED *** (%s)", (context_tag ? context_tag : "unknown"));
}

//======================== Output helpers (centralized) ========================

static void publish_and_log_irk(IRKCaptureComponent *self,
                                const ble_addr_t &peer_id_addr,
                                const std::string &irk_hex,
                                const char *context_tag) {
    const std::string addr_str = addr_to_str(peer_id_addr);
    log_spacer();
    log_banner(context_tag);
    ESP_LOGI(TAG, "Identity Address: %s", addr_str.c_str());
    ESP_LOGI(TAG, "IRK: %s", irk_hex.c_str());
    log_spacer();
    if (self) self->publish_irk_to_sensors(irk_hex, addr_str.c_str());
}

//======================== GATT DB ========================

static uint16_t g_hr_handle;
static uint16_t g_prot_handle;

static struct ble_gatt_chr_def hr_chrs[] = {
    {
        .uuid = &UUID_CHR_HR_MEAS.u,
        .access_cb = chr_read_hr,
        .arg = nullptr,
        // 1.0 behavior: read requires ENC, notify also present
        .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ_ENC,
        .val_handle = &g_hr_handle,
    },
    {0}
};

static struct ble_gatt_chr_def devinfo_chrs[] = {
    {
        .uuid = &UUID_CHR_MANUF.u,
        .access_cb = chr_read_devinfo,
        .arg = (void *)"ESPresense",
        .flags = BLE_GATT_CHR_F_READ_ENC,
    },
    {
        .uuid = &UUID_CHR_MODEL.u,
        .access_cb = chr_read_devinfo,
        .arg = (void *)"IRK Capture",
        .flags = BLE_GATT_CHR_F_READ_ENC,
    },
    {0}
};

static struct ble_gatt_chr_def batt_chrs[] = {
    {
        .uuid = &UUID_CHR_BATT_LVL.u,
        .access_cb = chr_read_batt,
        .arg = nullptr,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
    },
    {0}
};

static struct ble_gatt_chr_def prot_chrs[] = {
    {
        .uuid = &UUID_CHR_PROT.u,
        .access_cb = chr_read_protected,
        .arg = (void *)"Protected Info",
        .flags = BLE_GATT_CHR_F_READ_ENC,
        .val_handle = &g_prot_handle,
    },
    {0}
};

static struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &UUID_SVC_HR.u,
        .characteristics = hr_chrs,
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &UUID_SVC_DEVINFO.u,
        .characteristics = devinfo_chrs,
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &UUID_SVC_BAS.u,
        .characteristics = batt_chrs,
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &UUID_SVC_PROT.u,
        .characteristics = prot_chrs,
    },
    {0}
};

//======================== Access callbacks ========================

static int chr_read_devinfo(uint16_t conn_handle, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (!is_encrypted(conn_handle)) return BLE_ATT_ERR_INSUFFICIENT_ENC;
    append_const_string_or_default(ctxt->om, (const char *)arg, "IRK Capture");
    return 0;
}
static int chr_read_batt(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    uint8_t lvl = 100;
    os_mbuf_append(ctxt->om, &lvl, 1);
    return 0;
}
static int chr_read_hr(uint16_t conn_handle, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (!is_encrypted(conn_handle)) return BLE_ATT_ERR_INSUFFICIENT_ENC;
    uint8_t buf[2]; size_t len;
    hr_measurement_sample(buf, &len);
    os_mbuf_append(ctxt->om, buf, len);
    return 0;
}
static int chr_read_protected(uint16_t conn_handle, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (!is_encrypted(conn_handle)) return BLE_ATT_ERR_INSUFFICIENT_ENC;
    append_const_string_or_default(ctxt->om, (const char *)arg, "Protected Info");
    return 0;
}

//======================== Store helper ========================

static int read_peer_bond_by_conn(uint16_t conn_handle, struct ble_store_value_sec *out_bond) {
    struct ble_gap_conn_desc desc;
    int rc = ble_gap_conn_find(conn_handle, &desc);
    if (rc != 0) return rc;

    struct ble_store_key_sec key_sec{};
    key_sec.peer_addr = desc.peer_id_addr;

    return ble_store_read_peer_sec(&key_sec, out_bond);
}

//======================== Entity impls ========================

void IRKCaptureText::control(const std::string &value) {
    ESP_LOGI(TAG, "BLE name changed to: %s", value.c_str());
    publish_state(value);
    parent_->update_ble_name(value);
}

void IRKCaptureSwitch::write_state(bool state) {
    if (state) parent_->start_advertising();
    else parent_->stop_advertising();
    publish_state(state);
}

void IRKCaptureButton::press_action() {
    ESP_LOGI(TAG, "Refreshing MAC address...");
    parent_->refresh_mac();
}

//======================== GAP event handlers (extracted) ========================

static int handle_gap_connect(IRKCaptureComponent *self, struct ble_gap_event *ev) {
    if (ev->connect.status == 0) {
        ESP_LOGI(TAG, "Connection established successfully");
        self->on_connect(ev->connect.conn_handle);
    } else {
        ESP_LOGW(TAG, "Connection failed: status=%d (0x%02X)", ev->connect.status, ev->connect.status);
        self->advertising_ = false;
        self->start_advertising();
    }
    return 0;
}

static int handle_gap_disconnect(IRKCaptureComponent *self, struct ble_gap_event *ev) {
    ESP_LOGI(TAG, "Disconnect reason=%d (0x%02x)", ev->disconnect.reason, ev->disconnect.reason);
    self->on_disconnect();

    // Attempt post-disconnect IRK read using the peer identity address
    struct ble_gap_conn_desc d{};
    if (ble_gap_conn_find(ev->disconnect.conn.conn_handle, &d) == 0) {
        // cache for delayed retry
        self->timers_.last_peer_id = d.peer_id_addr;

        struct ble_store_value_sec bond{};
        struct ble_store_key_sec key{};
        key.peer_addr = d.peer_id_addr;
        int rc = ble_store_read_peer_sec(&key, &bond);
        if (rc == BLE_HS_ENOENT) {
            ESP_LOGD(TAG, "No bond for peer (ENOENT)");
        } else if (rc != 0) {
            ESP_LOGW(TAG, "ble_store_read_peer_sec rc=%d", rc);
        } else if (bond.irk_present) {
            std::string irk_hex = bytes_to_hex_rev(bond.irk, sizeof(bond.irk));
            publish_and_log_irk(self, d.peer_id_addr, irk_hex, "DISC_IMMEDIATE");
        } else {
            ESP_LOGD(TAG, "Bond present but no IRK in store post-disconnect");
        }
    } else {
        memset(&self->timers_.last_peer_id, 0, sizeof(self->timers_.last_peer_id));
    }

    // Schedule an extra delayed post-disconnect check (800 ms)
    self->schedule_post_disconnect_check(self->timers_.last_peer_id);

    // Only restart advertising if not suppressed (IRK re-publish case)
    self->advertising_ = false;
    if (!self->suppress_next_adv_) {
        self->start_advertising();
    } else {
        ESP_LOGI(TAG, "Advertising suppressed to break reconnect loop; will auto-restart in 5s");
        self->suppress_next_adv_ = false;  // Reset for next time
        // Update switch UI to show advertising is off during suppression
        if (self->advertising_switch_) self->advertising_switch_->publish_state(false);
        // Schedule auto-restart after 5 seconds to allow fresh connections later
        self->adv_restart_time_ = now_ms() + 5000;
    }
    return 0;
}

static int handle_gap_enc_change(IRKCaptureComponent *self, struct ble_gap_event *ev) {
    ESP_LOGI(TAG, "ENC_CHANGE status=%d (0x%02X)", ev->enc_change.status, ev->enc_change.status);

    // Log common encryption failure reasons for Android troubleshooting
    if (ev->enc_change.status != 0) {
        const char *status_desc = "Unknown";
        switch (ev->enc_change.status) {
            case 1: status_desc = "Passkey Entry Failed"; break;
            case 2: status_desc = "OOB Not Available"; break;
            case 3: status_desc = "Authentication Requirements"; break;
            case 4: status_desc = "Confirm Value Failed"; break;
            case 5: status_desc = "Pairing Not Supported"; break;
            case 6: status_desc = "Encryption Key Size"; break;
            case 7: status_desc = "Command Not Supported"; break;
            case 8: status_desc = "Unspecified Reason"; break;
            case 9: status_desc = "Repeated Attempts"; break;
            case 10: status_desc = "Invalid Parameters"; break;
            case 11: status_desc = "DHKey Check Failed"; break;
            case 12: status_desc = "Numeric Comparison Failed"; break;
            case 13: status_desc = "BR/EDR Pairing in Progress"; break;
            case 14: status_desc = "Cross-transport Key Derivation"; break;
            case 1288: status_desc = "DHKey Check Failed (NimBLE)"; break;
        }
        ESP_LOGW(TAG, "ENC_CHANGE failed: %s (status=%d)", status_desc, ev->enc_change.status);
    }

    if (ev->enc_change.status == 0) {
        self->enc_ready_ = true;
        self->enc_time_ = now_ms();
        self->on_auth_complete(true);
        log_conn_desc(ev->enc_change.conn_handle);

        // Immediate store read using identity address
        struct ble_gap_conn_desc d{};
        if (ble_gap_conn_find(ev->enc_change.conn_handle, &d) == 0) {
            self->timers_.enc_peer_id = d.peer_id_addr;

            struct ble_store_key_sec key{};
            key.peer_addr = d.peer_id_addr;
            struct ble_store_value_sec bond{};
            int rc = ble_store_read_peer_sec(&key, &bond);
            if (rc == BLE_HS_ENOENT) {
                ESP_LOGD(TAG, "No bond for peer yet (ENOENT); scheduling late check");
                self->schedule_late_enc_check(d.peer_id_addr);
            } else if (rc != 0) {
                ESP_LOGW(TAG, "ble_store_read_peer_sec rc=%d; scheduling late check", rc);
                self->schedule_late_enc_check(d.peer_id_addr);
            } else if (bond.irk_present) {
                std::string irk_hex = bytes_to_hex_rev(bond.irk, sizeof(bond.irk));
                publish_and_log_irk(self, d.peer_id_addr, irk_hex, "ENC_CHANGE");
                // 1.0 behavior: terminate immediately after successful ENC + IRK capture
                ble_gap_terminate(ev->enc_change.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            } else {
                ESP_LOGD(TAG, "Bond present but no IRK yet; scheduling late check");
                self->schedule_late_enc_check(d.peer_id_addr);
            }
        }

        ESP_LOGI(TAG, "Encryption established; will attempt IRK capture shortly");
    } else {
        // Encryption failed - clear all bonds and terminate
        ESP_LOGW(TAG, "ENC_CHANGE failed status=%d; clearing all bonds", ev->enc_change.status);
        ble_store_clear();  // Clear everything to force fresh pairing
        ble_gap_terminate(ev->enc_change.conn_handle, BLE_ERR_REM_USER_CONN_TERM);

        // For authentication/DHKey failures, suppress advertising to force peer reset
        // Status 7 = BLE_SM_ERR_AUTHREQ (authentication requirements)
        // Status 1288 = DHKey check failure
        if (ev->enc_change.status == 7 || ev->enc_change.status == 1288) {
            ESP_LOGW(TAG, "Authentication/DHKey failure (status=%d) - suppressing advertising to force peer reset",
                     ev->enc_change.status);
            self->suppress_next_adv_ = true;
        }
    }
    return 0;
}

static int handle_gap_repeat_pairing(IRKCaptureComponent *self, struct ble_gap_event *ev) {
    // Portable: get the current connection descriptor from the handle
    struct ble_gap_conn_desc d{};
    int rc = ble_gap_conn_find(ev->repeat_pairing.conn_handle, &d);
    if (rc == 0) {
        const ble_addr_t *peer = &d.peer_id_addr;
        ESP_LOGW(TAG, "Repeat pairing from %02X:%02X:%02X:%02X:%02X:%02X (clearing all bond data)",
                 peer->val[5], peer->val[4], peer->val[3], peer->val[2], peer->val[1], peer->val[0]);
        // Delete ALL stored data for this peer
        ble_store_util_delete_peer(peer);
    } else {
        ESP_LOGW(TAG, "Repeat pairing: conn desc not found rc=%d", rc);
    }
    return BLE_GAP_REPEAT_PAIRING_RETRY;
}

//======================== GAP event handler (dispatcher) ========================

int IRKCaptureComponent::gap_event_handler(struct ble_gap_event *ev, void *arg) {
    auto *self = static_cast<IRKCaptureComponent *>(arg);
    switch (ev->type) {
        case BLE_GAP_EVENT_CONNECT:
            return handle_gap_connect(self, ev);

        case BLE_GAP_EVENT_DISCONNECT:
            return handle_gap_disconnect(self, ev);

        case BLE_GAP_EVENT_ENC_CHANGE:
            return handle_gap_enc_change(self, ev);

        case BLE_GAP_EVENT_REPEAT_PAIRING:
            return handle_gap_repeat_pairing(self, ev);

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU updated: %u", ev->mtu.value);
            return 0;

        case BLE_GAP_EVENT_PASSKEY_ACTION: {
            const char *action_desc = "Unknown";
            switch (ev->passkey.params.action) {
                case BLE_SM_IOACT_NONE: action_desc = "None"; break;
                case BLE_SM_IOACT_OOB: action_desc = "OOB"; break;
                case BLE_SM_IOACT_INPUT: action_desc = "Input (peer displays passkey)"; break;
                case BLE_SM_IOACT_DISP: action_desc = "Display (we should show passkey)"; break;
                case BLE_SM_IOACT_NUMCMP: action_desc = "Numeric Comparison"; break;
            }
            ESP_LOGI(TAG, "PASSKEY_ACTION: %s (action=%d)", action_desc, ev->passkey.params.action);

            // Log passkey if we're supposed to display it (shouldn't happen with NO_INPUT_OUTPUT)
            if (ev->passkey.params.action == BLE_SM_IOACT_DISP) {
                ESP_LOGW(TAG, "UNEXPECTED: Peer requested passkey display (passkey=%06lu)",
                         (unsigned long)ev->passkey.params.numcmp);
            }

            // Just log and return - main branch behavior
            return 0;
        }

        case BLE_GAP_EVENT_NOTIFY_RX:
            return 0;

        case BLE_GAP_EVENT_NOTIFY_TX:
            return 0;

        case 14:  // BLE_GAP_EVENT_L2CAP_UPDATE_REQ
            // Accept connection parameter updates
            return 0;

        case 16:  // BLE_GAP_EVENT_IDENTITY_RESOLVED
            // Identity resolved successfully (IRK working!)
            ESP_LOGD(TAG, "Peer identity resolved using IRK");
            return 0;

        case 18:  // BLE_GAP_EVENT_PHY_UPDATE_COMPLETE
            // PHY layer updated (normal)
            return 0;

        case 27:  // BLE_GAP_EVENT_AUTHORIZE
            // Authorization event (allow by returning 0)
            return 0;

        case 34:  // BLE_GAP_EVENT_SUBRATE_CHANGE
            // BLE 5.2+ subrate change (normal)
            return 0;

        case 38:  // BLE_GAP_EVENT_VS_HCI
            // Vendor-specific HCI event (can ignore)
            return 0;

        default:
            // Log truly unknown GAP events
            ESP_LOGD(TAG, "Unhandled GAP event type=%d", ev->type);
            return 0;
    }
}

//======================== Component lifecycle ========================

void IRKCaptureComponent::setup() {
    // Always generate dynamic BLE device name: "IRK XXXXXX" where XXXXXX is a random 6-digit number
    // Why: Makes each boot appear as a fresh device to iOS, bypassing persistent Bluetooth cache
    uint32_t random_num = esp_random() % 1000000;  // 0-999999
    char name_buf[16];
    snprintf(name_buf, sizeof(name_buf), "IRK %06u", random_num);
    ble_name_ = name_buf;

    ESP_LOGI(TAG, "IRK Capture v%s ready (BLE name: %s)", VERSION, ble_name_.c_str());
    this->setup_ble();

    if (start_on_boot_) {
        this->start_advertising();
    }
    if (ble_name_text_) {
        ble_name_text_->publish_state(ble_name_);
    }
}

void IRKCaptureComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "IRK Capture:");
    ESP_LOGCONFIG(TAG, "  BLE Name: %s", ble_name_.c_str());
    ESP_LOGCONFIG(TAG, "  Advertising: %s", advertising_ ? "YES" : "NO");
}

void IRKCaptureComponent::loop() {
    const uint32_t now = now_ms();
    if (now - last_loop_ < TimingConfig::LOOP_MIN_INTERVAL_MS) return;
    last_loop_ = now;

    // Timers for IRK checks
    handle_post_disconnect_timer(now);
    handle_late_enc_timer(now);

    // Auto-restart advertising if suppressed and timer expired
    if (!advertising_ && adv_restart_time_ != 0 && now >= adv_restart_time_) {
        adv_restart_time_ = 0;
        ESP_LOGI(TAG, "Auto-restarting advertising after suppression timeout");
        start_advertising();
    }

    // Pairing robustness (1.0 single retry)
    retry_security_if_needed(now);

    if (!connected_ || conn_handle_ == BLE_HS_CONN_HANDLE_NONE) return;

    // HR notify and IRK polling (post-ENC)
    notify_hr_if_due(now);
    poll_irk_if_due(now);
}

//======================== BLE setup/registry ========================

void IRKCaptureComponent::setup_ble() {
    // NVS for key store
    auto err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // NimBLE host
    nimble_port_init();

    // Security (1.0: set once here; no later re-asserts)
    ble_hs_cfg.reset_cb = [](int reason) { ESP_LOGW(TAG, "NimBLE reset reason=%d", reason); };
    ble_hs_cfg.sync_cb = []() { ESP_LOGI(TAG, "NimBLE host synced"); };

    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;  // Secure Connections enabled
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;  // Just Works pairing (no PIN)
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    log_sm_config();

    // Key-value store for bonding/keys
    ble_store_config_init();

    // GAP/GATT and name
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(ble_name_.c_str());

    // Register services
    this->register_gatt_services();

    // Host task
    nimble_port_freertos_init([](void *) {
        nimble_port_run();
        nimble_port_freertos_deinit();
        vTaskDelete(NULL);
    });

    // Seed initial static random address at boot (matches Arduino BLE library behavior)
    // Why: Provides stable identity before user-triggered refresh_mac rotations
    uint8_t rnd[6];
    esp_fill_random(rnd, sizeof(rnd));
    rnd[0] |= 0xC0;  // static random
    rnd[0] &= 0xFE;
    int rc = ble_hs_id_set_rnd(rnd);
    if (rc == 0) {
        ESP_LOGI(TAG, "Initial MAC (set_rnd): %02X:%02X:%02X:%02X:%02X:%02X",
                 rnd[5], rnd[4], rnd[3], rnd[2], rnd[1], rnd[0]);
        log_mac("Effective");
    }

    host_synced_ = true;
}

void IRKCaptureComponent::register_gatt_services() {
    // Point model string at current name for DevInfo
    devinfo_chrs[1].arg = (void *)ble_name_.c_str();

    int rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc == 0) rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT registration failed rc=%d", rc);
        return;
    }
    hr_char_handle_ = g_hr_handle;
    prot_char_handle_ = g_prot_handle;

    ESP_LOGD(TAG, "HR handle=%u, Protected handle=%u", hr_char_handle_, prot_char_handle_);
}

//======================== Advertising ========================

void IRKCaptureComponent::start_advertising() {
    if (!host_synced_) {
        ESP_LOGW(TAG, "Host not synced; cannot advertise");
        return;
    }

    ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)ble_name_.c_str();
    fields.name_len = (uint8_t)ble_name_.size();
    fields.name_is_complete = 1;

    // Appearance: Heart Rate Sensor
    fields.appearance = APPEARANCE_HEART_RATE_SENSOR;
    fields.appearance_is_present = 1;

    // Include standard services in adv
    uint16_t svc16[] = { UUID_SVC_HEART_RATE, UUID_SVC_BATTERY, UUID_SVC_DEVICE_INFO };
    ble_uuid16_t uu16[3] = { BLE_UUID16_INIT(svc16[0]), BLE_UUID16_INIT(svc16[1]), BLE_UUID16_INIT(svc16[2]) };
    fields.uuids16 = uu16;
    fields.num_uuids16 = 3;
    fields.uuids16_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields rc=%d", rc);
        return;
    }

    ble_gap_adv_params advp{};
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;

    uint8_t own_addr_type;
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto rc=%d", rc);
        return;
    }

    rc = ble_gap_adv_start(own_addr_type, nullptr, BLE_HS_FOREVER, &advp, IRKCaptureComponent::gap_event_handler, this);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start rc=%d", rc);
        advertising_ = false;
    } else {
        advertising_ = true;
        if (advertising_switch_) advertising_switch_->publish_state(true);
        ESP_LOGD(TAG, "Advertising as '%s' (Heart Rate Sensor)", ble_name_.c_str());
    }
}

void IRKCaptureComponent::stop_advertising() {
    ble_gap_adv_stop();
    advertising_ = false;
    if (advertising_switch_) advertising_switch_->publish_state(false);
    ESP_LOGD(TAG, "Advertising stopped");
}

//======================== MAC refresh (blocking, v1.0 behavior) ========================

void IRKCaptureComponent::refresh_mac() {
    // Stop adv and terminate connection if any
    if (advertising_) this->stop_advertising();
    if (conn_handle_ != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
    }

    // Ensure idle before changing address (up to 500 ms)
    // Bounded loop with App.feed_wdt() prevents watchdog timeout during wait
    const uint32_t t0 = now_ms();
    while ((advertising_ || conn_handle_ != BLE_HS_CONN_HANDLE_NONE) &&
           (now_ms() - t0) < 500) {
        delay(10);
        App.feed_wdt();  // Watchdog safety: prevent timeout during blocking wait
    }

    // Clear all bonds since MAC change invalidates them
    ESP_LOGI(TAG, "Clearing all bond data before MAC refresh");
    ble_store_clear();

    // Log previous MAC before change
    log_mac("Previous");

    // Try to set a new static-random address; retries will succeed once host accepts RANDOM identity
    uint8_t mac[6];
    for (int tries = 0; tries < 6; ++tries) {
        esp_fill_random(mac, sizeof(mac));
        // Force static-random: top two bits 11, LSB bit0 = 0 (unicast)
        mac[0] |= 0xC0;
        mac[0] &= 0xFE;

        bool all_zero = true;
        for (int i = 0; i < 6; ++i) if (mac[i] != 0x00) { all_zero = false; break; }
        if (all_zero) continue;

        int rc = ble_hs_id_set_rnd(mac);
        if (rc == 0) {
            ESP_LOGI(TAG, "New MAC (set_rnd): %02X:%02X:%02X:%02X:%02X:%02X",
                     mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

            // Re-infer own address type for next adv start
            uint8_t own_addr_type;
            int rc2 = ble_hs_id_infer_auto(0, &own_addr_type);
            ESP_LOGI(TAG, "Own address type after set_rnd: %u (0=PUBLIC,1=RANDOM)", own_addr_type);
            if (rc2 != 0) {
                ESP_LOGW(TAG, "ble_hs_id_infer_auto rc=%d after set_rnd", rc2);
            }

            log_mac("Effective");

            // Reset pairing state for next session and restart advertising
            enc_ready_ = false;
            enc_time_ = 0;

            delay(100);
            this->start_advertising();
            return;
        } else if (rc == BLE_HS_EINVAL) {
            ESP_LOGW(TAG, "ble_hs_id_set_rnd EINVAL (try %d): %02X:%02X:%02X:%02X:%02X:%02X",
                     tries + 1, mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

            // Nudge: re-infer type, short delay, and retry
            uint8_t own_addr_type;
            (void)ble_hs_id_infer_auto(0, &own_addr_type);
            ESP_LOGD(TAG, "Current own_addr_type=%u; will retry set_rnd", own_addr_type);
            delay(50);
        } else {
            ESP_LOGE(TAG, "ble_hs_id_set_rnd rc=%d (try %d)", rc, tries + 1);
            delay(20);
        }
    }

    ESP_LOGE(TAG, "Failed to set a new static random MAC after retries");
}

//======================== BLE name update ========================

void IRKCaptureComponent::update_ble_name(const std::string &name) {
    // Update name in GAP
    ble_name_ = name;
    ble_svc_gap_device_name_set(ble_name_.c_str());

    // Update DevInfo model read callback source
    devinfo_chrs[1].arg = (void *)ble_name_.c_str();

    // Restart advertising with the new name first (keeps UI consistent)
    if (advertising_) {
        this->stop_advertising();
        delay(100);
        this->start_advertising();
    }

    // Immediately rotate MAC after name change
    this->refresh_mac();
}

//======================== GAP helpers ========================

void IRKCaptureComponent::on_connect(uint16_t conn_handle) {
    conn_handle_ = conn_handle;
    connected_ = true;
    enc_ready_ = false;
    enc_time_ = 0;

    // Reset loop-helper state (1.0 single retry model)
    sec_retry_done_ = false;
    sec_init_time_ms_ = 0;
    irk_gave_up_ = false;
    irk_last_try_ms_ = 0;

    ESP_LOGI(TAG, "Connected; handle=%u, initiating security", conn_handle_);
    log_conn_desc(conn_handle_);
    log_sm_config();

    // Cache peer identity address for delayed post-disconnect checks
    struct ble_gap_conn_desc d;
    if (ble_gap_conn_find(conn_handle_, &d) == 0) {
        timers_.last_peer_id = d.peer_id_addr;

        // Check for bond state mismatch: peer thinks it's unbonded but we have bond data
        if (!d.sec_state.bonded) {
            struct ble_store_key_sec key{};
            key.peer_addr = d.peer_id_addr;
            struct ble_store_value_sec bond{};
            int rc = ble_store_read_peer_sec(&key, &bond);
            if (rc == BLE_HS_ENOENT) {
                ESP_LOGD(TAG, "Peer unbonded and no cached bond (ENOENT) - will pair fresh");
            } else if (rc != 0) {
                ESP_LOGW(TAG, "ble_store_read_peer_sec rc=%d during bond mismatch check", rc);
            } else if (bond.irk_present) {
                // Re-publish the IRK we already have
                std::string irk_hex = bytes_to_hex_rev(bond.irk, sizeof(bond.irk));
                char addr_str[18];
                snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                         d.peer_id_addr.val[5], d.peer_id_addr.val[4], d.peer_id_addr.val[3],
                         d.peer_id_addr.val[2], d.peer_id_addr.val[1], d.peer_id_addr.val[0]);
                publish_irk_to_sensors(irk_hex, addr_str);
                ESP_LOGI(TAG, "Re-published existing IRK for already-paired device");
                // Set flag to prevent immediate re-advertising (break the reconnect loop)
                suppress_next_adv_ = true;
                // Disconnect since we already have what we need
                ble_gap_terminate(conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
                return;  // Don't initiate security, we're done
            } else {
                // No IRK in bond - delete and try fresh pairing
                ESP_LOGW(TAG, "Peer unbonded but we have cached bond without IRK. Clearing to force fresh pairing.");
                ble_store_util_delete_peer(&d.peer_id_addr);
            }
        }
    } else {
        memset(&timers_.last_peer_id, 0, sizeof(timers_.last_peer_id));
    }

    // Proactively initiate pairing; peer should show pairing dialog now
    int rc = ble_gap_security_initiate(conn_handle_);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "ble_gap_security_initiate rc=%d", rc);
    }
}

void IRKCaptureComponent::on_disconnect() {
    connected_ = false;
    conn_handle_ = BLE_HS_CONN_HANDLE_NONE;
    enc_ready_ = false;
    enc_time_ = 0;

    // Reset loop-helper state
    sec_retry_done_ = false;
    sec_init_time_ms_ = 0;
    irk_gave_up_ = false;
    irk_last_try_ms_ = 0;

    ESP_LOGI(TAG, "Disconnected");
}

void IRKCaptureComponent::on_auth_complete(bool) {
    // IRK retrieved in loop via try_get_irk
}

//======================== IRK extraction ========================

bool IRKCaptureComponent::try_get_irk(uint16_t conn_handle, uint8_t irk_out[16], ble_addr_t &peer_id_out) {
    struct ble_store_value_sec bond{};
    struct ble_gap_conn_desc desc{};

    int rc = ble_gap_conn_find(conn_handle, &desc);
    if (rc != 0) {
        ESP_LOGD(TAG, "ble_gap_conn_find rc=%d", rc);
        return false;
    }

    // Build store key from the peer identity address (stable)
    struct ble_store_key_sec key_sec{};
    key_sec.peer_addr = desc.peer_id_addr;

    ESP_LOGD(TAG, "Reading bond for peer: %s", addr_to_str(desc.peer_id_addr).c_str());

    rc = ble_store_read_peer_sec(&key_sec, &bond);
    if (rc == BLE_HS_ENOENT) {
        ESP_LOGD(TAG, "No bond for peer (ENOENT)");
        return false;
    }
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_store_read_peer_sec rc=%d", rc);
        return false;
    }

    // Detailed bond information for Android troubleshooting
    ESP_LOGD(TAG, "Bond found: ediv=%u rand=%llu irk_present=%d ltk_present=%d csrk_present=%d",
             (unsigned)bond.ediv, (unsigned long long)bond.rand_num,
             (int)bond.irk_present, (int)bond.ltk_present, (int)bond.csrk_present);

    ESP_LOGD(TAG, "Bond security: authenticated=%d sc=%d",
             (int)bond.authenticated, (int)bond.sc);

    if (!bond.irk_present) {
        ESP_LOGW(TAG, "Bond present but no IRK (peer did not distribute ID key)");
        ESP_LOGW(TAG, "Peer key distribution may be incomplete or unsupported by device");
        return false;
    }

    // Return raw IRK bytes and peer identity address
    memcpy(irk_out, bond.irk, 16);
    peer_id_out = desc.peer_id_addr;
    return true;
}

//======================== Timer helpers ========================

void IRKCaptureComponent::schedule_post_disconnect_check(const ble_addr_t &peer_id) {
    timers_.last_peer_id = peer_id;
    timers_.post_disc_due_ms = now_ms() + TimingConfig::POST_DISC_DELAY_MS;
}

// Schedule delayed IRK check to allow NVS flush after encryption (some platforms delay write)
void IRKCaptureComponent::schedule_late_enc_check(const ble_addr_t &peer_id) {
    timers_.enc_peer_id = peer_id;
    timers_.late_enc_due_ms = now_ms() + TimingConfig::ENC_LATE_READ_DELAY_MS;
}

void IRKCaptureComponent::handle_post_disconnect_timer(uint32_t now) {
    if (!timers_.post_disc_due_ms || now < timers_.post_disc_due_ms) return;
    timers_.post_disc_due_ms = 0;  // consume

    if (!addr_is_zero(timers_.last_peer_id)) {
        struct ble_store_value_sec bond{};
        struct ble_store_key_sec key{};
        key.peer_addr = timers_.last_peer_id;
        int rc = ble_store_read_peer_sec(&key, &bond);
        if (rc == BLE_HS_ENOENT) {
            ESP_LOGD(TAG, "No bond for peer (ENOENT) - post-disc delayed check");
        } else if (rc != 0) {
            ESP_LOGW(TAG, "ble_store_read_peer_sec rc=%d - post-disc delayed check", rc);
        } else if (bond.irk_present) {
            std::string irk_hex = bytes_to_hex_rev(bond.irk, sizeof(bond.irk));
            publish_and_log_irk(this, timers_.last_peer_id, irk_hex, "DISC_DELAYED");
        } else {
            ESP_LOGD(TAG, "Bond present but no IRK - post-disc delayed check");
        }
    }
}

void IRKCaptureComponent::handle_late_enc_timer(uint32_t now) {
    if (!timers_.late_enc_due_ms || now < timers_.late_enc_due_ms) return;
    timers_.late_enc_due_ms = 0;

    if (!addr_is_zero(timers_.enc_peer_id)) {
        struct ble_store_key_sec key{};
        key.peer_addr = timers_.enc_peer_id;
        struct ble_store_value_sec bond{};
        int rc = ble_store_read_peer_sec(&key, &bond);
        if (rc == BLE_HS_ENOENT) {
            ESP_LOGD(TAG, "No bond for peer (ENOENT) - late ENC check");
        } else if (rc != 0) {
            ESP_LOGW(TAG, "ble_store_read_peer_sec rc=%d - late ENC check", rc);
        } else if (bond.irk_present) {
            std::string irk_hex = bytes_to_hex_rev(bond.irk, sizeof(bond.irk));
            publish_and_log_irk(this, timers_.enc_peer_id, irk_hex, "ENC_LATE");

            // 1.0 compatible: terminate after late capture if still connected
            if (connected_ && conn_handle_ != BLE_HS_CONN_HANDLE_NONE) {
                ble_gap_terminate(conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
            }
        } else {
            ESP_LOGD(TAG, "Bond present but no IRK - late ENC check");
        }
    }
}

//======================== Loop helpers ========================

void IRKCaptureComponent::retry_security_if_needed(uint32_t now) {
    // 1.0 behavior: single retry ~2s after initial initiate
    if (connected_ && !enc_ready_) {
        if (sec_init_time_ms_ == 0) sec_init_time_ms_ = now;

        // Retry security at 2s
        if (!sec_retry_done_ && (now - sec_init_time_ms_) > 2000) {
            int rc = ble_gap_security_initiate(conn_handle_);
            ESP_LOGW(TAG, "Retry security initiate rc=%d", rc);
            sec_retry_done_ = true;
        }

        // If encryption still hasn't completed after 20s, assume peer forgot pairing
        // Delete our bond and disconnect to force fresh pairing on reconnect
        if ((now - sec_init_time_ms_) > 20000) {
            struct ble_gap_conn_desc d{};
            if (ble_gap_conn_find(conn_handle_, &d) == 0) {
                ESP_LOGW(TAG, "Encryption timeout after 20s; peer may have forgotten pairing. Clearing bond data.");
                ble_store_util_delete_peer(&d.peer_id_addr);
            }
            // Disconnect to trigger fresh pairing on next connection
            ble_gap_terminate(conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
        }
    } else if (!connected_) {
        sec_retry_done_ = false;
        sec_init_time_ms_ = 0;
    }
}

void IRKCaptureComponent::notify_hr_if_due(uint32_t now) {
    if (hr_char_handle_ == 0) return;
    if (now - last_notify_ <= TimingConfig::HR_NOTIFY_INTERVAL_MS) return;

    last_notify_ = now;
    uint8_t buf[2]; size_t len;
    hr_measurement_sample(buf, &len);
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
    int rc = ble_gatts_notify_custom(conn_handle_, hr_char_handle_, om);
    if (rc != 0) {
        ESP_LOGD(TAG, "notify rc=%d", rc);
    }
}

void IRKCaptureComponent::poll_irk_if_due(uint32_t now) {
    // Attempt IRK retrieval after encryption + delay (allow store write)
    if (!enc_ready_ || irk_gave_up_) return;
    if ((now - enc_time_) < TimingConfig::ENC_TO_FIRST_TRY_DELAY_MS) return;
    if ((now - irk_last_try_ms_) < TimingConfig::ENC_TRY_INTERVAL_MS) return;

    irk_last_try_ms_ = now;

    uint8_t irk_bytes[16];
    ble_addr_t peer_id;
    if (try_get_irk(conn_handle_, irk_bytes, peer_id)) {
        std::string irk_hex = bytes_to_hex_rev(irk_bytes, sizeof(irk_bytes));
        publish_and_log_irk(this, peer_id, irk_hex, "POLL_CONNECTED");
        ble_gap_terminate(conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
        irk_gave_up_ = true;
    } else {
        if ((now - enc_time_) > TimingConfig::ENC_GIVE_UP_AFTER_MS) {
            ESP_LOGW(TAG, "IRK not found after %u ms post-encryption", TimingConfig::ENC_GIVE_UP_AFTER_MS);
            irk_gave_up_ = true;
        }
    }
}

//======================== Public publish utility ========================

void IRKCaptureComponent::publish_irk_to_sensors(const std::string &irk_hex, const char *addr_str) {
    if (irk_sensor_)     irk_sensor_->publish_state(irk_hex);
    if (address_sensor_) address_sensor_->publish_state(addr_str);
}

}  // namespace irk_capture
}  // namespace esphome
