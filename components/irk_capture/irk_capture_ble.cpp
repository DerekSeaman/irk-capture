// NimBLE host setup and sync, the GATT tables and their read callbacks,
// heart-rate notifications, and reporting of a host that never synchronises.

#include "irk_capture_internal.h"
#include "irk_capture_gatt.h"

// ESP32-only implementation - requires Bluetooth hardware
#ifdef USE_ESP32

// Some ESP-IDF 5.x package variants omit this prototype from headers. NimBLE's
// implementation returns void; declaring it as int reads an undefined return
// register (observed as nonzero on ESP32-C6).
extern "C" void ble_store_config_init(void);

namespace esphome {
namespace irk_capture {

// Security Manager IO capability names (Core spec Vol 3 Part H 3.5.1).
static inline const char* io_cap_str(uint8_t io_cap) {
  switch (io_cap) {
    case BLE_HS_IO_DISPLAY_ONLY:
      return "DisplayOnly";
    case BLE_HS_IO_DISPLAY_YESNO:
      return "DisplayYesNo";
    case BLE_HS_IO_KEYBOARD_ONLY:
      return "KeyboardOnly";
    case BLE_HS_IO_NO_INPUT_OUTPUT:
      return "NoInputNoOutput";
    case BLE_HS_IO_KEYBOARD_DISPLAY:
      return "KeyboardDisplay";
    default:
      return "unknown";
  }
}

// Key distribution bits as "ENC|ID|SIGN|LINK"; "none" when no bit is set.
// Writes into `buf`, which must hold at least 20 bytes, and returns it.
static inline const char* key_dist_str(uint8_t dist, char* buf, size_t len) {
  static const struct {
    uint8_t bit;
    const char* name;
  } kBits[] = {
    { BLE_SM_PAIR_KEY_DIST_ENC, "ENC" },
    { BLE_SM_PAIR_KEY_DIST_ID, "ID" },
    { BLE_SM_PAIR_KEY_DIST_SIGN, "SIGN" },
    { BLE_SM_PAIR_KEY_DIST_LINK, "LINK" },
  };
  size_t pos = 0;
  buf[0] = '\0';
  for (const auto& b : kBits) {
    if ((dist & b.bit) == 0) continue;
    int n = snprintf(buf + pos, len - pos, "%s%s", pos ? "|" : "", b.name);
    if (n < 0 || (size_t) n >= len - pos) break;
    pos += (size_t) n;
  }
  if (pos == 0) snprintf(buf, len, "none");
  return buf;
}

void log_sm_config(const char* label) {
  char ours[20];
  char theirs[20];
  ESP_LOGI(TAG,
           "%s: bonding=%d mitm=%d sc=%d io_cap=%d (%s) our_key_dist=0x%02X (%s) "
           "their_key_dist=0x%02X (%s)",
           label, (int) ble_hs_cfg.sm_bonding, (int) ble_hs_cfg.sm_mitm, (int) ble_hs_cfg.sm_sc,
           (int) ble_hs_cfg.sm_io_cap, io_cap_str(ble_hs_cfg.sm_io_cap),
           (unsigned) ble_hs_cfg.sm_our_key_dist,
           key_dist_str(ble_hs_cfg.sm_our_key_dist, ours, sizeof(ours)),
           (unsigned) ble_hs_cfg.sm_their_key_dist,
           key_dist_str(ble_hs_cfg.sm_their_key_dist, theirs, sizeof(theirs)));
}

// Global instance pointer for NimBLE callbacks that don't accept user args
static IRKCaptureComponent* g_irk_instance = nullptr;

// GATT access callbacks (forward decls)
int chr_read_devinfo(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt,
                     void* arg);
int chr_read_batt(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt,
                  void* arg);
int chr_read_hr(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt,
                void* arg);
int chr_read_hid_protocol(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt* ctxt, void* arg);
int chr_read_protected(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt* ctxt, void* arg);

static bool is_encrypted(uint16_t conn_handle) {
  struct ble_gap_conn_desc d;
  if (ble_gap_conn_find(conn_handle, &d) == 0) {
    return d.sec_state.encrypted;
  }
  return false;
}

static void hr_measurement_sample(uint8_t* buf, size_t* len) {
  buf[0] = 0x00;  // Flags byte: bit0=0 (UINT8 format), bit1-2=0 (sensor contact
                  // not supported),
                  //             bit3=0 (no energy expended), bit4=0 (RR
                  //             interval not present)
  buf[1] = (uint8_t) (60 + (esp_random() % 40));  // 60-99 bpm
  *len = 2;
}

// A read callback that cannot fill its response must say so. Returning 0 would
// send the peer a truncated or empty value as if it were complete.
static int append_att_value(struct os_mbuf* om, const void* data, size_t len) {
  return os_mbuf_append(om, data, len) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// Helper to safely append const string or default value
static int append_const_string_or_default(struct os_mbuf* om, const char* str,
                                          const char* default_str) {
  // Safely append string to mbuf, using default if str is nullptr or empty
  const char* val = (str && str[0] != '\0') ? str : default_str;
  return append_att_value(om, val, strlen(val));
}

//======================== GATT DB ========================
// WARNING: These static GATT arrays and g_irk_instance assume a single
// IRKCaptureComponent instance per process. Multiple instances would share
// these statics and overwrite each other's .arg pointers, causing UB.

static uint16_t g_hr_handle;
static uint16_t g_prot_handle;

static struct ble_gatt_chr_def hr_chrs[] = {
  {
      .uuid = &UUID_CHR_HR_MEAS.u,
      .access_cb = chr_read_hr,
      .arg = nullptr,
      // READ grants read permission; READ_ENC adds
      // the encryption requirement. READ_ENC alone
      // maps to no ATT read permission at all, so
      // reads would return READ_NOT_PERMITTED.
      .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
      .val_handle = &g_hr_handle,
  },
  { 0 }
};

static struct ble_gatt_chr_def devinfo_chrs[] = {
  {
      .uuid = &UUID_CHR_MANUF.u,
      .access_cb = chr_read_devinfo,
      .arg = nullptr,  // Will be set to 'this' in register_gatt_services()
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
  },
  {
      .uuid = &UUID_CHR_MODEL.u,
      .access_cb = chr_read_devinfo,
      .arg = nullptr,  // Will be set to 'this' in register_gatt_services()
      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
  },
  { 0 }
};

static struct ble_gatt_chr_def batt_chrs[] = { {
                                                   .uuid = &UUID_CHR_BATT_LVL.u,
                                                   .access_cb = chr_read_batt,
                                                   .arg = nullptr,
                                                   .flags =
                                                       BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                                               },
                                               { 0 } };

// Minimal HID characteristic so HID service is valid on NimBLE builds that
// reject empty-characteristic services.
static struct ble_gatt_chr_def hid_chrs[] = { {
                                                  .uuid = &UUID_CHR_HID_PROTO.u,
                                                  .access_cb = chr_read_hid_protocol,
                                                  .arg = nullptr,
                                                  .flags = BLE_GATT_CHR_F_READ,
                                              },
                                              { 0 } };

static struct ble_gatt_chr_def prot_chrs[] = { {
                                                   .uuid = &UUID_CHR_PROT.u,
                                                   .access_cb = chr_read_protected,
                                                   .arg = (void*) "Protected Info",
                                                   .flags = BLE_GATT_CHR_F_READ |
                                                            BLE_GATT_CHR_F_READ_ENC,
                                                   .val_handle = &g_prot_handle,
                                               },
                                               { 0 } };

// Heart Sensor profile GATT services
static struct ble_gatt_svc_def gatt_svcs_heart_sensor[] = {
  {
      // Heart Rate service
      .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &UUID_SVC_HR.u,
      .characteristics = hr_chrs,
  },
  {
      // Device Information service
      .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &UUID_SVC_DEVINFO.u,
      .characteristics = devinfo_chrs,
  },
  {
      // Battery service
      .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &UUID_SVC_BAS.u,
      .characteristics = batt_chrs,
  },
  {
      // Encrypted-read test service; pairing itself is initiated on connect.
      .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &UUID_SVC_PROT.u,
      .characteristics = prot_chrs,
  },
  { 0 }
};

// Keyboard profile GATT services
static struct ble_gatt_svc_def gatt_svcs_keyboard[] = {
  {
      // HID service
      .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &UUID_SVC_HID_BLE.u,
      .characteristics = hid_chrs,
  },
  {
      // Device Information service
      .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &UUID_SVC_DEVINFO.u,
      .characteristics = devinfo_chrs,
  },
  {
      // Battery service
      .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &UUID_SVC_BAS.u,
      .characteristics = batt_chrs,
  },
  {
      // Encrypted-read test service; pairing itself is initiated on connect.
      .type = BLE_GATT_SVC_TYPE_PRIMARY,
      .uuid = &UUID_SVC_PROT.u,
      .characteristics = prot_chrs,
  },
  { 0 }
};

//======================== Access callbacks ========================

int chr_read_devinfo(uint16_t conn_handle, uint16_t, struct ble_gatt_access_ctxt* ctxt, void* arg) {
  if (!is_encrypted(conn_handle)) return BLE_ATT_ERR_INSUFFICIENT_ENC;

  // THREAD-SAFE: arg points to IRKCaptureComponent instance for both
  // characteristics We determine which characteristic by UUID comparison
  auto* self = static_cast<IRKCaptureComponent*>(arg);
  if (self == nullptr) return BLE_ATT_ERR_UNLIKELY;  // Registered without its component.

  // Determine which characteristic is being read
  const ble_uuid_t* char_uuid = ctxt->chr->uuid;
  bool is_manufacturer = ble_uuid_cmp(char_uuid, &UUID_CHR_MANUF.u) == 0;

  std::string value_copy;
  {
    MutexGuard lock(self->state_mutex_);
    if (is_manufacturer) {
      value_copy = self->manufacturer_name_;
    } else {
      // Model Number mirrors the advertised name so GATT stays consistent with
      // it (in Keyboard, ble_name_ would otherwise leak the Heart Sensor name).
      value_copy = self->advertised_name_();
    }
  }

  ESP_LOGD(TAG, "DevInfo read (%s): value='%s'", is_manufacturer ? "Manufacturer" : "Model",
           value_copy.c_str());
  return append_const_string_or_default(ctxt->om, value_copy.c_str(),
                                        is_manufacturer ? "ESP32" : DEFAULT_BLE_NAME);
}
int chr_read_batt(uint16_t, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*) {
  uint8_t lvl = 100;  // Placeholder static battery level
  return append_att_value(ctxt->om, &lvl, 1);
}
int chr_read_hr(uint16_t conn_handle, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*) {
  if (!is_encrypted(conn_handle)) return BLE_ATT_ERR_INSUFFICIENT_ENC;
  uint8_t buf[2];
  size_t len;
  hr_measurement_sample(buf, &len);
  return append_att_value(ctxt->om, buf, len);
}
int chr_read_hid_protocol(uint16_t, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*) {
  uint8_t protocol_mode = 0x01;  // Report protocol mode
  return append_att_value(ctxt->om, &protocol_mode, 1);
}
int chr_read_protected(uint16_t conn_handle, uint16_t, struct ble_gatt_access_ctxt* ctxt,
                       void* arg) {
  if (!is_encrypted(conn_handle)) return BLE_ATT_ERR_INSUFFICIENT_ENC;
  return append_const_string_or_default(ctxt->om, (const char*) arg, "Protected Info");
}

//======================== BLE setup/registry ========================

// Store status callback: logs when the bond table is full, then applies
// NimBLE's round-robin eviction. Eviction is otherwise silent, and a peer whose
// bond vanished re-pairs in ways that are hard to explain from the log alone.
// The bonded peers are listed before and after so the log names the evicted
// peer whichever rule NimBLE used to pick it.
static int log_and_evict_bond(struct ble_store_status_event* event, void* arg) {
  if (event->event_code != BLE_STORE_EVENT_OVERFLOW) {
    if (event->event_code == BLE_STORE_EVENT_FULL) {
      ESP_LOGD(TAG, "Bond store full (obj_type=%d); oldest bond is evicted on the next write",
               event->full.obj_type);
    }
    return ble_store_util_status_rr(event, arg);
  }

  constexpr int kMaxPeers = MYNEWT_VAL(BLE_STORE_MAX_BONDS);
  ble_addr_t before[kMaxPeers];
  ble_addr_t after[kMaxPeers];
  int num_before = 0;
  int num_after = 0;
  const bool listed = ble_store_util_bonded_peers(before, &num_before, kMaxPeers) == 0;

  const int rc = ble_store_util_status_rr(event, arg);

  const ble_addr_t* evicted = nullptr;
  if (listed && ble_store_util_bonded_peers(after, &num_after, kMaxPeers) == 0) {
    for (int i = 0; i < num_before && evicted == nullptr; ++i) {
      bool still_bonded = false;
      for (int j = 0; j < num_after; ++j) {
        if (addr_equal(before[i], after[j])) {
          still_bonded = true;
          break;
        }
      }
      if (!still_bonded) evicted = &before[i];
    }
  }

  if (rc != 0) {
    ESP_LOGW(TAG, "Bond store overflow (obj_type=%d, %d/%d bonds); eviction failed rc=%d",
             event->overflow.obj_type, num_before, kMaxPeers, rc);
  } else if (evicted != nullptr) {
    ESP_LOGI(TAG,
             "Bond store overflow (obj_type=%d, %d/%d bonds); evicted oldest bond %s (type=%u)",
             event->overflow.obj_type, num_before, kMaxPeers, addr_to_str(*evicted).c_str(),
             (unsigned) evicted->type);
  } else {
    ESP_LOGI(TAG, "Bond store overflow (obj_type=%d, %d/%d bonds); evicted a bond",
             event->overflow.obj_type, num_before, kMaxPeers);
  }
  return rc;
}

void IRKCaptureComponent::setup_ble_worker_() {
  ble_npl_event_init(
      &bond_clear_event_,
      [](struct ble_npl_event* event) {
        auto* self = static_cast<IRKCaptureComponent*>(ble_npl_event_get_arg(event));
        self->handle_pending_termination_();
        self->handle_forget_bonds_();
        self->handle_rotation_cleanup_();
      },
      this);

  // Older NPL ports wait indefinitely for space in the host event queue. Keep
  // that wait off both the ESPHome main task and the shared timer task. A
  // notification bit coalesces retries while this single producer is blocked.
  if (xTaskCreate(
          [](void* arg) {
            auto* self = static_cast<IRKCaptureComponent*>(arg);
            for (;;) {
              uint32_t notification;
              xTaskNotifyWait(0, UINT32_MAX, &notification, portMAX_DELAY);
              self->queue_bond_clear_();
            }
          },
          "irk_bond_clear", 2048, this, tskIDLE_PRIORITY + 1, &bond_clear_task_) != pdPASS) {
    bond_clear_task_ = nullptr;
    ESP_LOGE(TAG,
             "BLE worker allocation failed; bond clearing unavailable, disconnects use timeout "
             "recovery");
  }
}

void IRKCaptureComponent::setup_ble() {
  // NVS holds the persisted BLE profile. Bonds are kept in RAM (see below).
  // ESPHome has already initialised NVS, so this normally just confirms it. If
  // it fails, stop: erasing the partition to recover would also destroy the
  // Wi-Fi credentials and every other component's saved settings.
  const esp_err_t err = nvs_flash_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_flash_init failed (%s); not erasing shared storage", esp_err_to_name(err));
    this->mark_failed(LOG_STR("NVS initialization failed"));
    return;
  }

  // NimBLE host. Without a working host and bond store the component cannot
  // perform its only job, so initialization failures are fatal rather than a
  // degraded mode that still advertises.
  int rc = nimble_port_init();
  if (rc != 0) {
    ESP_LOGE(TAG, "nimble_port_init failed rc=%d", rc);
    this->mark_failed(LOG_STR("NimBLE host initialization failed"));
    return;
  }

  // Set global instance for callbacks that don't accept user args
  g_irk_instance = this;

  // From here on a failure must undo nimble_port_init(). Otherwise the
  // controller stays initialised, holding its memory, behind a component that
  // has been marked failed and will never use it.
  const auto abort_ble_setup = [this](const LogString* reason) {
    g_irk_instance = nullptr;
    const esp_err_t deinit_err = nimble_port_deinit();
    if (deinit_err != ESP_OK) {
      ESP_LOGW(TAG, "nimble_port_deinit failed (%s)", esp_err_to_name(deinit_err));
    }
    this->mark_failed(reason);
  };

  // Security (set once here; no later re-asserts)
  ble_hs_cfg.reset_cb = [](int reason) {
    ESP_LOGW(TAG, "NimBLE reset reason=%d", reason);
    if (!g_irk_instance) return;

    g_irk_instance->host_synced_ = false;
    MutexGuard lock(g_irk_instance->state_mutex_);
    g_irk_instance->host_generation_++;
    g_irk_instance->host_wait_since_ = deadline_after(now_ms(), 0);
    g_irk_instance->host_wait_log_time_ = 0;
    if (g_irk_instance->identity_refresh_result_.state == IdentityRefreshState::PENDING) {
      g_irk_instance->identity_refresh_result_.state = IdentityRefreshState::FAILED;
    }
    g_irk_instance->random_address_ready_ = false;
    g_irk_instance->advertising_start_attempts_ = 0;
    g_irk_instance->advertising_failure_log_time_ = 0;
    g_irk_instance->advertising_ = false;
    g_irk_instance->reset_connection_state_();
    g_irk_instance->adv_restart_time_ = 0;
    // Cancel the request without relying on the old host event being serviced.
    // Leave its event object intact: a delayed wake-up checks current state.
    if (g_irk_instance->bond_clear_pending_) {
      g_irk_instance->bond_clear_result_.state = BondClearState::CANCELLED;
      g_irk_instance->bond_clear_result_.reason = "Bluetooth restarted; retry once ready.";
    }
    g_irk_instance->bond_clear_pending_ = false;
    g_irk_instance->bond_clear_host_generation_ = 0;
    // A host reset proves the old controller state is gone. Abort any
    // in-flight MAC rotation instead of leaving REQUESTED waiting for a
    // disconnect callback that can no longer arrive.
    g_irk_instance->mac_rotation_generation_++;
    g_irk_instance->mac_rotation_state_ = MacRotationState::IDLE;
    g_irk_instance->mac_rotation_cleanup_pending_ = false;
    g_irk_instance->mac_rotation_retries_ = 0;
    g_irk_instance->mac_rotation_ready_time_ = 0;
    g_irk_instance->identity_refresh_pending_ = false;
  };
  ble_hs_cfg.sync_cb = []() {
    ESP_LOGI(TAG, "NimBLE host synced");
    // Print SM config once early to reduce chances of log drops later
    log_sm_config("SM config (on sync)");

    // Now that host is synced, set flag and start advertising if configured
    if (g_irk_instance) {
      g_irk_instance->on_ble_host_synced();
    }
  };

  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_mitm = 0;
  ble_hs_cfg.sm_sc = 1;                              // Secure Connections enabled
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;  // Just Works pairing (no PIN)
  ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  log_sm_config();

  // Key-value store for bonding/keys
  // This API returns void. The store callbacks it installs report later
  // operation failures through their own return values.
  ble_store_config_init();

  // ble_store_config_init() installs only read/write/delete callbacks, leaving
  // store_status_cb null. NimBLE caps bonds at CONFIG_BT_NIMBLE_MAX_BONDS (3 by
  // default) — far below max_captures — and when that is reached it asks the
  // application to make room. With no callback the request returns ENOTSUP and
  // the Security Manager rejects the next fresh pairing outright, so a fourth
  // device in one session could never pair. Round-robin eviction is the right
  // policy here: a captured IRK is already published and held in irk_cache_,
  // so the bond itself is disposable once the capture completes.
  ble_hs_cfg.store_status_cb = log_and_evict_bond;

  // Bonds are kept in RAM only: __init__.py disables CONFIG_BT_NIMBLE_NVS_PERSIST,
  // so no peer key outlives a reboot or a change of device ownership. Clearing
  // here as well keeps that true if a build ever enables persistence.
  rc = ble_store_clear();
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_store_clear on boot failed rc=%d", rc);
    abort_ble_setup(LOG_STR("BLE bond store clear failed"));
    return;
  }
  ESP_LOGI(TAG, "Bond table cleared on boot - fresh pairing session ready");

  // GAP/GATT and name
  ble_svc_gap_init();
  ble_svc_gatt_init();
  rc = ble_svc_gap_device_name_set(ble_name_.c_str());
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_svc_gap_device_name_set failed rc=%d", rc);
    abort_ble_setup(LOG_STR("BLE GAP name initialization failed"));
    return;
  }

  // Register services
  if (!this->register_gatt_services()) {
    abort_ble_setup(LOG_STR("BLE GATT service registration failed"));
    return;
  }

  // Initialize the event after NimBLE's NPL port is ready, and publish the
  // worker handle before the host can sync and dispatch connection callbacks.
  setup_ble_worker_();
  nimble_port_freertos_init([](void*) {
    nimble_port_run();
    nimble_port_freertos_deinit();
    vTaskDelete(NULL);
  });
}

void IRKCaptureComponent::on_ble_host_synced() {
  {
    MutexGuard lock(state_mutex_);
    host_generation_++;
    random_address_ready_ = false;
    advertising_start_attempts_ = 0;
    advertising_failure_log_time_ = 0;
    host_synced_ = true;
    host_wait_since_ = 0;
    host_wait_log_time_ = 0;
  }
  // start_advertising() configures the random address first and retries that
  // step too if it fails. A failed address setup must not leave us endlessly
  // retrying advertising with no usable random address.
  if (is_advertising_requested()) {
    ESP_LOGI(TAG, "Host synced - starting advertising (requested=true)");
    start_advertising();
  }
}

bool IRKCaptureComponent::register_gatt_services() {
  // THREAD-SAFE: Pass 'this' pointer to BOTH DevInfo characteristics
  // Callback will determine which field to read based on UUID
  devinfo_chrs[0].arg = (void*) this;  // Manufacturer Name
  devinfo_chrs[1].arg = (void*) this;  // Model Number

  // NimBLE fills these in only for the characteristics it registers, in
  // ble_gatts_start(). The Keyboard table has no HR service, so g_hr_handle
  // stays 0 there, which is how notify_hr_if_due() knows there is nothing to
  // notify. Zeroing both first makes that explicit for either registration
  // path, including the Keyboard→Heart Sensor fallback.
  g_hr_handle = 0;
  g_prot_handle = 0;

  // Select GATT services based on current profile
  BLEProfile current_profile;
  {
    MutexGuard lock(state_mutex_);
    current_profile = ble_profile_;
  }

  auto register_svcs = [](struct ble_gatt_svc_def* svcs, const char* profile_name) -> int {
    ESP_LOGI(TAG, "Registering GATT services for %s profile", profile_name);
    int rc = ble_gatts_count_cfg(svcs);
    if (rc == 0) {
      rc = ble_gatts_add_svcs(svcs);
    }
    if (rc != 0) {
      ESP_LOGE(TAG, "GATT registration for %s profile failed rc=%d", profile_name, rc);
    }
    return rc;
  };

  int rc;
  if (current_profile == BLEProfile::KEYBOARD) {
    rc = register_svcs(gatt_svcs_keyboard, "Keyboard");
    if (rc != 0) {
      ESP_LOGW(TAG,
               "Keyboard GATT registration failed; falling back to Heart "
               "Sensor profile "
               "to keep component operational");
      rc = register_svcs(gatt_svcs_heart_sensor, "Heart Sensor");
      if (rc == 0) {
        ESP_LOGW(TAG, "Keyboard->Heart Sensor fallback succeeded");
        // Persist fallback so next boot doesn't repeat the same failure path.
        {
          MutexGuard lock(state_mutex_);
          ble_profile_ = BLEProfile::HEART_SENSOR;
        }
        persist_ble_profile(BLEProfile::HEART_SENSOR);
      }
    }
  } else {
    rc = register_svcs(gatt_svcs_heart_sensor, "Heart Sensor");
  }

  if (rc != 0) {
    ESP_LOGE(TAG,
             "No valid GATT profile could be registered; IRK capture cannot "
             "operate");
    return false;
  }

  // Do NOT snapshot g_hr_handle/g_prot_handle here. ble_gatts_add_svcs() only
  // queues the service definitions; NimBLE assigns characteristic handles later,
  // in ble_gatts_start(), when the host task starts. Any copy taken now is 0
  // forever. Readers use the globals, which NimBLE populates via val_handle.
  return true;
}

void IRKCaptureComponent::notify_hr_if_due(uint32_t now) {
  // Read the handle NimBLE populated during ble_gatts_start(); it is still 0
  // when the Keyboard profile is active (no HR service) or before host start.
  const uint16_t hr_handle = g_hr_handle;
  if (hr_handle == 0) return;
  if (now - last_notify_ <= TimingConfig::HR_NOTIFY_INTERVAL_MS) return;

  // THREAD-SAFE: Copy connection handle before BLE stack call
  uint16_t conn_handle_copy;
  {
    MutexGuard lock(state_mutex_);
    conn_handle_copy = conn_handle_;
  }

  last_notify_ = now;
  uint8_t buf[2];
  size_t len;
  hr_measurement_sample(buf, &len);
  struct os_mbuf* om = ble_hs_mbuf_from_flat(buf, len);
  if (om == nullptr) {
    ESP_LOGD(TAG, "No buffer for heart-rate notification; skipping");
    return;
  }
  int rc = ble_gatts_notify_custom(conn_handle_copy, hr_handle, om);
  if (rc != 0) {
    ESP_LOGD(TAG, "notify rc=%d", rc);
  }
}

void IRKCaptureComponent::report_host_stall_(uint32_t now) {
  // Main task only. Without this a host that never syncs leaves the device
  // idle for ever, with nothing to say why.
  if (host_synced_) return;
  {
    MutexGuard lock(state_mutex_);
    if (host_wait_since_ == 0 ||
        elapsed_ms(now, host_wait_since_) <= TimingConfig::HOST_SYNC_TIMEOUT_MS)
      return;
    if (host_wait_log_time_ != 0 &&
        elapsed_ms(now, host_wait_log_time_) < TimingConfig::HOST_SYNC_LOG_INTERVAL_MS)
      return;
    host_wait_log_time_ = deadline_after(now, 0);
  }
  ESP_LOGE(TAG,
           "Bluetooth host has not synchronised for over %" PRIu32
           " s; nothing can pair. The device restarts itself if this continues; if it "
           "repeats, check the build's Bluetooth configuration.",
           TimingConfig::HOST_SYNC_TIMEOUT_MS / 1000);

  // One bounded recovery attempt. Only when nothing has been captured, so the
  // reboot cannot discard a session.
  bool nothing_to_lose;
  {
    MutexGuard lock(state_mutex_);
    nothing_to_lose = capture_events_ == 0 && host_wait_since_ != 0 &&
                      elapsed_ms(now, host_wait_since_) > TimingConfig::HOST_SYNC_REBOOT_MS;
  }
  if (nothing_to_lose) {
    ESP_LOGE(TAG, "Bluetooth host still not synchronised; rebooting to recover");
    request_reboot_();
  }
}

}  // namespace irk_capture
}  // namespace esphome

#endif  // USE_ESP32
