#include "irk_wizard.h"

#ifdef USE_ESP32

#include <cstring>

#include "esphome/components/network/util.h"
#include "wizard_util.h"
#include "esphome/core/log.h"

namespace esphome {
namespace irk_wizard {

static const char* const TAG = "irk_wizard";

//======================== Thread safety ========================
// Mirrors irk_capture's MutexGuard: RAII wrapper so every lock/unlock pair
// is exception- and early-return-safe.
class MutexGuard {
 public:
  explicit MutexGuard(SemaphoreHandle_t mutex) : mutex_(mutex) {
    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
  }
  ~MutexGuard() {
    if (mutex_) xSemaphoreGive(mutex_);
  }
  MutexGuard(const MutexGuard&) = delete;
  MutexGuard& operator=(const MutexGuard&) = delete;

 private:
  SemaphoreHandle_t mutex_;
};

static esp_err_t send_unauthorized(httpd_req_t* req) {
  httpd_resp_set_status(req, "401 Unauthorized");
  httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"IRK Capture\"");
  return httpd_resp_send(req, "Unauthorized", HTTPD_RESP_USE_STRLEN);
}

// Returns true when the request may proceed. On failure it has already sent
// the response, so the handler must just return ESP_OK.
static bool authorized(httpd_req_t* req) {
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  const std::string& expected = self->expected_auth();
  if (expected.empty()) {
    self->note_request();
    return true;  // auth not configured
  }

  // Requests are handled one at a time on httpd's task, so the throttle
  // needs no locking.
  AuthThrottle& throttle = self->auth_throttle();
  const uint32_t now = millis();
  if (throttle.locked(now)) {
    httpd_resp_set_status(req, "429 Too Many Requests");
    httpd_resp_send(req, "Too many failed attempts, try again shortly", HTTPD_RESP_USE_STRLEN);
    return false;
  }

  size_t len = httpd_req_get_hdr_value_len(req, "Authorization");
  if (len == 0 || len > 256) {
    // No header at all is a browser's first request, not a guess.
    if (len > 256) throttle.fail(now);
    send_unauthorized(req);
    return false;
  }
  std::string got;
  got.resize(len + 1);
  if (httpd_req_get_hdr_value_str(req, "Authorization", &got[0], len + 1) != ESP_OK) {
    send_unauthorized(req);
    return false;
  }
  got.resize(len);
  if (!secure_equals(got, expected)) {
    throttle.fail(now);
    ESP_LOGW(TAG, "Rejected request with bad credentials%s",
             throttle.locked(now) ? "; locking out further attempts briefly" : "");
    send_unauthorized(req);
    return false;
  }
  throttle.ok();
  self->note_request();
  return true;
}

// CSRF defense for the state-changing endpoints. A cross-origin <form> POST
// can only send the CORS-safelisted content types (text/plain,
// multipart/form-data, application/x-www-form-urlencoded); requiring JSON
// forces a preflight, which this server never answers, so a hostile page
// can't reach these handlers even though the browser sends LAN requests.
static bool json_request(httpd_req_t* req) {
  char buf[64];
  if (httpd_req_get_hdr_value_str(req, "Content-Type", buf, sizeof(buf)) != ESP_OK ||
      !is_json_content_type(buf)) {
    httpd_resp_set_status(req, "415 Unsupported Media Type");
    httpd_resp_send(req, "Content-Type must be application/json", HTTPD_RESP_USE_STRLEN);
    return false;
  }
  return true;
}

static std::string read_request_body(httpd_req_t* req) {
  if (req->content_len == 0 || req->content_len > 512) return "";
  std::string body;
  body.resize(req->content_len);
  int received = 0;
  while (received < (int) req->content_len) {
    int r = httpd_req_recv(req, &body[received], req->content_len - received);
    if (r <= 0) return "";
    received += r;
  }
  return body;
}

//======================== Snapshot ========================

void IRKWizardComponent::set_auth(const std::string& username, const std::string& password) {
  if (username.empty() && password.empty()) {
    expected_auth_.clear();
    return;
  }
  expected_auth_ = "Basic " + base64_encode(username + ":" + password);
}

void IRKWizardComponent::fresh_identity() {
  // irk_capture owns the naming: it derives the name from the address that
  // actually took effect, which only the rotation knows. Deferring keeps the
  // BLE work off esp_http_server's task and lets this request answer first, so
  // the page gets its response instead of the connection dropping mid-reply.
  this->defer([this]() {
    if (irk_capture_) irk_capture_->refresh_identity();
  });
}

WizardSnapshot IRKWizardComponent::get_snapshot() {
  MutexGuard lock(snapshot_mutex_);
  return snapshot_;
}

void IRKWizardComponent::refresh_snapshot_() {
  if (!irk_capture_) return;
  WizardSnapshot next;
  next.status = status_sensor_ ? status_sensor_->state : "idle";
  next.history_json = irk_capture_->build_history_json();
  next.irk = irk_sensor_ ? irk_sensor_->state : "";
  next.device_mac = device_mac_sensor_ ? device_mac_sensor_->state : "";
  next.effective_mac = effective_mac_sensor_ ? effective_mac_sensor_->state : "";
  next.next_capture_label = irk_capture_->get_next_capture_label();
  const bool keyboard = irk_capture_->get_ble_profile() == irk_capture::BLEProfile::KEYBOARD;
  next.profile = keyboard ? "Keyboard" : "Heart Sensor";
  // get_ble_name() is the name on air, already accounting for the Keyboard
  // profile's "Logitech K380" and for anything that has replaced it since
  // boot. Deciding it here instead would send the user hunting for a name the
  // device stopped advertising the moment the identity was refreshed.
  next.advertised_name = irk_capture_->get_ble_name();
  next.advertising = irk_capture_->is_advertising_requested();
  next.stop_after_capture = irk_capture_->get_stop_after_capture();
  if (next.history_json.empty()) next.history_json = "[]";

  MutexGuard lock(snapshot_mutex_);
  snapshot_ = next;
}

//======================== HTTP handlers ========================

static esp_err_t handle_index(httpd_req_t* req);
static esp_err_t handle_get_status(httpd_req_t* req);
static esp_err_t handle_post_advertising(httpd_req_t* req);
static esp_err_t handle_post_profile(httpd_req_t* req);
static esp_err_t handle_post_label(httpd_req_t* req);
static esp_err_t handle_post_forget_bonds(httpd_req_t* req);
static esp_err_t handle_post_fresh_identity(httpd_req_t* req);
static esp_err_t handle_post_stop_after_capture(httpd_req_t* req);

static esp_err_t send_json(httpd_req_t* req, const std::string& body) {
  httpd_resp_set_type(req, "application/json");
  // /api/status carries the captured IRK and the peer's identity address, which
  // permanently deanonymize a phone. no-store keeps them out of the browser's
  // disk cache and out of any intermediary, rather than merely requiring
  // revalidation the way the page's no-cache does.
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, body.c_str(), body.size());
}

static esp_err_t send_ok(httpd_req_t* req) {
  return send_json(req, "{\"ok\":true}");
}

bool IRKWizardComponent::start_server_() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = port_;
  // Every esp_http_server instance opens an internal UDP control socket on
  // ctrl_port, which defaults to 32768 for ALL instances. ESPHome's own
  // web_server: (port 80) already holds that default, so leaving it here
  // makes our httpd_start() fail with "error in creating ctrl socket (112)".
  // Offset it by our server port to stay clear of that and of each other.
  config.ctrl_port = 32768 + port_;
  config.max_uri_handlers = 12;
  config.lru_purge_enable = true;
  // This is a single-user on-device tool, not a public server - keep our
  // socket footprint small since web_server:, API, OTA and mDNS are all
  // competing for the same LWIP socket ceiling (see CONFIG_LWIP_MAX_SOCKETS
  // in __init__.py). Default is 7; we only ever expect one browser tab.
  config.max_open_sockets = 4;

  esp_err_t err = httpd_start(&server_, &config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "httpd_start failed on port %u: %s", port_, esp_err_to_name(err));
    return false;
  }

  const struct {
    const char* uri;
    httpd_method_t method;
    esp_err_t (*handler)(httpd_req_t*);
  } routes[] = {
    { "/", HTTP_GET, handle_index },
    { "/api/status", HTTP_GET, handle_get_status },
    { "/api/advertising", HTTP_POST, handle_post_advertising },
    { "/api/profile", HTTP_POST, handle_post_profile },
    { "/api/label", HTTP_POST, handle_post_label },
    { "/api/forget_bonds", HTTP_POST, handle_post_forget_bonds },
    { "/api/fresh_identity", HTTP_POST, handle_post_fresh_identity },
    { "/api/stop_after_capture", HTTP_POST, handle_post_stop_after_capture },
  };

  for (const auto& route : routes) {
    httpd_uri_t uri_handler = {};
    uri_handler.uri = route.uri;
    uri_handler.method = route.method;
    uri_handler.handler = route.handler;
    uri_handler.user_ctx = this;
    esp_err_t rc = httpd_register_uri_handler(server_, &uri_handler);
    if (rc != ESP_OK) {
      ESP_LOGE(TAG, "Failed to register %s: %s", route.uri, esp_err_to_name(rc));
    }
  }
  return true;
}

void IRKWizardComponent::setup() {
  snapshot_mutex_ = xSemaphoreCreateMutex();
  if (!snapshot_mutex_) {
    ESP_LOGE(TAG, "Failed to create snapshot mutex");
    this->mark_failed(LOG_STR("snapshot mutex allocation failed"));
    return;
  }
  refresh_snapshot_();
  // Starting httpd here (even at AFTER_WIFI priority) is too early: all
  // components' setup() run back-to-back in one pass, and LWIP's TCP/IP core
  // isn't reliably up yet - httpd_start() fails to create its internal
  // control socket ("error in creating ctrl socket"). loop() only begins
  // once every component has finished setup(), which is late enough.
}

void IRKWizardComponent::loop() {
  if (!server_ && start_attempts_ < 20) {
    // httpd's internal control socket connects to 127.0.0.1; on this
    // ESP-IDF/LWIP integration that fails with EHOSTDOWN until the default
    // network interface (WiFi STA) is actually up - a loopback pseudo-netif
    // existing (CONFIG_LWIP_NETIF_LOOPBACK) is not sufficient on its own.
    // Gate on network::is_connected() instead of guessing a fixed delay.
    if (!network::is_connected()) return;
    uint32_t now = millis();
    if (now - last_start_attempt_ms_ < 500) return;
    last_start_attempt_ms_ = now;
    start_attempts_++;
    if (start_server_()) {
      ESP_LOGI(TAG, "Wizard listening on port %u", port_);
    } else if (start_attempts_ >= 20) {
      this->mark_failed(LOG_STR("httpd_start failed after retries"));
    }
    return;
  }

  // Each refresh takes irk_capture's state_mutex_ several times, which the
  // NimBLE task also contends for during pairing. Poll briskly only while
  // someone is actually using the wizard; otherwise back right off.
  uint32_t now = millis();
  const bool active = (now - last_request_ms_.load(std::memory_order_relaxed)) < 30000;
  if (now - last_snapshot_ms_ < (active ? 500 : 5000)) return;
  last_snapshot_ms_ = now;
  refresh_snapshot_();
}

void IRKWizardComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "IRK Capture Wizard:");
  ESP_LOGCONFIG(TAG, "  Port: %u", port_);
}

//======================== Page ========================
// The page lives in wizard_page.html. __init__.py gzips it at build time and
// hands it over as a flash array, so it costs about a quarter of its source
// size and is sent as-is with Content-Encoding: gzip.

static esp_err_t handle_index(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  httpd_resp_set_type(req, "text/html");
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, reinterpret_cast<const char*>(self->page_data()), self->page_size());
}

static esp_err_t handle_get_status(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  WizardSnapshot s = self->get_snapshot();

  std::string json = "{";
  json += "\"status\":\"" + json_escape(s.status) + "\",";
  json += "\"next_capture_label\":\"" + json_escape(s.next_capture_label) + "\",";
  json += "\"advertised_name\":\"" + json_escape(s.advertised_name) + "\",";
  json += "\"profile\":\"" + json_escape(s.profile) + "\",";
  json += "\"advertising\":" + std::string(s.advertising ? "true" : "false") + ",";
  json += "\"effective_mac\":\"" + json_escape(s.effective_mac) + "\",";
  json += "\"device_mac\":\"" + json_escape(s.device_mac) + "\",";
  json += "\"irk\":\"" + json_escape(s.irk) + "\",";
  json += "\"stop_after_capture\":" + std::string(s.stop_after_capture ? "true" : "false") + ",";
  json += "\"history\":" + (s.history_json.empty() ? std::string("[]") : s.history_json);
  json += "}";
  return send_json(req, json);
}

static esp_err_t handle_post_advertising(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  if (!json_request(req)) return ESP_OK;
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  std::string body = read_request_body(req);
  bool on = false;
  if (json_extract_bool(body, "on", on) && self->irk_capture()) {
    self->irk_capture()->set_advertising_requested(on);
  }
  return send_ok(req);
}

static esp_err_t handle_post_profile(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  if (!json_request(req)) return ESP_OK;
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  std::string body = read_request_body(req);
  std::string profile;
  if (json_extract_string(body, "profile", profile) && self->irk_capture()) {
    self->irk_capture()->set_ble_profile(profile == "Keyboard"
                                             ? irk_capture::BLEProfile::KEYBOARD
                                             : irk_capture::BLEProfile::HEART_SENSOR);
  }
  return send_ok(req);
}

static esp_err_t handle_post_label(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  if (!json_request(req)) return ESP_OK;
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  std::string body = read_request_body(req);
  std::string label;
  if (json_extract_string(body, "label", label) && self->irk_capture()) {
    self->irk_capture()->set_next_capture_label(label);
  }
  return send_ok(req);
}

static esp_err_t handle_post_forget_bonds(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  if (!json_request(req)) return ESP_OK;
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  if (self->irk_capture()) self->irk_capture()->forget_all_bonds();
  return send_ok(req);
}

static esp_err_t handle_post_stop_after_capture(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  if (!json_request(req)) return ESP_OK;
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  std::string body = read_request_body(req);
  bool enabled = false;
  if (json_extract_bool(body, "enabled", enabled) && self->irk_capture()) {
    self->irk_capture()->set_stop_after_capture(enabled);
  }
  return send_ok(req);
}

static esp_err_t handle_post_fresh_identity(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  if (!json_request(req)) return ESP_OK;
  static_cast<IRKWizardComponent*>(req->user_ctx)->fresh_identity();
  return send_ok(req);
}

}  // namespace irk_wizard
}  // namespace esphome

#endif  // USE_ESP32
