#include "irk_wizard.h"

#ifdef USE_ESP32

#include <cstring>

#include <lwip/sockets.h>

#include "esphome/components/irk_capture/mutex_guard.h"
#include "esphome/components/network/util.h"
#if defined(USE_WIFI) && defined(USE_WIFI_AP)
#include "esphome/components/wifi/wifi_component.h"
#endif
#include "wizard_util.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace irk_wizard {

static const char* const TAG = "irk_wizard";

// irk_capture's RAII guard, so every lock/unlock pair here is early-return-safe.
using irk_capture::MutexGuard;

static esp_err_t send_json(httpd_req_t* req, const std::string& body);

// Answers with an HTTP error and a JSON {"error": message}. Returns false so a
// check that has to report its own failure can end with "return send_error(...)".
// message must not need JSON escaping.
static bool send_error(httpd_req_t* req, const char* status, const char* message) {
  httpd_resp_set_status(req, status);
  send_json(req, std::string("{\"error\":\"") + message + "\"}");
  return false;
}

static esp_err_t send_unauthorized(httpd_req_t* req) {
  httpd_resp_set_status(req, "401 Unauthorized");
  httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"IRK Capture\"");
  return httpd_resp_send(req, "Unauthorized", HTTPD_RESP_USE_STRLEN);
}

// Identifies the client by its address, for the per-client login throttle.
// Returns 0, a key every unidentified client shares, if the address cannot be
// read; that degrades to one shared throttle rather than to none.
static uint32_t client_key(httpd_req_t* req) {
  struct sockaddr_storage address {};
  socklen_t length = sizeof(address);
  if (getpeername(httpd_req_to_sockfd(req), (struct sockaddr*) &address, &length) != 0) return 0;
  const uint8_t* bytes;
  size_t count;
  if (address.ss_family == AF_INET) {
    bytes = (const uint8_t*) &((const struct sockaddr_in*) &address)->sin_addr;
    count = 4;
#if LWIP_IPV6
    // lwIP only has IPv6 types when the build enables IPv6 (network:
    // enable_ipv6). This also covers an IPv4 client on a dual-stack socket
    // (::ffff:a.b.c.d).
  } else if (address.ss_family == AF_INET6) {
    bytes = (const uint8_t*) &((const struct sockaddr_in6*) &address)->sin6_addr;
    count = 16;
#endif
  } else {
    return 0;
  }
  uint32_t hash = 2166136261u;  // FNV-1a
  for (size_t i = 0; i < count; i++) hash = (hash ^ bytes[i]) * 16777619u;
  return hash != 0 ? hash : 1;
}

// Returns true when the request may proceed. On failure it has already sent
// the response, so the handler must just return ESP_OK.
static bool authorized(httpd_req_t* req) {
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  const std::string& expected = self->expected_auth();
  if (expected.empty()) {
    // The config schema requires credentials, so this means set_auth() never
    // ran. Refuse rather than serve captured keys to anyone who asks.
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Wizard credentials not configured");
    return false;
  }

  // Requests are handled one at a time on httpd's task, so the throttles
  // need no locking.
  const uint32_t now = millis();
  AuthThrottle& throttle = self->client_throttles().for_client(client_key(req), now);
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
  // The buffer is sized from the header. A fixed 64 bytes rejected a valid
  // JSON type as soon as its parameters ran past the end of the buffer.
  static constexpr size_t MAX_CONTENT_TYPE = 256;
  const size_t len = httpd_req_get_hdr_value_len(req, "Content-Type");
  bool is_json = false;
  if (len != 0 && len <= MAX_CONTENT_TYPE) {
    std::string value(len + 1, '\0');
    is_json = httpd_req_get_hdr_value_str(req, "Content-Type", &value[0], len + 1) == ESP_OK &&
              is_json_content_type(value.c_str());
  }
  if (!is_json) {
    httpd_resp_set_status(req, "415 Unsupported Media Type");
    httpd_resp_send(req, "Content-Type must be application/json", HTTPD_RESP_USE_STRLEN);
  }
  return is_json;
}

// Reads the whole request body. As with authorized(), a failure has already
// sent its response, so the handler must just return ESP_OK. Each failure gets
// its own answer: reporting a missing or oversized body as a bad field value
// sends whoever is debugging the request to the wrong place.
static bool read_request_body(httpd_req_t* req, std::string& body) {
  static constexpr size_t MAX_BODY = 512;
  static constexpr int MAX_TIMEOUTS = 3;
  if (req->content_len == 0) return send_error(req, "400 Bad Request", "Request body required");
  if (req->content_len > MAX_BODY) {
    return send_error(req, "413 Content Too Large", "Request body too large");
  }
  body.resize(req->content_len);
  size_t received = 0;
  int timeouts = 0;
  while (received < req->content_len) {
    const int r = httpd_req_recv(req, &body[received], req->content_len - received);
    if (r == HTTPD_SOCK_ERR_TIMEOUT) {
      // A slow link is not a malformed request. Allow a few more receive
      // windows, but not so many that one stalled client holds the server.
      if (++timeouts >= MAX_TIMEOUTS) {
        return send_error(req, "408 Request Timeout", "Timed out reading the request body");
      }
      continue;
    }
    if (r <= 0) {
      // The connection closed or failed, so there is nobody left to answer.
      ESP_LOGW(TAG, "Request body read failed (%d)", r);
      body.clear();
      return false;
    }
    received += (size_t) r;
  }
  return true;
}

// The capture component, when it can act on a command. Otherwise this answers
// 503 and returns nullptr. Saying ok for a command that nothing will carry out
// leaves the page waiting for a change that cannot come.
static irk_capture::IRKCaptureComponent* usable_capture(httpd_req_t* req) {
  auto* capture = static_cast<IRKWizardComponent*>(req->user_ctx)->irk_capture();
  if (capture == nullptr) {
    send_error(req, "503 Service Unavailable", "Capture component unavailable");
    return nullptr;
  }
  if (capture->is_failed()) {
    send_error(req, "503 Service Unavailable",
               "Bluetooth capture failed to start; check the device log, then reboot");
    return nullptr;
  }
  return capture;
}

//======================== Snapshot ========================

void IRKWizardComponent::set_auth(const std::string& username, const std::string& password) {
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

void IRKWizardComponent::set_profile(bool keyboard) {
  // set_ble_profile() publishes the BLE Profile and BLE Device Name entities
  // itself rather than staging them, and entity publishes belong on the main
  // task. Deferring also lets this request answer before the reboot the
  // switch schedules.
  this->defer([this, keyboard]() {
    const auto requested =
        keyboard ? irk_capture::BLEProfile::KEYBOARD : irk_capture::BLEProfile::HEART_SENSOR;
    if (irk_capture_) irk_capture_->set_ble_profile(requested);
    // A failed NVS save restores the old profile and does not reboot. Allow a
    // retry then; successful changes remain reserved until the new boot.
    if (!irk_capture_ || irk_capture_->get_ble_profile() != requested) release_profile_change();
  });
}

void IRKWizardComponent::reboot() {
  // App.safe_reboot() tears down every component, so it must run on the main
  // task. The short delay lets this request's response reach the browser
  // first, and the fixed name folds repeated presses into one reboot.
  this->set_timeout("reboot", 500, []() { App.safe_reboot(); });
}

void IRKWizardComponent::set_advertising(bool on) {
  if (!on) {
    MutexGuard lock(snapshot_mutex_);
    if (capture_start_.state == CaptureStartState::PENDING ||
        capture_start_.state == CaptureStartState::READY)
      capture_start_.state = CaptureStartState::CANCELLED;
  }
  this->defer([this, on]() {
    if (irk_capture_) irk_capture_->set_advertising_requested(on);
  });
}

CaptureStart IRKWizardComponent::start_capture() {
  const uint32_t boot_id = irk_capture_ ? irk_capture_->get_capture_result().boot_id : 0;
  CaptureStart request;
  {
    MutexGuard lock(snapshot_mutex_);
    request.operation_id = capture_start_.operation_id + 1;
    if (request.operation_id == 0) request.operation_id = 1;
    request.state = CaptureStartState::PENDING;
    request.boot_id = boot_id;
    capture_start_ = request;
  }
  this->defer([this, id = request.operation_id]() {
    {
      MutexGuard lock(snapshot_mutex_);
      if (capture_start_.operation_id != id || capture_start_.state != CaptureStartState::PENDING)
        return;
    }
    const bool available = irk_capture_ && !irk_capture_->is_failed();
    // The core takes the baseline and installs ON intent in one state lock.
    // A cached status response could predate a capture by several seconds.
    const auto baseline = available ? irk_capture_->begin_capture() : irk_capture::CaptureResult {};
    MutexGuard lock(snapshot_mutex_);
    // OFF or another start can arrive while the radio operation runs. Its
    // queued main-task command still owns the final intent and acknowledgement.
    if (capture_start_.operation_id != id || capture_start_.state != CaptureStartState::PENDING)
      return;
    capture_start_.boot_id = baseline.boot_id;
    capture_start_.sequence = baseline.sequence;
    capture_start_.state = available ? CaptureStartState::READY : CaptureStartState::FAILED;
  });
  return request;
}

WizardSnapshot IRKWizardComponent::get_snapshot() {
  MutexGuard lock(snapshot_mutex_);
  WizardSnapshot result = snapshot_;
  result.capture_start = capture_start_;
  return result;
}

void IRKWizardComponent::refresh_snapshot_() {
  if (!irk_capture_) return;
  WizardSnapshot next;
  next.status = status_sensor_ ? status_sensor_->state : "idle";
  // One lock on irk_capture's state for the whole snapshot.
  irk_capture::SessionState core = irk_capture_->get_session_state();
  // Shared, not copied. With a long history this is by far the largest value
  // the wizard holds, and every status request used to copy it twice more.
  if (core.history_json.empty()) core.history_json = "[]";
  next.history_json = std::make_shared<const std::string>(std::move(core.history_json));
  next.irk = irk_sensor_ ? irk_sensor_->state : "";
  next.device_mac = device_mac_sensor_ ? device_mac_sensor_->state : "";
  next.effective_mac = effective_mac_sensor_ ? effective_mac_sensor_->state : "";
  next.next_capture_label = core.next_capture_label;
  const bool keyboard = core.profile == irk_capture::BLEProfile::KEYBOARD;
  next.profile = keyboard ? "Keyboard" : "Heart Sensor";
  // get_ble_name() is the name on air, already accounting for the Keyboard
  // profile's "Logitech K380" and for anything that has replaced it since
  // boot. Deciding it here instead would send the user hunting for a name the
  // device stopped advertising the moment the identity was refreshed.
  next.advertised_name = core.advertised_name;
  next.advertising = core.advertising_requested;
  next.stop_after_capture = core.stop_after_capture;
  next.capture_result = core.capture_result;
  next.bond_clear = core.bond_clear;
  next.identity_refresh = core.identity_refresh;

  MutexGuard lock(snapshot_mutex_);
  snapshot_ = std::move(next);
}

//======================== HTTP handlers ========================

static esp_err_t handle_index(httpd_req_t* req);
static esp_err_t handle_get_status(httpd_req_t* req);
static esp_err_t handle_post_advertising(httpd_req_t* req);
static esp_err_t handle_post_capture_start(httpd_req_t* req);
static esp_err_t handle_post_profile(httpd_req_t* req);
static esp_err_t handle_post_label(httpd_req_t* req);
static esp_err_t handle_post_forget_bonds(httpd_req_t* req);
static esp_err_t handle_post_fresh_identity(httpd_req_t* req);
static esp_err_t handle_post_reboot(httpd_req_t* req);
static esp_err_t handle_post_stop_after_capture(httpd_req_t* req);

// The wizard has no reason to be framed or to have its responses reinterpreted
// as another type. Without these, a page on another origin could overlay the
// wizard in a frame and trick a signed-in user into pressing its controls.
static void set_security_headers(httpd_req_t* req) {
  httpd_resp_set_hdr(req, "X-Frame-Options", "DENY");
  httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
}

static void set_json_headers(httpd_req_t* req) {
  httpd_resp_set_type(req, "application/json");
  set_security_headers(req);
  // /api/status carries the captured IRK and the peer's identity address, which
  // permanently deanonymize a phone. no-store keeps them out of the browser's
  // disk cache and out of any intermediary, rather than merely requiring
  // revalidation the way the page's no-cache does.
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
}

static esp_err_t send_json(httpd_req_t* req, const std::string& body) {
  set_json_headers(req);
  return httpd_resp_send(req, body.c_str(), body.size());
}

static esp_err_t send_ok(httpd_req_t* req) {
  return send_json(req, "{\"ok\":true}");
}

// Reject commands whose browser baseline predates an intervening device reboot.
// As with authorized(), failure sends its response before returning false.
static bool matching_boot_baseline(httpd_req_t* req, const std::string& body, uint32_t& boot_id) {
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  boot_id = self->irk_capture() ? self->irk_capture()->get_capture_result().boot_id : 0;
  if (boot_id == 0) {
    // The server starts only after irk_capture's setup(), so a missing boot
    // identity means the component is absent or never initialised. Neither
    // clears on its own; telling the user to retry would be wrong.
    return send_error(req, "503 Service Unavailable",
                      "Capture component unavailable; check the device log");
  }
  // Treat the browser's baseline as an opaque string of digits. The extractor
  // accepts only a complete, quoted value that ends the member.
  std::string expected_boot_id;
  const bool valid_baseline = json_extract_string(body, "expected_boot_id", expected_boot_id) &&
                              !expected_boot_id.empty() && expected_boot_id.size() <= 10 &&
                              expected_boot_id.find_first_not_of("0123456789") == std::string::npos;
  if (!valid_baseline) {
    httpd_resp_set_status(req, "400 Bad Request");
    send_json(req, "{\"error\":\"expected_boot_id must be a quoted boot identity\"}");
    return false;
  }
  if (expected_boot_id != std::to_string(boot_id)) {
    httpd_resp_set_status(req, "409 Conflict");
    send_json(req, "{\"error\":\"Device already restarted; refresh and try again\"}");
    return false;
  }
  return true;
}

static std::string bond_clear_json(const irk_capture::BondClearResult& result) {
  return "{\"operation_id\":" + std::to_string(result.operation_id) + ",\"state\":\"" +
         irk_capture::bond_clear_state_name(result.state) + "\",\"reason\":\"" +
         json_escape(result.reason) +
         "\",\"history_cleared\":" + (result.history_cleared ? "true" : "false") + "}";
}

bool IRKWizardComponent::start_server_() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = port_;
  // Every esp_http_server instance opens an internal UDP control socket on
  // ctrl_port, which defaults to 32768 for ALL instances. ESPHome's own
  // web_server: (port 80) already holds that default, so leaving it here
  // makes our httpd_start() fail with "error in creating ctrl socket (112)".
  // Offset it by our server port to stay clear of that and of each other. The
  // config schema limits port to 32767 so the sum fits the 16-bit field.
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
    { "/api/capture/start", HTTP_POST, handle_post_capture_start },
    { "/api/profile", HTTP_POST, handle_post_profile },
    { "/api/label", HTTP_POST, handle_post_label },
    { "/api/forget_bonds", HTTP_POST, handle_post_forget_bonds },
    { "/api/fresh_identity", HTTP_POST, handle_post_fresh_identity },
    { "/api/reboot", HTTP_POST, handle_post_reboot },
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
      // A server missing a route looks healthy but has controls that return
      // 404. Report the start as failed so loop() retries it as a whole.
      ESP_LOGE(TAG, "Failed to register %s: %s", route.uri, esp_err_to_name(rc));
      httpd_stop(server_);
      server_ = nullptr;
      return false;
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
    // The fallback AP is also a usable interface, even without a station link.
    bool ready = network::is_connected();
#if defined(USE_WIFI) && defined(USE_WIFI_AP)
    ready = ready || (wifi::global_wifi_component && wifi::global_wifi_component->is_ap_active());
#endif
    if (!ready) return;
    uint32_t now = millis();
    if (now - last_start_attempt_ms_ < 500) return;
    last_start_attempt_ms_ = now;
    start_attempts_++;
    // irk_capture has a later setup priority than the wizard. Refresh after
    // all setup() calls, before HTTP can expose the provisional boot ID/state.
    refresh_snapshot_();
    last_snapshot_ms_ = now;
    if (start_server_()) {
      ESP_LOGI(TAG, "Wizard listening on port %u", port_);
    } else if (start_attempts_ >= 20) {
      this->mark_failed(LOG_STR("httpd_start failed after retries"));
    }
    return;
  }

  // Each refresh takes irk_capture's state_mutex_, which the NimBLE task also
  // contends for during pairing, and copies the capture history. Poll briskly
  // only while someone is actually using the wizard; otherwise back right off.
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
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  if (self->page_data() == nullptr || self->page_size() == 0) {
    // An empty 200 renders as a blank page with nothing to explain it.
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "Wizard page missing from this build");
  }
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  set_security_headers(req);
  return httpd_resp_send(req, reinterpret_cast<const char*>(self->page_data()), self->page_size());
}

static esp_err_t handle_get_status(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  WizardSnapshot s = self->get_snapshot();
  // Editable values and consume-once label updates must be visible immediately,
  // even when the main task has not rebuilt its periodic snapshot yet.
  if (self->irk_capture()) {
    s.next_capture_label = self->irk_capture()->get_next_capture_label();
    s.profile = self->irk_capture()->get_ble_profile() == irk_capture::BLEProfile::KEYBOARD
                    ? "Keyboard"
                    : "Heart Sensor";
  }

  std::string head = "{";
  head += "\"boot_id\":" + std::to_string(s.capture_result.boot_id) + ",";
  head += "\"status\":\"" + json_escape(s.status) + "\",";
  head += "\"next_capture_label\":\"" + json_escape(s.next_capture_label) + "\",";
  head += "\"advertised_name\":\"" + json_escape(s.advertised_name) + "\",";
  head += "\"profile\":\"" + json_escape(s.profile) + "\",";
  head += "\"advertising\":" + std::string(s.advertising ? "true" : "false") + ",";
  head += "\"effective_mac\":\"" + json_escape(s.effective_mac) + "\",";
  head += "\"device_mac\":\"" + json_escape(s.device_mac) + "\",";
  head += "\"irk\":\"" + json_escape(s.irk) + "\",";
  head += "\"stop_after_capture\":" + std::string(s.stop_after_capture ? "true" : "false") + ",";
  head += "\"history\":";
  std::string tail =
      ",\"capture_result\":{\"boot_id\":" + std::to_string(s.capture_result.boot_id) +
      ",\"sequence\":" + std::to_string(s.capture_result.sequence) + ",\"irk\":\"" +
      json_escape(s.capture_result.irk) + "\",\"device_mac\":\"" +
      json_escape(s.capture_result.device_mac) + "\"},";
  tail += "\"capture_start\":{\"operation_id\":" + std::to_string(s.capture_start.operation_id) +
          ",\"state\":\"" + capture_start_state_name(s.capture_start.state) +
          "\",\"boot_id\":" + std::to_string(s.capture_start.boot_id) +
          ",\"sequence\":" + std::to_string(s.capture_start.sequence) + "},";
  tail += "\"bond_clear\":" + bond_clear_json(s.bond_clear) + ",";
  tail +=
      "\"identity_refresh\":{\"operation_id\":" + std::to_string(s.identity_refresh.operation_id) +
      ",\"state\":\"" + irk_capture::identity_refresh_state_name(s.identity_refresh.state) + "\"}";
  tail += "}";

  // Sent in pieces. The history can reach tens of kilobytes, and joining it
  // to the rest would need a second buffer of that size for every request. A
  // zero-length chunk ends the response, so no piece may be empty.
  static const std::string EMPTY_HISTORY = "[]";
  const std::string& history =
      s.history_json && !s.history_json->empty() ? *s.history_json : EMPTY_HISTORY;
  set_json_headers(req);
  esp_err_t err = httpd_resp_send_chunk(req, head.c_str(), head.size());
  if (err == ESP_OK) err = httpd_resp_send_chunk(req, history.c_str(), history.size());
  if (err == ESP_OK) err = httpd_resp_send_chunk(req, tail.c_str(), tail.size());
  if (err == ESP_OK) err = httpd_resp_send_chunk(req, nullptr, 0);
  // A failed send returns an error so the server closes the connection; the
  // browser must not take a truncated body for a complete one.
  return err;
}

static esp_err_t handle_post_advertising(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  if (!json_request(req)) return ESP_OK;
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  if (usable_capture(req) == nullptr) return ESP_OK;
  std::string body;
  if (!read_request_body(req, body)) return ESP_OK;
  bool on = false;
  if (!json_extract_bool(body, "on", on)) {
    httpd_resp_set_status(req, "400 Bad Request");
    return send_json(req, "{\"error\":\"on must be a boolean\"}");
  }
  self->set_advertising(on);
  return send_ok(req);
}

static esp_err_t handle_post_capture_start(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  if (!json_request(req)) return ESP_OK;
  const auto start = static_cast<IRKWizardComponent*>(req->user_ctx)->start_capture();
  httpd_resp_set_status(req, "202 Accepted");
  return send_json(req, "{\"operation_id\":" + std::to_string(start.operation_id) +
                            ",\"boot_id\":" + std::to_string(start.boot_id) + "}");
}

static esp_err_t handle_post_profile(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  if (!json_request(req)) return ESP_OK;
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  std::string body;
  if (!read_request_body(req, body)) return ESP_OK;
  std::string profile;
  bool keyboard = false;
  if (!json_extract_string(body, "profile", profile) || !parse_profile_name(profile, keyboard)) {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_send(req, "profile must be \"Heart Sensor\" or \"Keyboard\"", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
  }
  uint32_t boot_id = 0;
  if (!matching_boot_baseline(req, body, boot_id)) return ESP_OK;
  if (!self->try_begin_profile_change()) {
    httpd_resp_set_status(req, "409 Conflict");
    return send_json(req,
                     "{\"error\":\"A profile change is already in progress; retry after reboot\"}");
  }
  const auto requested =
      keyboard ? irk_capture::BLEProfile::KEYBOARD : irk_capture::BLEProfile::HEART_SENSOR;
  const bool reboot_required = self->irk_capture()->get_ble_profile() != requested;
  // Reply before dispatching the main-task save. Persistence can still fail, so
  // the browser confirms a new boot and the selected profile through status.
  const esp_err_t result =
      send_json(req, "{\"ok\":true,\"boot_id\":" + std::to_string(boot_id) +
                         ",\"reboot_required\":" + (reboot_required ? "true" : "false") + "}");
  if (reboot_required)
    self->set_profile(keyboard);
  else
    self->release_profile_change();
  return result;
}

static esp_err_t handle_post_label(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  if (!json_request(req)) return ESP_OK;
  auto* capture = usable_capture(req);
  if (capture == nullptr) return ESP_OK;
  std::string body;
  if (!read_request_body(req, body)) return ESP_OK;
  std::string label;
  if (!json_extract_string(body, "label", label)) {
    httpd_resp_set_status(req, "400 Bad Request");
    return send_json(req, "{\"error\":\"label must be a string\"}");
  }
  const std::string saved_label = capture->set_next_capture_label(label);
  return send_json(req, "{\"ok\":true,\"label\":\"" + json_escape(saved_label) + "\"}");
}

static esp_err_t handle_post_forget_bonds(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  if (!json_request(req)) return ESP_OK;
  auto* capture = usable_capture(req);
  if (capture == nullptr) return ESP_OK;
  const auto result = capture->request_bond_clear();
  std::string json = bond_clear_json(result);
  json.pop_back();
  json += ",\"boot_id\":" + std::to_string(capture->get_capture_result().boot_id) + "}";
  if (result.state == irk_capture::BondClearState::PENDING) {
    httpd_resp_set_status(req, "202 Accepted");
  }
  return send_json(req, json);
}

static esp_err_t handle_post_stop_after_capture(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  if (!json_request(req)) return ESP_OK;
  auto* capture = usable_capture(req);
  if (capture == nullptr) return ESP_OK;
  std::string body;
  if (!read_request_body(req, body)) return ESP_OK;
  bool enabled = false;
  if (!json_extract_bool(body, "enabled", enabled)) {
    httpd_resp_set_status(req, "400 Bad Request");
    return send_json(req, "{\"error\":\"enabled must be a boolean\"}");
  }
  capture->set_stop_after_capture(enabled);
  return send_ok(req);
}

static esp_err_t handle_post_fresh_identity(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  if (!json_request(req)) return ESP_OK;
  if (usable_capture(req) == nullptr) return ESP_OK;
  static_cast<IRKWizardComponent*>(req->user_ctx)->fresh_identity();
  return send_ok(req);
}

static esp_err_t handle_post_reboot(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  if (!json_request(req)) return ESP_OK;
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  std::string body;
  if (!read_request_body(req, body)) return ESP_OK;
  uint32_t boot_id = 0;
  if (!matching_boot_baseline(req, body, boot_id)) return ESP_OK;
  ESP_LOGI(TAG, "Reboot requested from the wizard");
  // Read the core identity, not the periodic snapshot, and finish sending the
  // acknowledgment before starting the reboot delay. A lost response can still
  // accompany a successful reboot; the browser confirms the new boot by polling.
  const esp_err_t result =
      send_json(req, "{\"ok\":true,\"boot_id\":" + std::to_string(boot_id) + "}");
  self->reboot();
  return result;
}

}  // namespace irk_wizard
}  // namespace esphome

#endif  // USE_ESP32
