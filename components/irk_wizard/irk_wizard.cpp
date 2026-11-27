#include "irk_wizard.h"

#ifdef USE_ESP32

#include <cstdlib>
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

// Answers with an HTTP error and a JSON {"error": message}, and returns what
// the send returned so a handler can end with "return reply_error(...)".
// message must not need JSON escaping.
static esp_err_t reply_error(httpd_req_t* req, const char* status, const char* message) {
  ESP_LOGW(TAG, "Request refused (%s): %s", status, message);
  httpd_resp_set_status(req, status);
  return send_json(req, std::string("{\"error\":\"") + message + "\"}");
}

// The same answer for a check that reports its own failure: always false, so
// the check can end with "return send_error(...)".
static bool send_error(httpd_req_t* req, const char* status, const char* message) {
  reply_error(req, status, message);
  return false;
}

static void mark_close_if_unread(httpd_req_t* req) {
  if (req->content_len != 0) httpd_resp_set_hdr(req, "Connection", "close");
}

// The wizard has no reason to be framed or to have its responses reinterpreted
// as another type. Without these, a page on another origin could overlay the
// wizard in a frame and trick a signed-in user into pressing its controls.
// The page is one inline document that only fetches its own API, so the policy
// allows exactly that; frame-ancestors is the modern form of X-Frame-Options,
// which stays for older browsers. httpd keeps at most max_resp_headers (8)
// extra headers per response; the busiest, a 429 or a 401, send 7.
static void set_security_headers(httpd_req_t* req) {
  httpd_resp_set_hdr(req, "Content-Security-Policy",
                     "default-src 'none'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; "
                     "connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; "
                     "form-action 'none'");
  httpd_resp_set_hdr(req, "Referrer-Policy", "no-referrer");
  httpd_resp_set_hdr(req, "X-Frame-Options", "DENY");
  httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
}

static esp_err_t send_unauthorized(httpd_req_t* req) {
  mark_close_if_unread(req);
  httpd_resp_set_status(req, "401 Unauthorized");
  // Seven headers with Connection: close, within max_resp_headers (8).
  httpd_resp_set_type(req, "text/plain");
  set_security_headers(req);
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
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

// Returning ESP_OK makes httpd drain any unread bytes on its server task.
// After a rejection, close any session whose request body was not read.
static esp_err_t unread_body_result(const httpd_req_t* req) {
  return req->content_len == 0 ? ESP_OK : ESP_FAIL;
}

static bool bodyless_request(httpd_req_t* req) {
  if (req->content_len == 0) return true;
  mark_close_if_unread(req);
  return send_error(req, "400 Bad Request", "Unexpected request body");
}

// Returns true when the request may proceed. On failure the response has been sent.
static bool authorized(httpd_req_t* req) {
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  const std::string& expected = self->expected_auth();
  if (expected.empty()) {
    // The config schema requires credentials, so this means set_auth() never
    // ran. Refuse rather than serve captured keys to anyone who asks.
    mark_close_if_unread(req);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Wizard credentials not configured");
    return false;
  }

  // Requests are handled one at a time on httpd's task, so the throttles
  // need no locking.
  const uint32_t now = millis();
  AuthThrottle& throttle = self->client_throttles().for_client(client_key(req), now);
  if (throttle.locked(now)) {
    // The header value must outlive this call; httpd keeps only the pointer.
    static const std::string RETRY_AFTER = std::to_string(AuthThrottle::LOCKOUT_MS / 1000);
    mark_close_if_unread(req);
    httpd_resp_set_hdr(req, "Retry-After", RETRY_AFTER.c_str());
    return send_error(req, "429 Too Many Requests", "Too many failed attempts, try again shortly");
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
  self->client_throttles().succeeded(throttle, now);
  self->note_request();
  return true;
}

// CSRF defense for the state-changing endpoints. A cross-origin <form> POST
// can only send the CORS-safelisted content types (text/plain,
// multipart/form-data, application/x-www-form-urlencoded); requiring JSON
// forces a preflight, which this server never answers, so a hostile page
// can't reach these handlers even though the browser sends LAN requests.
static bool json_request(httpd_req_t* req) {
  // Cap header allocation while allowing long Content-Type parameters.
  static constexpr size_t MAX_CONTENT_TYPE = 256;
  const size_t len = httpd_req_get_hdr_value_len(req, "Content-Type");
  bool is_json = false;
  if (len != 0 && len <= MAX_CONTENT_TYPE) {
    std::string value(len + 1, '\0');
    is_json = httpd_req_get_hdr_value_str(req, "Content-Type", &value[0], len + 1) == ESP_OK &&
              is_json_content_type(value.c_str());
  }
  if (!is_json) {
    mark_close_if_unread(req);
    send_error(req, "415 Unsupported Media Type", "Content-Type must be application/json");
  }
  return is_json;
}

// Reads the whole request body, answering failures when the socket is usable.
static bool read_request_body(httpd_req_t* req, std::string& body) {
  static constexpr size_t MAX_BODY = 512;
  static constexpr int MAX_TIMEOUTS = 3;
  // The timeout count alone can't stop a client that sends a byte just before
  // each receive window closes; that would hold the server for many minutes.
  // Stop after the receive window that reaches the deadline, even if it
  // delivered the last bytes.
  static constexpr uint32_t MAX_BODY_READ_MS = 10000;
  if (req->content_len == 0) return send_error(req, "400 Bad Request", "Request body required");
  if (req->content_len > MAX_BODY) {
    mark_close_if_unread(req);
    return send_error(req, "413 Content Too Large", "Request body too large");
  }
  body.resize(req->content_len);
  size_t received = 0;
  int timeouts = 0;
  const uint32_t started = millis();
  while (received < req->content_len) {
    const int r = httpd_req_recv(req, &body[received], req->content_len - received);
    const bool timed_out = r == HTTPD_SOCK_ERR_TIMEOUT;
    if (r <= 0 && !timed_out) {
      // The connection closed or failed, so there is nobody left to answer.
      ESP_LOGW(TAG, "Request body read failed (%d)", r);
      body.clear();
      return false;
    }
    if (millis() - started >= MAX_BODY_READ_MS || (timed_out && ++timeouts >= MAX_TIMEOUTS)) {
      mark_close_if_unread(req);
      return send_error(req, "408 Request Timeout", "Timed out reading the request body");
    }
    if (!timed_out) received += (size_t) r;
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

// Check login, content type, request body, and capture availability. On
// failure, command.error_result is ESP_FAIL whenever bytes remain unread so
// httpd closes the session instead of draining untrusted input on its task.
struct Command {
  std::string body;
  irk_capture::IRKCaptureComponent* capture { nullptr };
  esp_err_t error_result { ESP_OK };
};
enum : uint8_t { NEEDS_CAPTURE = 1, NEEDS_BODY = 2 };

static bool begin_command(httpd_req_t* req, uint8_t needs, Command& command) {
  command.error_result = unread_body_result(req);
  if (!authorized(req) || !json_request(req)) return false;
  // The page sends {} even for commands that take no arguments. Read any
  // supplied body through the size/time bounds before checking capture health
  // so an authenticated 503 drains cleanly instead of closing with unread bytes
  // (which triggers a TCP RST on lwIP and can hide the error message).
  if ((needs & NEEDS_BODY) || req->content_len != 0) {
    if (!read_request_body(req, command.body)) return false;
  }
  command.error_result = ESP_OK;
  if (needs & NEEDS_CAPTURE) {
    command.capture = usable_capture(req);
    if (command.capture == nullptr) return false;
  }
  return true;
}

//======================== Snapshot ========================

void IRKWizardComponent::set_auth(const std::string& username, const std::string& password) {
  expected_auth_ = "Basic " + base64_encode(username + ":" + password);
}

void IRKWizardComponent::fresh_identity() {
  // Defer to the main task to keep BLE work off httpd's task and serialize
  // identity refreshes with Home Assistant controls.
  this->defer([this]() {
    ESP_LOGI(TAG, "Wizard: refresh BLE identity requested");
    if (irk_capture_) irk_capture_->refresh_identity();
  });
}

void IRKWizardComponent::set_profile(bool keyboard) {
  // set_ble_profile() publishes entities directly (main task only) and
  // schedules a reboot; deferring keeps both off httpd's task.
  this->defer([this, keyboard]() {
    const auto requested =
        keyboard ? irk_capture::BLEProfile::KEYBOARD : irk_capture::BLEProfile::HEART_SENSOR;
    ESP_LOGI(TAG, "Wizard: BLE profile %s requested", keyboard ? "Keyboard" : "Heart Sensor");
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
    ESP_LOGI(TAG, "Wizard: advertising %s", on ? "on" : "off");
    if (irk_capture_) irk_capture_->set_advertising_requested(on);
  });
}

bool IRKWizardComponent::stop_capture(uint32_t operation_id, uint32_t boot_id) {
  {
    // A tab whose run another tab replaced must not cancel the newer run or
    // close its peer. Start and stop both run on the single httpd task, so no
    // new start can land between this check and the cancellation below.
    MutexGuard lock(snapshot_mutex_);
    if (operation_id != 0 &&
        (capture_start_.operation_id != operation_id || capture_start_.boot_id != boot_id))
      return false;
    // Cancel the run in the same lock scope as the check, named or not. The
    // deferred START checks and arms under this lock too, so it either armed
    // already, and cancel_capture() below undoes that, or it sees CANCELLED
    // and never arms. Either way it cannot re-arm after this stop replies.
    if (capture_start_.state == CaptureStartState::PENDING ||
        capture_start_.state == CaptureStartState::READY)
      capture_start_.state = CaptureStartState::CANCELLED;
  }
  // Cancel the result before acknowledging the stop. The next label can
  // arrive before the NimBLE host has finished closing the old link.
  if (irk_capture_) irk_capture_->cancel_capture();
  set_advertising(false);
  return true;
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
    ESP_LOGI(TAG, "Wizard: capture start requested");
    const bool available = irk_capture_ && !irk_capture_->is_failed();
    irk_capture::CaptureResult baseline;
    {
      // Check the run and arm capture as one step: stop_capture() cancels
      // under this lock, so it cannot land between them. The core takes the
      // baseline and installs ON intent in one state lock, with no BLE call,
      // so this holds snapshot_mutex_ -> state_mutex_ for microseconds. A
      // cached status response could predate a capture by several seconds.
      MutexGuard lock(snapshot_mutex_);
      if (capture_start_.operation_id != id || capture_start_.state != CaptureStartState::PENDING)
        return;
      if (available) baseline = irk_capture_->arm_capture();
    }
    // Outside the lock: the start can wait for the BLE lock. A stop that lands
    // now has set intent OFF, which start_advertising() checks first.
    if (available) irk_capture_->start_advertising();
    MutexGuard lock(snapshot_mutex_);
    // OFF or another start can arrive while the radio operation runs. Its
    // queued main-task command still owns the final intent and acknowledgement.
    if (capture_start_.operation_id != id || capture_start_.state != CaptureStartState::PENDING)
      return;
    capture_start_.boot_id = baseline.boot_id;
    capture_start_.sequence = baseline.sequence;
    capture_start_.connection_generation = baseline.connection_generation;
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
  // One lock on irk_capture's state for the whole snapshot. An unchanged
  // history comes back empty, so keep the one already shared.
  irk_capture::SessionState core = irk_capture_->get_session_state(history_version_);
  if (core.history_changed) {
    // Shared, not copied. With a long history this is by far the largest value
    // the wizard holds, and every status request used to copy it twice more.
    if (core.history_json.empty()) core.history_json = "[]";
    next.history_json = std::make_shared<const std::string>(std::move(core.history_json));
  } else {
    MutexGuard lock(snapshot_mutex_);
    next.history_json = snapshot_.history_json;
  }
  history_version_ = core.history_version;
  next.history_version = core.history_version;
  next.irk = irk_sensor_ ? irk_sensor_->state : "";
  next.device_mac = device_mac_sensor_ ? device_mac_sensor_->state : "";
  next.effective_mac = effective_mac_sensor_ ? effective_mac_sensor_->state : "";
  // core is a local copy; move its strings rather than copying them again.
  next.next_capture_label = std::move(core.next_capture_label);
  const bool keyboard = core.profile == irk_capture::BLEProfile::KEYBOARD;
  next.profile = keyboard ? "Keyboard" : "Heart Sensor";
  // Capture owns the selected advertising name, including profile defaults,
  // renames, and identity refreshes. It can select a name while the radio is off.
  next.advertised_name = std::move(core.advertised_name);
  next.advertising = core.advertising_requested;
  next.stop_after_capture = core.stop_after_capture;
  next.capture_result = std::move(core.capture_result);
  next.bond_clear = std::move(core.bond_clear);
  next.identity_refresh = std::move(core.identity_refresh);

  MutexGuard lock(snapshot_mutex_);
  snapshot_ = std::move(next);
}

//======================== HTTP handlers ========================

static esp_err_t handle_index(httpd_req_t* req);
static esp_err_t handle_get_status(httpd_req_t* req);
static esp_err_t handle_post_advertising(httpd_req_t* req);
static esp_err_t handle_post_capture_start(httpd_req_t* req);
static esp_err_t handle_post_capture_stop(httpd_req_t* req);
static esp_err_t handle_post_profile(httpd_req_t* req);
static esp_err_t handle_post_label(httpd_req_t* req);
static esp_err_t handle_post_forget_bonds(httpd_req_t* req);
static esp_err_t handle_post_fresh_identity(httpd_req_t* req);
static esp_err_t handle_post_reboot(httpd_req_t* req);
static esp_err_t handle_post_stop_after_capture(httpd_req_t* req);

static void set_json_headers(httpd_req_t* req) {
  httpd_resp_set_type(req, "application/json");
  set_security_headers(req);
  // /api/status returns sensitive IRKs and identity MACs; no-store prevents
  // disk/proxy caching (unlike the HTML page's no-cache).
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
    return send_error(req, "400 Bad Request", "expected_boot_id must be a quoted boot identity");
  }
  if (expected_boot_id != std::to_string(boot_id)) {
    return send_error(req, "409 Conflict", "Device already restarted; refresh and try again");
  }
  return true;
}

// Appends the bond-clear object to out, so the status response can build it in
// its own buffer instead of joining temporaries.
static void append_bond_clear_json(std::string& out, const irk_capture::BondClearResult& result) {
  out += "{\"operation_id\":";
  out += std::to_string(result.operation_id);
  out += ",\"state\":\"";
  out += irk_capture::bond_clear_state_name(result.state);
  out += "\",\"reason\":\"";
  json_escape_into(out, result.reason);
  out += "\",\"history_cleared\":";
  out += result.history_cleared ? "true" : "false";
  out += '}';
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
    { "/api/capture/stop", HTTP_POST, handle_post_capture_stop },
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
  // contends for during pairing, and copies the capture history whenever it
  // has changed. Poll briskly
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
// __init__.py gzips wizard_page.html at build time into a flash array.
// The HTTP handler sends that array as-is with Content-Encoding: gzip.

static esp_err_t handle_index(httpd_req_t* req) {
  if (!authorized(req) || !bodyless_request(req)) return unread_body_result(req);
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

// True when the page's ?history=<boot_id>.<version> names the history this
// snapshot holds, so the response can leave it out. Anything else, including
// a token from before a reboot, gets the full history.
static bool history_unchanged(httpd_req_t* req, const WizardSnapshot& s) {
  if (!s.history_json) return false;
  char query[64];
  const size_t len = httpd_req_get_url_query_len(req);
  if (len == 0 || len >= sizeof(query)) return false;
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;
  char token[24];
  if (httpd_query_key_value(query, "history", token, sizeof(token)) != ESP_OK) return false;
  // Digits, one dot, digits: strtoul alone would also accept signs and spaces.
  const auto number = [](const char* p, char stop, const char** end, uint32_t& out) {
    if (*p < '0' || *p > '9') return false;
    char* e;
    const unsigned long long value = std::strtoull(p, &e, 10);
    if (*e != stop || value > UINT32_MAX) return false;
    out = static_cast<uint32_t>(value);
    *end = e;
    return true;
  };
  const char* p = token;
  uint32_t boot_id, version;
  if (!number(p, '.', &p, boot_id) || !number(p + 1, '\0', &p, version)) return false;
  return boot_id == s.capture_result.boot_id && version == s.history_version;
}

static esp_err_t handle_get_status(httpd_req_t* req) {
  if (!authorized(req) || !bodyless_request(req)) return unread_body_result(req);
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  WizardSnapshot s = self->get_snapshot();
  // Editable values and consume-once label updates must be visible immediately,
  // even when the main task has not rebuilt its periodic snapshot yet. The
  // same goes for the controls: the page refreshes right after a command, and
  // a start acknowledged as ready next to stale advertising=false would tell
  // the user that advertising is off.
  if (self->irk_capture()) {
    irk_capture::LiveControls live = self->irk_capture()->get_live_controls();
    s.next_capture_label = std::move(live.next_capture_label);
    s.profile = live.profile == irk_capture::BLEProfile::KEYBOARD ? "Keyboard" : "Heart Sensor";
    s.advertising = live.advertising_requested;
    s.stop_after_capture = live.stop_after_capture;
  }

  // Appended in place: chained + would build a temporary string for nearly
  // every field, on every poll, for the life of the device.
  const auto add_string = [](std::string& out, const char* key, const std::string& value) {
    out += key;
    json_escape_into(out, value);
    out += '"';
  };
  const auto add_number = [](std::string& out, const char* key, uint32_t value) {
    out += key;
    out += std::to_string(value);
  };
  const auto add_bool = [](std::string& out, const char* key, bool value) {
    out += key;
    out += value ? "true" : "false";
  };
  const bool skip_history = history_unchanged(req, s);
  std::string head;
  head.reserve(416);
  add_number(head, "{\"boot_id\":", s.capture_result.boot_id);
  add_string(head, ",\"status\":\"", s.status);
  add_string(head, ",\"next_capture_label\":\"", s.next_capture_label);
  add_string(head, ",\"advertised_name\":\"", s.advertised_name);
  add_string(head, ",\"profile\":\"", s.profile);
  add_bool(head, ",\"advertising\":", s.advertising);
  add_string(head, ",\"effective_mac\":\"", s.effective_mac);
  add_string(head, ",\"device_mac\":\"", s.device_mac);
  add_string(head, ",\"irk\":\"", s.irk);
  add_bool(head, ",\"stop_after_capture\":", s.stop_after_capture);
  // The page sends this back as ?history=; see history_unchanged().
  add_number(head, ",\"history_version\":\"", s.capture_result.boot_id);
  add_number(head, ".", s.history_version);
  head += skip_history ? "\",\"history_unchanged\":true" : "\",\"history\":";

  // 526 bytes with every ID at 10 digits, a full IRK and MAC, and the longest
  // state names; the two reasons are the only variable-length values. Sized
  // once so the tail never reallocates; it used to outgrow 384 bytes on the
  // first poll (390 at startup, 449 after a capture).
  static constexpr size_t TAIL_FIXED_MAX = 528;
  std::string tail;
  tail.reserve(TAIL_FIXED_MAX + s.bond_clear.reason.size() +
               s.identity_refresh.cleanup_reason.size());
  add_number(tail, ",\"capture_result\":{\"boot_id\":", s.capture_result.boot_id);
  add_number(tail, ",\"sequence\":", s.capture_result.sequence);
  add_string(tail, ",\"irk\":\"", s.capture_result.irk);
  add_string(tail, ",\"device_mac\":\"", s.capture_result.device_mac);
  tail += ",\"verification\":\"";
  tail += irk_capture::irk_check_name(s.capture_result.verification);
  add_number(tail, "\",\"connection_generation\":", s.capture_result.connection_generation);
  add_number(tail, "},\"capture_start\":{\"operation_id\":", s.capture_start.operation_id);
  tail += ",\"state\":\"";
  tail += capture_start_state_name(s.capture_start.state);
  add_number(tail, "\",\"boot_id\":", s.capture_start.boot_id);
  add_number(tail, ",\"sequence\":", s.capture_start.sequence);
  add_number(tail, ",\"connection_generation\":", s.capture_start.connection_generation);
  tail += "},\"bond_clear\":";
  append_bond_clear_json(tail, s.bond_clear);
  add_number(tail, ",\"identity_refresh\":{\"operation_id\":", s.identity_refresh.operation_id);
  tail += ",\"state\":\"";
  tail += irk_capture::identity_refresh_state_name(s.identity_refresh.state);
  add_string(tail, "\",\"cleanup_reason\":\"", s.identity_refresh.cleanup_reason);
  tail += "}}";

  // Sent in pieces. The history can reach tens of kilobytes, and joining it
  // to the rest would need a second buffer of that size for every request. A
  // zero-length chunk ends the response, so no piece may be empty.
  static const std::string EMPTY_HISTORY = "[]";
  const std::string& history =
      s.history_json && !s.history_json->empty() ? *s.history_json : EMPTY_HISTORY;
  set_json_headers(req);
  esp_err_t err = httpd_resp_send_chunk(req, head.c_str(), head.size());
  if (err == ESP_OK && !skip_history)
    err = httpd_resp_send_chunk(req, history.c_str(), history.size());
  if (err == ESP_OK) err = httpd_resp_send_chunk(req, tail.c_str(), tail.size());
  if (err == ESP_OK) err = httpd_resp_send_chunk(req, nullptr, 0);
  // A failed send returns an error so the server closes the connection; the
  // browser must not take a truncated body for a complete one.
  return err;
}

static esp_err_t handle_post_advertising(httpd_req_t* req) {
  Command command;
  if (!begin_command(req, NEEDS_CAPTURE | NEEDS_BODY, command)) return command.error_result;
  bool on = false;
  if (!json_extract_bool(command.body, "on", on)) {
    return reply_error(req, "400 Bad Request", "on must be a boolean");
  }
  static_cast<IRKWizardComponent*>(req->user_ctx)->set_advertising(on);
  return send_ok(req);
}

static esp_err_t handle_post_capture_start(httpd_req_t* req) {
  // Without the capture check the request is accepted and the page is told only
  // that the start "failed", with advice to retry something that needs a reboot.
  Command command;
  if (!begin_command(req, NEEDS_CAPTURE, command)) return command.error_result;
  const auto start = static_cast<IRKWizardComponent*>(req->user_ctx)->start_capture();
  httpd_resp_set_status(req, "202 Accepted");
  return send_json(req, "{\"operation_id\":" + std::to_string(start.operation_id) +
                            ",\"boot_id\":" + std::to_string(start.boot_id) + "}");
}

// Reads an optional quoted decimal ID, the form boot IDs already use. An
// absent key leaves 0; a present one must be a nonzero 32-bit value.
static bool optional_quoted_id(const std::string& body, const char* key, uint32_t& out) {
  out = 0;
  size_t pos;
  if (!json_find_value_start(body, key, pos)) {
    // Absent from a well-formed object, or no body at all. A malformed body
    // or a duplicated key is rejected rather than read as an unnamed stop.
    if (body.empty()) return true;
    size_t end = 0;
    json_skip_ws(body, end);
    if (end >= body.size() || body[end] != '{' || !json_skip_value(body, end)) return false;
    json_skip_ws(body, end);
    return end == body.size() && body.find(std::string("\"") + key + "\"") == std::string::npos;
  }
  std::string text;
  if (!json_extract_string(body, key, text) || text.empty() || text.size() > 10 ||
      text.find_first_not_of("0123456789") != std::string::npos)
    return false;
  const unsigned long long value = strtoull(text.c_str(), nullptr, 10);
  if (value == 0 || value > UINT32_MAX) return false;
  out = static_cast<uint32_t>(value);
  return true;
}

static esp_err_t handle_post_capture_stop(httpd_req_t* req) {
  Command command;
  if (!begin_command(req, NEEDS_CAPTURE, command)) return command.error_result;
  uint32_t operation_id = 0;
  uint32_t boot_id = 0;
  if (!optional_quoted_id(command.body, "operation_id", operation_id) ||
      !optional_quoted_id(command.body, "boot_id", boot_id) ||
      (operation_id == 0) != (boot_id == 0)) {
    return reply_error(req, "400 Bad Request",
                       "operation_id and boot_id must be quoted IDs, sent together");
  }
  // A stop for a replaced run is already satisfied from that page's view.
  const bool stopped =
      static_cast<IRKWizardComponent*>(req->user_ctx)->stop_capture(operation_id, boot_id);
  return send_json(req,
                   std::string("{\"ok\":true,\"stopped\":") + (stopped ? "true" : "false") + "}");
}

static esp_err_t handle_post_profile(httpd_req_t* req) {
  // A failed capture component still accepts a profile change: switching
  // profile and rebooting is the way out of a GATT table that will not register.
  Command command;
  if (!begin_command(req, NEEDS_BODY, command)) return command.error_result;
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  const std::string& body = command.body;
  std::string profile;
  bool keyboard = false;
  if (!json_extract_string(body, "profile", profile) || !parse_profile_name(profile, keyboard)) {
    send_error(req, "400 Bad Request", "profile must be Heart Sensor or Keyboard");
    return ESP_OK;
  }
  uint32_t boot_id = 0;
  if (!matching_boot_baseline(req, body, boot_id)) return ESP_OK;
  if (!self->try_begin_profile_change()) {
    return reply_error(req, "409 Conflict",
                       "A profile change is already in progress; retry after reboot");
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
  Command command;
  if (!begin_command(req, NEEDS_CAPTURE | NEEDS_BODY, command)) return command.error_result;
  std::string label;
  if (!json_extract_string(command.body, "label", label)) {
    return reply_error(req, "400 Bad Request", "label must be a string");
  }
  const auto saved = command.capture->set_next_capture_label(label);
  if (!saved.accepted) {
    return reply_error(req, "400 Bad Request",
                       "label needs letters, digits, spaces, hyphens, underscores or apostrophes");
  }
  ESP_LOGI(TAG, "Wizard: next capture label '%s'", saved.stored_label.c_str());
  return send_json(req, "{\"ok\":true,\"label\":\"" + json_escape(saved.stored_label) + "\"}");
}

static esp_err_t handle_post_forget_bonds(httpd_req_t* req) {
  Command command;
  if (!begin_command(req, NEEDS_CAPTURE, command)) return command.error_result;
  auto* capture = command.capture;
  ESP_LOGI(TAG, "Wizard: forget all bonds requested");
  const auto result = capture->request_bond_clear();
  std::string json;
  append_bond_clear_json(json, result);
  json.pop_back();
  json += ",\"boot_id\":";
  json += std::to_string(capture->get_capture_result().boot_id);
  json += '}';
  if (result.state == irk_capture::BondClearState::PENDING) {
    httpd_resp_set_status(req, "202 Accepted");
  }
  return send_json(req, json);
}

static esp_err_t handle_post_stop_after_capture(httpd_req_t* req) {
  Command command;
  if (!begin_command(req, NEEDS_CAPTURE | NEEDS_BODY, command)) return command.error_result;
  bool enabled = false;
  if (!json_extract_bool(command.body, "enabled", enabled)) {
    return reply_error(req, "400 Bad Request", "enabled must be a boolean");
  }
  ESP_LOGI(TAG, "Wizard: stop advertising after capture %s", enabled ? "on" : "off");
  command.capture->set_stop_after_capture(enabled);
  return send_ok(req);
}

static esp_err_t handle_post_fresh_identity(httpd_req_t* req) {
  Command command;
  if (!begin_command(req, NEEDS_CAPTURE, command)) return command.error_result;
  static_cast<IRKWizardComponent*>(req->user_ctx)->fresh_identity();
  return send_ok(req);
}

static esp_err_t handle_post_reboot(httpd_req_t* req) {
  // A reboot is how a failed component recovers, so it needs no usable one.
  Command command;
  if (!begin_command(req, NEEDS_BODY, command)) return command.error_result;
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  uint32_t boot_id = 0;
  if (!matching_boot_baseline(req, command.body, boot_id)) return ESP_OK;
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
