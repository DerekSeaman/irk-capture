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

  size_t len = httpd_req_get_hdr_value_len(req, "Authorization");
  if (len == 0 || len > 256) {
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
    ESP_LOGW(TAG, "Rejected request with bad credentials");
    send_unauthorized(req);
    return false;
  }
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
  next.ble_name = irk_capture_->get_ble_name();
  next.next_capture_label = irk_capture_->get_next_capture_label();
  const bool keyboard = irk_capture_->get_ble_profile() == irk_capture::BLEProfile::KEYBOARD;
  next.profile = keyboard ? "Keyboard" : "Heart Sensor";
  // The Keyboard profile advertises a fixed Logitech name, so this is what
  // the user must actually look for on the phone - telling them to look for
  // the configured BLE name there would send them hunting for the wrong entry.
  next.advertised_name = keyboard ? "Logitech K380" : next.ble_name;
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
static esp_err_t handle_post_new_mac(httpd_req_t* req);
static esp_err_t handle_post_stop_after_capture(httpd_req_t* req);

static esp_err_t send_json(httpd_req_t* req, const std::string& body) {
  httpd_resp_set_type(req, "application/json");
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
    { "/api/new_mac", HTTP_POST, handle_post_new_mac },
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

//======================== Embedded UI ========================
// Self-contained (no external assets) so it works with no internet access,
// including from the fallback AP. Polls /api/status; POSTs single-field
// JSON bodies the handlers above parse with json_extract_*().

static const char INDEX_HTML[] = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>IRK Capture Wizard</title>
<style>
:root{--bg:#10181a;--surf:#172124;--surf2:#1c2729;--border:#2a3739;--ink1:#eef3f2;--ink2:#aebcbc;--ink3:#7f9294;
--accent:#3fc3c2;--accentInk:#06282a;--amber:#e3a94a;--green:#6fc98d;--red:#e18a82}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink1);font:15px/1.5 -apple-system,system-ui,sans-serif;padding:20px 16px 60px}
.wrap{max-width:540px;margin:0 auto}
h1{font-size:20px;margin:0 0 4px}
.sub{color:var(--ink3);font-size:13px;margin:0 0 18px}
.card{background:var(--surf);border:1px solid var(--border);border-radius:12px;padding:16px;margin-bottom:14px}
.row{display:flex;justify-content:space-between;align-items:center;gap:10px;margin-bottom:10px}
.row:last-child{margin-bottom:0}
.k{color:var(--ink3);font-size:12px;text-transform:uppercase;letter-spacing:.05em}
.v{font-family:ui-monospace,monospace;font-size:13px;word-break:break-all;text-align:right}
.pill{display:inline-flex;align-items:center;gap:6px;font-size:12px;font-weight:600;padding:5px 10px;border-radius:99px}
.pill.idle{background:#2a3739;color:var(--ink2)}
.pill.advertising{background:#28323a;color:var(--accent)}
.pill.pairing,.pill.capturing{background:#3a2f1b;color:var(--amber)}
.pill.captured{background:#1c3626;color:var(--green)}
.pill.no_irk{background:#3a2320;color:var(--red)}
.pill::before{content:"";width:6px;height:6px;border-radius:50%;background:currentColor}
select,input[type=text]{width:100%;background:var(--surf2);border:1px solid var(--border);color:var(--ink1);
border-radius:8px;padding:9px 10px;font-size:14px;font-family:inherit}
.btnrow{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}
button{font:inherit;font-weight:600;font-size:13px;border:1px solid var(--border);background:var(--surf2);
color:var(--ink1);border-radius:8px;padding:10px 14px;cursor:pointer}
button.primary{background:var(--accent);color:var(--accentInk);border-color:var(--accent)}
button.danger{color:var(--red);border-color:#4a2a26}
button:disabled{opacity:.45;cursor:default}
button.big{display:block;width:100%;padding:14px;font-size:15px;margin-bottom:18px}
.toggle{display:flex;justify-content:space-between;align-items:center}
.switch{width:38px;height:22px;border-radius:99px;background:var(--border);position:relative;cursor:pointer;flex:none}
.switch.on{background:var(--accent)}
.switch::after{content:"";position:absolute;width:18px;height:18px;border-radius:50%;background:#fff;top:2px;left:2px;transition:.15s}
.switch.on::after{left:18px}
.hist{display:flex;flex-direction:column;gap:6px}
.hist .item{display:flex;justify-content:space-between;background:var(--surf2);border:1px solid var(--border);
border-radius:8px;padding:8px 10px;font-size:12.5px}
.hist .mac{font-family:ui-monospace,monospace;color:var(--ink3)}
.empty{color:var(--ink3);font-size:13px;text-align:center;padding:10px 0}
label{display:block;font-size:12px;color:var(--ink3);text-transform:uppercase;letter-spacing:.05em;margin-bottom:6px}
.steps{display:flex;gap:6px;margin-bottom:16px}
.steps .s{flex:1;height:4px;border-radius:2px;background:var(--border)}
.steps .s.done{background:var(--accent)}
.steps .s.now{background:var(--accent);opacity:.55}
.stepno{font-size:12px;color:var(--accent);text-transform:uppercase;letter-spacing:.08em;font-weight:600;margin-bottom:6px}
h2{font-size:17px;margin:0 0 8px}
p.help{color:var(--ink2);font-size:13.5px;margin:0 0 12px}
.callout{background:var(--surf2);border-left:3px solid var(--amber);border-radius:6px;padding:10px 12px;font-size:13px;color:var(--ink2);margin:12px 0}
.callout b{color:var(--ink1)}
.target{background:var(--surf2);border:1px solid var(--accent);border-radius:8px;padding:12px;text-align:center;margin:12px 0}
.target .nm{font-family:ui-monospace,monospace;font-size:17px;color:var(--accent);font-weight:600;word-break:break-all}
.target .cap{font-size:11.5px;color:var(--ink3);text-transform:uppercase;letter-spacing:.06em;margin-bottom:4px}
.irkbox{background:var(--surf2);border:1px solid var(--border);border-radius:8px;padding:12px;margin:12px 0;
font-family:ui-monospace,monospace;font-size:14px;word-break:break-all;-webkit-user-select:all;user-select:all}
.ok{color:var(--green)}
.hide{display:none}
ol.pair{margin:0 0 4px;padding-left:20px;color:var(--ink2);font-size:13.5px}
ol.pair li{margin-bottom:5px}
.banner{background:#3a2f1b;color:var(--amber);border-radius:8px;padding:10px 12px;font-size:13px;font-weight:500;margin-bottom:14px}
</style></head><body><div class="wrap">

<div id="offline" class="banner hide">Device is rebooting or unreachable - reconnecting...</div>

<!-- ============ WIZARD VIEW ============ -->
<div id="wizView" class="hide">
  <h1>IRK Wizard</h1>
  <p class="sub" id="wizSub">Guided capture, one step at a time</p>
  <div class="steps">
    <div class="s" data-step="1"></div><div class="s" data-step="2"></div>
    <div class="s" data-step="3"></div><div class="s" data-step="4"></div><div class="s" data-step="5"></div>
  </div>

  <div class="card" id="step1">
    <div class="stepno">Step 1 of 5</div>
    <h2>What are you capturing from?</h2>
    <p class="help">This picks how the device disguises itself so your phone will show it.</p>
    <select id="wizProfile">
      <option value="Heart Sensor">iPhone or Apple Watch</option>
      <option value="Keyboard">Samsung Galaxy / Android phone</option>
    </select>
    <div class="callout" id="profileWarn">Changing this <b>reboots the device</b> and takes about 15 seconds. The page reconnects on its own.</div>
    <div class="btnrow"><button class="primary" onclick="wizGo(2)">Next</button>
      <button onclick="showPanel()">Exit wizard</button></div>
  </div>

  <div class="card hide" id="step2">
    <div class="stepno">Step 2 of 5</div>
    <h2>Whose device is this?</h2>
    <p class="help">Optional. The name is stored with the captured key so you can tell them apart later.</p>
    <input type="text" id="wizLabel" placeholder="e.g. Daves iPhone" maxlength="24">
    <div class="btnrow"><button class="primary" onclick="wizSaveLabel()">Next</button>
      <button onclick="wizGo(1)">Back</button></div>
  </div>

  <div class="card hide" id="step3">
    <div class="stepno">Step 3 of 5</div>
    <h2>Clear any old pairing</h2>
    <p class="help">If this phone has paired with this device before, it will silently reuse the old
      pairing and no new key is captured. This is the most common reason a capture never happens.</p>
    <div class="callout">On your phone: open <b>Bluetooth settings</b>, find
      <b id="forgetName">IRK Capture</b>, and choose <b>Forget This Device</b> (iPhone: tap the
      &#9432; first). If it is not listed, nothing to do.</div>
    <div class="btnrow"><button class="danger" onclick="wizForget()">Also clear it on this device</button></div>
    <p class="help ok hide" id="forgetDone">Cleared on this device.</p>
    <div class="btnrow"><button class="primary" onclick="wizStart()">Next: start pairing</button>
      <button onclick="wizGo(2)">Back</button></div>
  </div>

  <div class="card hide" id="step4">
    <div class="stepno">Step 4 of 5</div>
    <h2>Pair from your phone now</h2>
    <div class="row"><span class="k">Device status</span><span id="wizPill" class="pill idle">idle</span></div>
    <div class="target"><div class="cap">Look for this name</div><div class="nm" id="wizAdvName">-</div></div>
    <ol class="pair" id="wizSteps"></ol>
    <div class="callout" id="wizHint">Waiting for your phone to connect. Keep this page open.</div>
    <div class="btnrow"><button onclick="wizStop()">Stop</button><button onclick="wizGo(3)">Back</button></div>
  </div>

  <div class="card hide" id="step5">
    <div class="stepno">Step 5 of 5</div>
    <h2 class="ok">Key captured</h2>
    <p class="help">Copy this IRK into Home Assistant: <b>Settings &rarr; Devices &amp; Services &rarr;
      Add Integration &rarr; Private BLE Device</b>.</p>
    <div class="irkbox" id="wizIrk">-</div>
    <div class="btnrow"><button class="primary" onclick="copyIrk()">Copy IRK</button>
      <span id="copyMsg" class="help" style="margin:10px 0 0"></span></div>
    <div class="callout">You can now <b>forget this device</b> in your phone's Bluetooth settings -
      the pairing was only needed to hand over the key.</div>
    <div class="btnrow"><button onclick="wizRestart()">Capture another device</button>
      <button onclick="showPanel()">Done</button></div>
  </div>
</div>

<!-- ============ PANEL VIEW ============ -->
<div id="panelView">
  <h1>IRK Capture</h1>
  <p class="sub" id="deviceName">-</p>

  <button class="big primary" onclick="showWizard()">Start IRK Wizard &rarr;</button>

  <div class="card">
    <div class="row"><span class="k">Status</span><span id="statusPill" class="pill idle">idle</span></div>
    <div class="row"><span class="k">Advertising as</span><span class="v" id="advName">-</span></div>
    <div class="row"><span class="k">Effective MAC</span><span class="v" id="effMac">-</span></div>
    <div class="row"><span class="k">Captured IRK</span><span class="v" id="irkVal">-</span></div>
  </div>

  <div class="card">
    <label>Target device</label>
    <select id="profile">
      <option value="Heart Sensor">iPhone / Apple Watch (Heart Sensor)</option>
      <option value="Keyboard">Samsung Galaxy (Keyboard)</option>
    </select>
    <div style="height:10px"></div>
    <label>Label this capture (optional)</label>
    <input type="text" id="label" placeholder="e.g. Daves iPhone" maxlength="24">
    <div class="btnrow">
      <button class="primary" id="advBtn">Start Advertising</button>
      <button id="newMacBtn">Generate New MAC</button>
      <button class="danger" id="forgetBtn">Forget All Bonds</button>
    </div>
  </div>

  <div class="card">
    <div class="row toggle"><span class="k">Stop advertising after capture</span>
      <div class="switch" id="stopAfterSwitch"></div></div>
  </div>

  <div class="card">
    <div class="k" style="margin-bottom:10px">Captured this session</div>
    <div class="hist" id="history"><div class="empty">No devices captured yet</div></div>
  </div>
</div>

</div>
<script>
function el(id){return document.getElementById(id)}
var wizStep = 0;          // 0 = panel view
var lastData = null;
var labelDirty = false;
var awaitingCapture = false;
var irkAtStart = '';
var failCount = 0;

async function post(url,body){
  try{
    await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})});
  }catch(e){}
  refresh();
}

/* ---------- view switching ---------- */
function showPanel(){ wizStep=0; el('wizView').classList.add('hide'); el('panelView').classList.remove('hide');
  window.scrollTo(0,0); refresh(); }
function showWizard(){ wizGo(1); el('panelView').classList.add('hide'); el('wizView').classList.remove('hide');
  window.scrollTo(0,0); }
function wizGo(n){
  wizStep=n;
  for(var i=1;i<=5;i++) el('step'+i).classList.toggle('hide', i!==n);
  document.querySelectorAll('.steps .s').forEach(function(s){
    var i=parseInt(s.dataset.step,10);
    s.className='s'+(i<n?' done':(i===n?' now':''));
  });
  window.scrollTo(0,0);
  if(lastData) render(lastData);
}

/* ---------- wizard actions ---------- */
function wizSaveLabel(){ post('/api/label',{label:el('wizLabel').value}); wizGo(3); }
function wizForget(){ post('/api/forget_bonds',{}); el('forgetDone').classList.remove('hide'); }
function wizStart(){
  // Remember the key showing now: entering this step while a previous
  // capture is still inside its "captured" hold would otherwise jump
  // straight to step 5 with that older key.
  irkAtStart = lastData ? (lastData.irk || '') : '';
  awaitingCapture = true;
  post('/api/advertising',{on:true});
  wizGo(4);
}
function wizStop(){ awaitingCapture=false; post('/api/advertising',{on:false}); }
function wizRestart(){ awaitingCapture=false; el('wizLabel').value=''; wizGo(1); }

function copyIrk(){
  var txt = el('wizIrk').textContent;
  function done(ok){ el('copyMsg').textContent = ok ? 'Copied.' : 'Select the key above and copy it.'; }
  // navigator.clipboard is unavailable over plain HTTP, so fall back.
  if(navigator.clipboard && window.isSecureContext){
    navigator.clipboard.writeText(txt).then(function(){done(true)},function(){done(false)});
    return;
  }
  try{
    var ta=document.createElement('textarea'); ta.value=txt; document.body.appendChild(ta);
    ta.select(); var ok=document.execCommand('copy'); document.body.removeChild(ta); done(ok);
  }catch(e){ done(false); }
}

/* ---------- pairing instructions ---------- */
function pairSteps(profile,name){
  if(profile==='Keyboard'){
    return ['Open Settings, then Connections, then Bluetooth.',
            'Tap Scan if the list does not refresh on its own.',
            'Tap "'+name+'" in the list of available devices.',
            'Confirm the pairing request if one appears.'];
  }
  return ['Open Settings, then Bluetooth (make sure Bluetooth is on).',
          'Look under "Other Devices" for "'+name+'".',
          'Tap it, then tap Pair if asked.',
          'For an Apple Watch, pair from a heart-rate app instead.'];
}

/* ---------- panel wiring ---------- */
el('label').addEventListener('input',function(){labelDirty=true});
el('label').addEventListener('blur',function(){post('/api/label',{label:el('label').value});labelDirty=false});
el('profile').addEventListener('change',function(){post('/api/profile',{profile:el('profile').value})});
el('wizProfile').addEventListener('change',function(){post('/api/profile',{profile:el('wizProfile').value})});
el('advBtn').addEventListener('click',function(){
  post('/api/advertising',{on: el('advBtn').dataset.on !== '1'});
});
el('newMacBtn').addEventListener('click',function(){post('/api/new_mac',{})});
el('forgetBtn').addEventListener('click',function(){
  if(confirm('Clear all cached bonds and this session history?')) post('/api/forget_bonds',{});
});
el('stopAfterSwitch').addEventListener('click',function(){
  post('/api/stop_after_capture',{enabled:!el('stopAfterSwitch').classList.contains('on')});
});

/* ---------- rendering ---------- */
var KNOWN=['idle','advertising','pairing','capturing','captured','no_irk'];
function setPill(node,status){
  node.textContent=status;
  node.className='pill '+(KNOWN.indexOf(status)>=0?status:'idle');
}

function render(d){
  var advName = d.advertised_name || d.ble_name || '-';

  /* panel */
  el('deviceName').textContent = advName + ' · ' + d.profile;
  setPill(el('statusPill'), d.status);
  el('advName').textContent = advName;
  el('effMac').textContent = d.effective_mac || '-';
  el('irkVal').textContent = d.irk || '-';
  el('profile').value = d.profile;
  if(!labelDirty) el('label').value = d.next_capture_label || '';
  el('advBtn').textContent = d.advertising ? 'Stop Advertising' : 'Start Advertising';
  el('advBtn').dataset.on = d.advertising ? '1' : '0';
  el('stopAfterSwitch').classList.toggle('on', !!d.stop_after_capture);

  var history=[];
  try{ history=JSON.parse(d.history||'[]'); }catch(e){}
  var histEl=el('history');
  histEl.textContent='';
  if(!history.length){
    var em=document.createElement('div'); em.className='empty';
    em.textContent='No devices captured yet'; histEl.appendChild(em);
  } else {
    // DOM nodes with textContent, never innerHTML: labels and MACs come from
    // paired devices and are untrusted here.
    history.forEach(function(h){
      var it=document.createElement('div'); it.className='item';
      var nm=document.createElement('span'); nm.textContent=h.label||'(unlabeled)';
      var mc=document.createElement('span'); mc.className='mac'; mc.textContent=h.mac||'';
      it.appendChild(nm); it.appendChild(mc); histEl.appendChild(it);
    });
  }

  /* wizard */
  if(wizStep===0) return;
  el('wizProfile').value = d.profile;
  el('forgetName').textContent = advName;
  if(document.activeElement !== el('wizLabel') && !el('wizLabel').value && d.next_capture_label)
    el('wizLabel').value = d.next_capture_label;

  if(wizStep===4){
    setPill(el('wizPill'), d.status);
    el('wizAdvName').textContent = advName;
    var ol=el('wizSteps'); ol.textContent='';
    pairSteps(d.profile, advName).forEach(function(t){
      var li=document.createElement('li'); li.textContent=t; ol.appendChild(li);
    });
    var hint=el('wizHint');
    if(d.status==='no_irk'){
      hint.textContent='Your phone paired but did not hand over an identity key. '+
        'Forget this device in its Bluetooth settings, then go back a step and try again.';
    } else if(d.status==='pairing'||d.status==='capturing'){
      hint.textContent='Your phone is connected - finishing the key exchange. Do not touch anything.';
    } else if(!d.advertising){
      hint.textContent='Advertising is off. Go back a step and press "Next: start pairing".';
    } else {
      hint.textContent='Waiting for your phone to connect. Keep this page open.';
    }
    // The capture itself is what advances the wizard.
    if(awaitingCapture && d.status==='captured' && d.irk && d.irk !== irkAtStart){
      awaitingCapture=false; wizGo(5);
    }
  }

  if(wizStep===5) el('wizIrk').textContent = d.irk || '-';
}

async function refresh(){
  try{
    var r=await fetch('/api/status');
    if(!r.ok) throw new Error('http');
    var d=await r.json();
    failCount=0; el('offline').classList.add('hide');
    lastData=d; render(d);
  }catch(e){
    // A profile change reboots the device; say so instead of going silent.
    if(++failCount>2) el('offline').classList.remove('hide');
  }
}
refresh();
setInterval(refresh, 1200);
</script></body></html>)HTML";

static esp_err_t handle_index(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handle_get_status(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  WizardSnapshot s = self->get_snapshot();

  std::string json = "{";
  json += "\"status\":\"" + json_escape(s.status) + "\",";
  json += "\"ble_name\":\"" + json_escape(s.ble_name) + "\",";
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

static esp_err_t handle_post_new_mac(httpd_req_t* req) {
  if (!authorized(req)) return ESP_OK;
  if (!json_request(req)) return ESP_OK;
  auto* self = static_cast<IRKWizardComponent*>(req->user_ctx);
  if (self->irk_capture()) self->irk_capture()->refresh_mac();
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

}  // namespace irk_wizard
}  // namespace esphome

#endif  // USE_ESP32
