"""On-device capture wizard: a small esp_http_server serving a guided UI.

Runs on its own port (default 8080), separate from ESPHome's own
`web_server:` (usually port 80), so it can be added alongside the stock
entity UI without conflicting with it. Talks to an `irk_capture` instance
and a handful of its text sensors directly - it does not duplicate any
capture/BLE logic, only reads published entity state and calls the same
public control methods the switch/button/text/select platforms use.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32, text_sensor
from esphome.components.irk_capture import CONF_IRK_CAPTURE_ID, IRKCaptureComponent
from esphome.const import CONF_ID

CODEOWNERS = ["@davidcoulson"]
# network: the server start is gated on network::is_connected(), so the
# network component must be pulled into the build even for a config that
# does not otherwise declare wifi/ethernet.
DEPENDENCIES = ["irk_capture", "network", "text_sensor"]

CONF_PORT = "port"
CONF_USERNAME = "username"
CONF_PASSWORD = "password"
CONF_STATUS_ID = "status_id"
CONF_IRK_ID = "irk_id"
CONF_DEVICE_MAC_ID = "device_mac_id"
CONF_EFFECTIVE_MAC_ID = "effective_mac_id"

irk_wizard_ns = cg.esphome_ns.namespace("irk_wizard")
IRKWizardComponent = irk_wizard_ns.class_("IRKWizardComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(IRKWizardComponent),
        cv.Required(CONF_IRK_CAPTURE_ID): cv.use_id(IRKCaptureComponent),
        # The four text sensors the wizard displays. These must already be
        # defined elsewhere in the device YAML (see irk-capture-base.yaml) -
        # this component only reads their published state, it doesn't create
        # them, so the same values stay visible in Home Assistant too.
        cv.Required(CONF_STATUS_ID): cv.use_id(text_sensor.TextSensor),
        cv.Required(CONF_IRK_ID): cv.use_id(text_sensor.TextSensor),
        cv.Required(CONF_DEVICE_MAC_ID): cv.use_id(text_sensor.TextSensor),
        cv.Required(CONF_EFFECTIVE_MAC_ID): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_PORT, default=8080): cv.int_range(min=1, max=65535),
        # Optional HTTP Basic auth. Strongly recommended: without it anyone
        # on the LAN can read captured IRKs (which permanently deanonymize a
        # phone's rotating BLE address) and drive advertising/bond wiping.
        cv.Inclusive(CONF_USERNAME, "auth"): cv.string_strict,
        cv.Inclusive(CONF_PASSWORD, "auth"): cv.string_strict,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    irk = await cg.get_variable(config[CONF_IRK_CAPTURE_ID])
    cg.add(var.set_irk_capture(irk))

    status = await cg.get_variable(config[CONF_STATUS_ID])
    cg.add(var.set_status_sensor(status))
    irk_sensor = await cg.get_variable(config[CONF_IRK_ID])
    cg.add(var.set_irk_sensor(irk_sensor))
    device_mac = await cg.get_variable(config[CONF_DEVICE_MAC_ID])
    cg.add(var.set_device_mac_sensor(device_mac))
    effective_mac = await cg.get_variable(config[CONF_EFFECTIVE_MAC_ID])
    cg.add(var.set_effective_mac_sensor(effective_mac))

    cg.add(var.set_port(config[CONF_PORT]))
    if CONF_USERNAME in config:
        cg.add(var.set_auth(config[CONF_USERNAME], config[CONF_PASSWORD]))

    # ESPHome 2026.9+ excludes most built-in ESP-IDF components from the
    # build unless something asks for them (irk_capture does this for "bt";
    # esp_http_server needs the same treatment and can't assume web_server:
    # is configured to pull it in).
    if hasattr(esp32, "include_builtin_idf_component"):
        esp32.include_builtin_idf_component("esp_http_server")

    # A second esp_http_server instance (this one) alongside ESPHome's own
    # web_server: on port 80 - plus API, OTA and mDNS - exhausts the default
    # LWIP socket ceiling before our server even opens its control socket
    # (fails as "error in creating ctrl socket (112)" == EHOSTDOWN, LWIP's
    # generic failure when a UDP connect() can't get a socket/route at all).
    # web_server:'s own httpd alone can claim close to its max_open_sockets
    # (default 7) plus control/listen sockets; 16 wasn't enough headroom on
    # top of that plus API/OTA/mDNS/WiFi's own sockets. irk_wizard.cpp also
    # trims its own max_open_sockets to keep its ask small.
    esp32.add_idf_sdkconfig_option("CONFIG_LWIP_MAX_SOCKETS", 32)
