"""On-device capture wizard: a small esp_http_server serving a guided UI.

Runs on its own port (default 8080), separate from ESPHome's `web_server:`
(port 80), so it can coexist with the stock entity UI. Talks directly to an
`irk_capture` instance and four of its text sensors, delegating all BLE and
capture logic to `irk_capture`.
"""

import gzip
from pathlib import Path

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32, text_sensor
from esphome.components.irk_capture import (
    CONF_IRK_CAPTURE_ID,
    IRKCaptureComponent,
    header_constant,
)
from esphome.const import CONF_ID
from esphome.core import EsphomeError

CODEOWNERS = ["@davidcoulson"]
# network: server startup checks network::is_connected() (provided by wifi/ethernet).
DEPENDENCIES = ["irk_capture", "network", "text_sensor"]

CONF_PAGE_DATA_ID = "page_data_id"
CONF_PORT = "port"
CONF_USERNAME = "username"
CONF_PASSWORD = "password"
CONF_STATUS_ID = "status_id"
CONF_IRK_ID = "irk_id"
CONF_DEVICE_MAC_ID = "device_mac_id"
CONF_EFFECTIVE_MAC_ID = "effective_mac_id"


def validate_credential(value):
    """Accept only non-empty printable ASCII credentials for HTTP Basic auth.

    Browsers disagree on non-ASCII Basic auth encoding (UTF-8 vs Latin-1), so
    restricting to printable ASCII keeps logins consistent across browsers.
    """
    value = cv.string_strict(value)
    if not value:
        raise cv.Invalid("Must not be empty.")
    if any(ord(char) < 0x20 or ord(char) > 0x7E for char in value):
        raise cv.Invalid(
            "Use printable ASCII characters only. Browsers encode other "
            "characters differently for HTTP Basic authentication, so the "
            "login would never match."
        )
    return value


def validate_auth_config(config):
    """Match the runtime Authorization header budget before compiling firmware."""
    username = config[CONF_USERNAME]
    password = config[CONF_PASSWORD]
    if ":" in username:
        raise cv.Invalid("Wizard username cannot contain ':' (HTTP Basic separator).")
    encoded_length = 6 + 4 * ((len(username) + 1 + len(password) + 2) // 3)
    if encoded_length > 256:
        raise cv.Invalid(
            "Wizard credentials produce an Authorization header longer than 256 bytes. "
            "Username and password together must be at most 185 ASCII characters."
        )
    return config


irk_wizard_ns = cg.esphome_ns.namespace("irk_wizard")
IRKWizardComponent = irk_wizard_ns.class_("IRKWizardComponent", cg.Component)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(IRKWizardComponent),
            cv.GenerateID(CONF_PAGE_DATA_ID): cv.declare_id(cg.uint8),
            cv.Required(CONF_IRK_CAPTURE_ID): cv.use_id(IRKCaptureComponent),
            # Existing irk_capture text sensors (see irk-capture-base.yaml); the wizard
            # reads their published state rather than creating duplicate entities.
            cv.Required(CONF_STATUS_ID): cv.use_id(text_sensor.TextSensor),
            cv.Required(CONF_IRK_ID): cv.use_id(text_sensor.TextSensor),
            cv.Required(CONF_DEVICE_MAC_ID): cv.use_id(text_sensor.TextSensor),
            cv.Required(CONF_EFFECTIVE_MAC_ID): cv.use_id(text_sensor.TextSensor),
            # The server's internal control socket uses 32768 + port, which must
            # also fit in 16 bits.
            cv.Optional(CONF_PORT, default=8080): cv.int_range(min=1, max=32767),
            # HTTP Basic auth, required. A captured IRK permanently deanonymizes
            # a phone's rotating BLE address, and an open device on the LAN is
            # reachable by anything else on it, including a page in a browser.
            cv.Required(CONF_USERNAME): validate_credential,
            cv.Required(CONF_PASSWORD): validate_credential,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_auth_config,
)


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
    # Gzip the page at build time to about 30% of its original HTML size.
    # mtime=0 keeps the output byte-identical between builds.
    page_path = Path(__file__).parent / "wizard_page.html"
    try:
        page = page_path.read_bytes()
    except OSError as err:
        raise EsphomeError(
            f"irk_wizard cannot read its page {page_path}: {err}"
        ) from err
    # The label inputs take their length cap from the firmware, which trims a
    # label to CAPTURE_LABEL_MAX_LEN, so the page never accepts a longer one.
    label_max = header_constant("irk_capture.h", "CAPTURE_LABEL_MAX_LEN")
    placeholder = b"@CAPTURE_LABEL_MAX_LEN@"
    if page.count(placeholder) != 2:
        raise EsphomeError(
            f"irk_wizard: {page_path} must use {placeholder.decode()} for both label inputs"
        )
    page = page.replace(placeholder, str(label_max).encode())
    packed = gzip.compress(page, compresslevel=9, mtime=0)
    page_arr = cg.progmem_array(config[CONF_PAGE_DATA_ID], list(packed))
    cg.add(var.set_page(page_arr, len(packed)))

    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_auth(config[CONF_USERNAME], config[CONF_PASSWORD]))

    # ESPHome 2026.9+ excludes built-in ESP-IDF components unless requested
    # (cannot assume web_server: is configured to pull in esp_http_server).
    if hasattr(esp32, "include_builtin_idf_component"):
        esp32.include_builtin_idf_component("esp_http_server")

    # Running a second esp_http_server alongside ESPHome's web_server: (default
    # max_open_sockets=7 plus listen/ctrl sockets), API, OTA, and mDNS can
    # exhaust lwIP's default socket limit (16 was not enough headroom).
    # irk_wizard.cpp also trims its own max_open_sockets to 4.
    esp32.add_idf_sdkconfig_option("CONFIG_LWIP_MAX_SOCKETS", 32)
