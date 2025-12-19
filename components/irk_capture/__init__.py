import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import esp32

DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["text_sensor", "switch", "button", "text"]
CODEOWNERS = ["@esphome"]

CONF_BLE_NAME = "ble_name"
CONF_START_ON_BOOT = "start_on_boot"
CONF_CONTINUOUS_MODE = "continuous_mode"
CONF_MAX_CAPTURES = "max_captures"

irk_capture_ns = cg.esphome_ns.namespace("irk_capture")
IRKCaptureComponent = irk_capture_ns.class_("IRKCaptureComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(IRKCaptureComponent),
    cv.Optional(CONF_BLE_NAME, default="Beats Solo4"): cv.string,
    cv.Optional(CONF_START_ON_BOOT, default=True): cv.boolean,
    cv.Optional(CONF_CONTINUOUS_MODE, default=False): cv.boolean,
    cv.Optional(CONF_MAX_CAPTURES, default=1): cv.int_range(min=0, max=255),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_ble_name(config[CONF_BLE_NAME]))
    cg.add(var.set_start_on_boot(config[CONF_START_ON_BOOT]))
    cg.add(var.set_continuous_mode(config[CONF_CONTINUOUS_MODE]))
    cg.add(var.set_max_captures(config[CONF_MAX_CAPTURES]))

    # Enable NimBLE in ESP-IDF
    esp32.add_idf_sdkconfig_option("CONFIG_BT_ENABLED", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_BLUEDROID_ENABLED", False)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ENABLED", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_CENTRAL", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_PERIPHERAL", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_BROADCASTER", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_OBSERVER", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_SECURITY_ENABLE", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_SM_LEGACY", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_SM_SC", True)