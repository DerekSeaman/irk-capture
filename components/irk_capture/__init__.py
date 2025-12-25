import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import esp32

DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["text_sensor", "switch", "button", "text"]
CODEOWNERS = ["@esphome"]

CONF_IRK_CAPTURE_ID = "irk_capture_id"
CONF_BLE_NAME = "ble_name"
CONF_START_ON_BOOT = "start_on_boot"
CONF_CONTINUOUS_MODE = "continuous_mode"
CONF_MAX_CAPTURES = "max_captures"

irk_capture_ns = cg.esphome_ns.namespace("irk_capture")
IRKCaptureComponent = irk_capture_ns.class_("IRKCaptureComponent", cg.Component)


def validate_ble_name(value):
    """Validate BLE name length (BLE spec: max 29 bytes for advertising packet)"""
    value = cv.string(value)
    if len(value.encode("utf-8")) > 29:
        raise cv.Invalid(
            f"BLE name too long ({len(value.encode('utf-8'))} bytes). "
            f"Maximum 29 bytes for BLE advertising packet. Shorten your name."
        )
    return value


def validate_continuous_mode_config(config):
    """Validate continuous_mode and max_captures interaction"""
    continuous_mode = config.get(CONF_CONTINUOUS_MODE, False)
    max_captures = config.get(CONF_MAX_CAPTURES, 1)

    # Configuration conflict: continuous_mode=false with max_captures>1
    if not continuous_mode and max_captures > 1:
        raise cv.Invalid(
            f"Configuration conflict: continuous_mode=false with max_captures={max_captures}. "
            f"To capture multiple devices, set continuous_mode=true. "
            f"For single capture, set max_captures=1."
        )

    # Warning for unlimited mode
    if max_captures == 0:
        import logging

        logging.getLogger(__name__).warning(
            "max_captures=0 enables unlimited capture mode (no auto-stop). "
            "This is not recommended for production. Set a specific limit (e.g., max_captures=5)."
        )

    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(IRKCaptureComponent),
            cv.Optional(CONF_BLE_NAME, default="IRK Capture"): validate_ble_name,
            cv.Optional(CONF_START_ON_BOOT, default=True): cv.boolean,
            cv.Optional(CONF_CONTINUOUS_MODE, default=False): cv.boolean,
            cv.Optional(CONF_MAX_CAPTURES, default=1): cv.int_range(min=0, max=255),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_continuous_mode_config,  # Cross-field validation
)


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

    # Increase NimBLE host stack size for stability with ESP_LOG calls
    # Default: 4096 bytes. Increased to 5120 for safety margin with debug logging.
    # Stack overflow risk: ESP_LOG calls in gap_event callbacks are stack-heavy
    esp32.add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_TASK_STACK_SIZE", 5120)
