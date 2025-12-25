"""Text input platform for IRK Capture component."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text
from esphome.const import CONF_ID

from . import CONF_BLE_NAME, CONF_IRK_CAPTURE_ID, IRKCaptureComponent, irk_capture_ns

IRKCaptureText = irk_capture_ns.class_("IRKCaptureText", text.Text, cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_IRK_CAPTURE_ID): cv.use_id(IRKCaptureComponent),
        cv.Optional(CONF_BLE_NAME): text.text_schema(IRKCaptureText),
    }
)


async def to_code(config):
    """Generate code for text inputs."""
    parent = await cg.get_variable(config[CONF_IRK_CAPTURE_ID])

    if CONF_BLE_NAME in config:
        txt = await text.new_text(config[CONF_BLE_NAME])
        cg.add(parent.set_ble_name_text(txt))
