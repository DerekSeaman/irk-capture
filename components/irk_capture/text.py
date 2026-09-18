"""Text input platform for IRK Capture component."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text
from esphome.const import CONF_ID

from . import CONF_BLE_NAME, CONF_IRK_CAPTURE_ID, IRKCaptureComponent, irk_capture_ns

CONF_NEXT_CAPTURE_LABEL = "next_capture_label"

IRKCaptureText = irk_capture_ns.class_("IRKCaptureText", text.Text, cg.Component)
IRKCaptureLabelText = irk_capture_ns.class_(
    "IRKCaptureLabelText", text.Text, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_IRK_CAPTURE_ID): cv.use_id(IRKCaptureComponent),
        cv.Optional(CONF_BLE_NAME): text.text_schema(IRKCaptureText),
        # Label attached to the next new device captured this session; a
        # wizard sets it right before starting advertising.
        cv.Optional(CONF_NEXT_CAPTURE_LABEL): text.text_schema(
            IRKCaptureLabelText, entity_category="config"
        ),
    }
)


async def to_code(config):
    """Generate code for text inputs."""
    parent = await cg.get_variable(config[CONF_IRK_CAPTURE_ID])

    if CONF_BLE_NAME in config:
        txt = await text.new_text(config[CONF_BLE_NAME])
        cg.add(parent.set_ble_name_text(txt))

    if CONF_NEXT_CAPTURE_LABEL in config:
        txt = await text.new_text(config[CONF_NEXT_CAPTURE_LABEL])
        cg.add(parent.set_next_capture_label_text(txt))
