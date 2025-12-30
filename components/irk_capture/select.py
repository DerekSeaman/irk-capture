"""Select platform for IRK Capture component - BLE Profile selection."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select

from . import CONF_IRK_CAPTURE_ID, IRKCaptureComponent, irk_capture_ns

CONF_BLE_PROFILE = "ble_profile"

IRKCaptureSelect = irk_capture_ns.class_(
    "IRKCaptureSelect", select.Select, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_IRK_CAPTURE_ID): cv.use_id(IRKCaptureComponent),
        cv.Optional(CONF_BLE_PROFILE): select.select_schema(IRKCaptureSelect),
    }
)


async def to_code(config):
    """Generate code for select components."""
    parent = await cg.get_variable(config[CONF_IRK_CAPTURE_ID])

    if CONF_BLE_PROFILE in config:
        sel = await select.new_select(
            config[CONF_BLE_PROFILE],
            options=["Heart Sensor", "Keyboard"],
        )
        cg.add(parent.set_ble_profile_select(sel))
