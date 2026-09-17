"""Switch platform for IRK Capture component."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID

from . import CONF_IRK_CAPTURE_ID, IRKCaptureComponent, irk_capture_ns

CONF_ADVERTISING = "advertising"

IRKCaptureSwitch = irk_capture_ns.class_(
    "IRKCaptureSwitch", switch.Switch, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_IRK_CAPTURE_ID): cv.use_id(IRKCaptureComponent),
        # DISABLED delegates startup to the parent's start_on_boot setting.
        # Explicit restore modes override that setting before NimBLE starts.
        cv.Optional(CONF_ADVERTISING): switch.switch_schema(
            IRKCaptureSwitch, default_restore_mode="DISABLED"
        ),
    }
)


async def to_code(config):
    """Generate code for switches."""
    parent = await cg.get_variable(config[CONF_IRK_CAPTURE_ID])

    if CONF_ADVERTISING in config:
        sw = await switch.new_switch(config[CONF_ADVERTISING])
        cg.add(parent.set_advertising_switch(sw))
