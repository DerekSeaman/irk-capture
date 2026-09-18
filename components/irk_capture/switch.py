"""Switch platform for IRK Capture component."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID

from . import CONF_IRK_CAPTURE_ID, IRKCaptureComponent, irk_capture_ns

CONF_ADVERTISING = "advertising"
CONF_STOP_AFTER_CAPTURE = "stop_after_capture"
CONF_AUTO_PROFILE_FALLBACK = "auto_profile_fallback"

IRKCaptureSwitch = irk_capture_ns.class_(
    "IRKCaptureSwitch", switch.Switch, cg.Component
)
IRKCaptureStopAfterCaptureSwitch = irk_capture_ns.class_(
    "IRKCaptureStopAfterCaptureSwitch", switch.Switch, cg.Component
)
IRKCaptureAutoProfileFallbackSwitch = irk_capture_ns.class_(
    "IRKCaptureAutoProfileFallbackSwitch", switch.Switch, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_IRK_CAPTURE_ID): cv.use_id(IRKCaptureComponent),
        # DISABLED delegates startup to the parent's start_on_boot setting.
        # Explicit restore modes override that setting before NimBLE starts.
        cv.Optional(CONF_ADVERTISING): switch.switch_schema(
            IRKCaptureSwitch, default_restore_mode="DISABLED"
        ),
        # Both default OFF so existing continuous multi-capture behavior is
        # unchanged unless a user opts in.
        cv.Optional(CONF_STOP_AFTER_CAPTURE): switch.switch_schema(
            IRKCaptureStopAfterCaptureSwitch,
            default_restore_mode="RESTORE_DEFAULT_OFF",
            entity_category="config",
        ),
        cv.Optional(CONF_AUTO_PROFILE_FALLBACK): switch.switch_schema(
            IRKCaptureAutoProfileFallbackSwitch,
            default_restore_mode="RESTORE_DEFAULT_OFF",
            entity_category="config",
        ),
    }
)


async def to_code(config):
    """Generate code for switches."""
    parent = await cg.get_variable(config[CONF_IRK_CAPTURE_ID])

    if CONF_ADVERTISING in config:
        sw = await switch.new_switch(config[CONF_ADVERTISING])
        cg.add(parent.set_advertising_switch(sw))

    if CONF_STOP_AFTER_CAPTURE in config:
        sw = await switch.new_switch(config[CONF_STOP_AFTER_CAPTURE])
        cg.add(parent.set_stop_after_capture_switch(sw))

    if CONF_AUTO_PROFILE_FALLBACK in config:
        sw = await switch.new_switch(config[CONF_AUTO_PROFILE_FALLBACK])
        cg.add(parent.set_auto_profile_fallback_switch(sw))
