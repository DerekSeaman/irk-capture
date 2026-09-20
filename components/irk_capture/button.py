"""Button platform for IRK Capture component."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_ID

from . import CONF_IRK_CAPTURE_ID, IRKCaptureComponent, irk_capture_ns

CONF_NEW_MAC = "new_mac"
CONF_FORGET_BONDS = "forget_bonds"

IRKCaptureButton = irk_capture_ns.class_(
    "IRKCaptureButton", button.Button, cg.Component
)
IRKCaptureForgetBondsButton = irk_capture_ns.class_(
    "IRKCaptureForgetBondsButton", button.Button, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_IRK_CAPTURE_ID): cv.use_id(IRKCaptureComponent),
        cv.Optional(CONF_NEW_MAC): button.button_schema(IRKCaptureButton),
        # Clears capture history and requests a bond-store wipe without
        # rotating the advertised address.
        cv.Optional(CONF_FORGET_BONDS): button.button_schema(
            IRKCaptureForgetBondsButton
        ),
    }
)


async def to_code(config):
    """Generate code for buttons."""
    parent = await cg.get_variable(config[CONF_IRK_CAPTURE_ID])

    if CONF_NEW_MAC in config:
        btn = await button.new_button(config[CONF_NEW_MAC])
        cg.add(parent.set_new_mac_button(btn))

    if CONF_FORGET_BONDS in config:
        btn = await button.new_button(config[CONF_FORGET_BONDS])
        cg.add(parent.set_forget_bonds_button(btn))
