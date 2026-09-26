"""Button platform for IRK Capture component."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_ID

from . import CONF_IRK_CAPTURE_ID, IRKCaptureComponent, irk_capture_ns

CONF_REFRESH_IDENTITY = "refresh_identity"
# What refresh_identity was called while the press only rotated the address.
CONF_NEW_MAC = "new_mac"
CONF_FORGET_BONDS = "forget_bonds"

IRKCaptureButton = irk_capture_ns.class_(
    "IRKCaptureButton", button.Button, cg.Component
)
IRKCaptureForgetBondsButton = irk_capture_ns.class_(
    "IRKCaptureForgetBondsButton", button.Button, cg.Component
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_IRK_CAPTURE_ID): cv.use_id(IRKCaptureComponent),
            # Rotates the advertised address and, in a profile whose name is
            # not fixed, adopts a name carrying the new address's low two
            # octets.
            cv.Optional(CONF_REFRESH_IDENTITY): button.button_schema(IRKCaptureButton),
            cv.Optional(CONF_NEW_MAC): button.button_schema(IRKCaptureButton),
            # Clears capture history and requests a bond-store wipe without
            # rotating the advertised address.
            cv.Optional(CONF_FORGET_BONDS): button.button_schema(
                IRKCaptureForgetBondsButton
            ),
        }
    ),
    # Both spellings drive one setter, so accepting both at once would build a
    # second button that nothing ever calls.
    cv.has_at_most_one_key(CONF_REFRESH_IDENTITY, CONF_NEW_MAC),
)


async def to_code(config):
    """Generate code for buttons."""
    parent = await cg.get_variable(config[CONF_IRK_CAPTURE_ID])

    # new_mac is the previous spelling, still accepted so an existing
    # configuration keeps compiling across the rename.
    for key in (CONF_REFRESH_IDENTITY, CONF_NEW_MAC):
        if key in config:
            btn = await button.new_button(config[key])
            cg.add(parent.set_refresh_identity_button(btn))
            break

    if CONF_FORGET_BONDS in config:
        btn = await button.new_button(config[CONF_FORGET_BONDS])
        cg.add(parent.set_forget_bonds_button(btn))
