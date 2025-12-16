import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID

from . import irk_capture_ns, IRKCaptureComponent

CONF_IRK_CAPTURE_ID = "irk_capture_id"
CONF_ADVERTISING = "advertising"

IRKCaptureSwitch = irk_capture_ns.class_(
    "IRKCaptureSwitch", switch.Switch, cg.Component
)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_IRK_CAPTURE_ID): cv.use_id(IRKCaptureComponent),
    cv.Optional(CONF_ADVERTISING): switch.switch_schema(
        IRKCaptureSwitch
    ),
})

async def to_code(config):
    parent = await cg.get_variable(config[CONF_IRK_CAPTURE_ID])
    
    if CONF_ADVERTISING in config:
        sw = await switch.new_switch(config[CONF_ADVERTISING])
        cg.add(parent.set_advertising_switch(sw))

