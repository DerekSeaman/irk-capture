import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text
from esphome.const import CONF_ID
from . import irk_capture_ns, IRKCaptureComponent

CONF_IRK_CAPTURE_ID = "irk_capture_id"
CONF_BLE_NAME = "ble_name"

IRKCaptureText = irk_capture_ns.class_("IRKCaptureText", text.Text, cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_IRK_CAPTURE_ID): cv.use_id(IRKCaptureComponent),
    cv.Optional(CONF_BLE_NAME): text.text_schema(IRKCaptureText),
})

async def to_code(config):
    parent = await cg.get_variable(config[CONF_IRK_CAPTURE_ID])
    
    if CONF_BLE_NAME in config:
        conf = config[CONF_BLE_NAME]
        var = await text.new_text(conf)
        await cg.register_component(var, conf)
        cg.add(parent.set_ble_name_text(var))
