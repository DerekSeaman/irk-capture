"""Text sensor platform for IRK Capture component."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID

from . import CONF_IRK_CAPTURE_ID, IRKCaptureComponent, irk_capture_ns

CONF_LAST_IRK = "last_irk"
CONF_LAST_ADDRESS = "last_address"

IRKCaptureTextSensor = irk_capture_ns.class_(
    "IRKCaptureTextSensor", text_sensor.TextSensor, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_IRK_CAPTURE_ID): cv.use_id(IRKCaptureComponent),
        cv.Optional(CONF_LAST_IRK): text_sensor.text_sensor_schema(
            IRKCaptureTextSensor
        ),
        cv.Optional(CONF_LAST_ADDRESS): text_sensor.text_sensor_schema(
            IRKCaptureTextSensor
        ),
    }
)


async def to_code(config):
    """Generate code for text sensors."""
    parent = await cg.get_variable(config[CONF_IRK_CAPTURE_ID])

    if CONF_LAST_IRK in config:
        sens = await text_sensor.new_text_sensor(config[CONF_LAST_IRK])
        cg.add(parent.set_irk_sensor(sens))

    if CONF_LAST_ADDRESS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_LAST_ADDRESS])
        cg.add(parent.set_address_sensor(sens))
