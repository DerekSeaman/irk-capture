"""Text input platform for IRK Capture component."""

from pathlib import Path
import re

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text

from . import CONF_BLE_NAME, CONF_IRK_CAPTURE_ID, IRKCaptureComponent, irk_capture_ns

CONF_NEXT_CAPTURE_LABEL = "next_capture_label"


def capture_label_max_len():
    """Read IRKCaptureComponent::CAPTURE_LABEL_MAX_LEN from the C++ header.

    The firmware trims a label to that length, so the entity must offer the
    same limit. Reading it keeps one definition instead of a copy that drifts.
    """
    header = (Path(__file__).parent / "irk_capture.h").read_text()
    return int(re.search(r"\bCAPTURE_LABEL_MAX_LEN = (\d+);", header).group(1))


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
        # 13 fits the Keyboard default ("Logitech K380"); the device trims any
        # other name to 12. Without a limit Home Assistant accepts 255.
        txt = await text.new_text(config[CONF_BLE_NAME], max_length=13)
        cg.add(parent.set_ble_name_text(txt))

    if CONF_NEXT_CAPTURE_LABEL in config:
        # Home Assistant limits the field instead of the device silently
        # trimming it.
        txt = await text.new_text(
            config[CONF_NEXT_CAPTURE_LABEL], max_length=capture_label_max_len()
        )
        cg.add(parent.set_next_capture_label_text(txt))
