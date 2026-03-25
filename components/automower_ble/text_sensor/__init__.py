"""Automower BLE Text Sensor Platform."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID
from .. import automower_ble_ns, AutomowerBLE

CONF_AUTOMOWER_BLE_ID = "automower_ble_id"
CONF_STATE = "state"
CONF_ACTIVITY = "activity"
CONF_ERROR = "error"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AUTOMOWER_BLE_ID): cv.use_id(AutomowerBLE),
        cv.Optional(CONF_STATE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_ACTIVITY): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_ERROR): text_sensor.text_sensor_schema(),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AUTOMOWER_BLE_ID])

    if CONF_STATE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_STATE])
        cg.add(parent.set_state_sensor(sens))

    if CONF_ACTIVITY in config:
        sens = await text_sensor.new_text_sensor(config[CONF_ACTIVITY])
        cg.add(parent.set_activity_sensor(sens))

    if CONF_ERROR in config:
        sens = await text_sensor.new_text_sensor(config[CONF_ERROR])
        cg.add(parent.set_error_sensor(sens))
