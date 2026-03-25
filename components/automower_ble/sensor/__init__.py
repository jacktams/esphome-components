"""Automower BLE Sensor Platform."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_BATTERY_LEVEL,
    DEVICE_CLASS_BATTERY,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
)
from .. import automower_ble_ns, AutomowerBLE

CONF_AUTOMOWER_BLE_ID = "automower_ble_id"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AUTOMOWER_BLE_ID): cv.use_id(AutomowerBLE),
        cv.Optional(CONF_BATTERY_LEVEL): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            device_class=DEVICE_CLASS_BATTERY,
            state_class=STATE_CLASS_MEASUREMENT,
            accuracy_decimals=0,
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AUTOMOWER_BLE_ID])

    if CONF_BATTERY_LEVEL in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_LEVEL])
        cg.add(parent.set_battery_sensor(sens))
