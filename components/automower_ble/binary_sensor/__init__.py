"""Automower BLE Binary Sensor Platform."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID, DEVICE_CLASS_BATTERY_CHARGING, DEVICE_CLASS_CONNECTIVITY
from .. import automower_ble_ns, AutomowerBLE

CONF_AUTOMOWER_BLE_ID = "automower_ble_id"
CONF_CHARGING = "charging"
CONF_CONNECTED = "connected"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AUTOMOWER_BLE_ID): cv.use_id(AutomowerBLE),
        cv.Optional(CONF_CHARGING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_BATTERY_CHARGING,
        ),
        cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AUTOMOWER_BLE_ID])

    if CONF_CHARGING in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CHARGING])
        cg.add(parent.set_charging_sensor(sens))

    if CONF_CONNECTED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CONNECTED])
        cg.add(parent.set_connected_sensor(sens))
