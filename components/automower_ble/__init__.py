"""Automower BLE Component - Direct BLE control of Husqvarna/Flymo/Gardena mowers."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client
from esphome.const import CONF_ID

DEPENDENCIES = ["ble_client"]
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor", "button"]
MULTI_CONF = False

CONF_PIN = "pin"

automower_ble_ns = cg.esphome_ns.namespace("automower_ble")
AutomowerBLE = automower_ble_ns.class_(
    "AutomowerBLE", cg.PollingComponent, ble_client.BLEClientNode
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AutomowerBLE),
            cv.Optional(CONF_PIN, default=0): cv.int_range(min=0, max=9999),
        }
    )
    .extend(cv.polling_component_schema("30s"))
    .extend(ble_client.BLE_CLIENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)

    if config[CONF_PIN] > 0:
        cg.add(var.set_pin(config[CONF_PIN]))
