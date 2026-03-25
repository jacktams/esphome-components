"""Automower BLE Button Platform."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_ID
from .. import automower_ble_ns, AutomowerBLE

CONF_AUTOMOWER_BLE_ID = "automower_ble_id"
CONF_START = "start"
CONF_PAUSE = "pause"
CONF_PARK = "park"

AutomowerStartButton = automower_ble_ns.class_(
    "AutomowerStartButton", button.Button, cg.Parented.template(AutomowerBLE)
)
AutomowerPauseButton = automower_ble_ns.class_(
    "AutomowerPauseButton", button.Button, cg.Parented.template(AutomowerBLE)
)
AutomowerParkButton = automower_ble_ns.class_(
    "AutomowerParkButton", button.Button, cg.Parented.template(AutomowerBLE)
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AUTOMOWER_BLE_ID): cv.use_id(AutomowerBLE),
        cv.Optional(CONF_START): button.button_schema(AutomowerStartButton),
        cv.Optional(CONF_PAUSE): button.button_schema(AutomowerPauseButton),
        cv.Optional(CONF_PARK): button.button_schema(AutomowerParkButton),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AUTOMOWER_BLE_ID])

    if CONF_START in config:
        btn = await button.new_button(config[CONF_START])
        await cg.register_parented(btn, config[CONF_AUTOMOWER_BLE_ID])

    if CONF_PAUSE in config:
        btn = await button.new_button(config[CONF_PAUSE])
        await cg.register_parented(btn, config[CONF_AUTOMOWER_BLE_ID])

    if CONF_PARK in config:
        btn = await button.new_button(config[CONF_PARK])
        await cg.register_parented(btn, config[CONF_AUTOMOWER_BLE_ID])
