import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID, DEVICE_CLASS_VOLTAGE, DEVICE_CLASS_CURRENT, DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_TEMPERATURE, DEVICE_CLASS_BATTERY, 
    UNIT_VOLT, UNIT_AMPERE, UNIT_CELSIUS, UNIT_PERCENT
)
from . import jbd_bms_ble_ns, JbdBmsBle

CONF_JBD_BMS_BLE_ID = "jbd_bms_ble_id"

# Описуємо, які ключі можна писати в YAML
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_JBD_BMS_BLE_ID): cv.use_id(JbdBmsBle),
    cv.Optional("voltage"): sensor.sensor_schema(unit_of_measurement=UNIT_VOLT, device_class=DEVICE_CLASS_VOLTAGE, accuracy_decimals=2),
    cv.Optional("current"): sensor.sensor_schema(unit_of_measurement=UNIT_AMPERE, device_class=DEVICE_CLASS_CURRENT, accuracy_decimals=2),
    cv.Optional("cell_temperature"): sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS, device_class=DEVICE_CLASS_TEMPERATURE, accuracy_decimals=1),
    cv.Optional("mosfet_temperature"): sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS, device_class=DEVICE_CLASS_TEMPERATURE, accuracy_decimals=1),
    cv.Optional("soc"): sensor.sensor_schema(unit_of_measurement=UNIT_PERCENT, device_class=DEVICE_CLASS_BATTERY, accuracy_decimals=0),
    cv.Optional("soh"): sensor.sensor_schema(unit_of_measurement=UNIT_PERCENT, device_class=DEVICE_CLASS_BATTERY, accuracy_decimals=0),
    cv.Optional("cycles"): sensor.sensor_schema(accuracy_decimals=0),
    cv.Optional("batterystate"): sensor.sensor_schema(accuracy_decimals=0),
    cv.Optional("equilibriumstate"): sensor.sensor_schema(accuracy_decimals=0),
    cv.Optional("charge"): sensor.sensor_schema(unit_of_measurement="Ah", device_class=DEVICE_CLASS_ENERGY, accuracy_decimals=2),
    cv.Optional("capacity"): sensor.sensor_schema(unit_of_measurement="Ah", device_class=DEVICE_CLASS_ENERGY, accuracy_decimals=2),
    cv.Optional("cells"): cv.ensure_list(sensor.sensor_schema(unit_of_measurement=UNIT_VOLT, device_class=DEVICE_CLASS_VOLTAGE, accuracy_decimals=3)),
})

async def to_code(config):
    hub = await cg.get_variable(config[CONF_JBD_BMS_BLE_ID])
    
    # Обробка стандартних сенсорів
    for key in ["voltage", "current", "mosfet_temperature", "cell_temperature", "soc", "soh", "cycles", "equilibriumstate", "batterystate", "charge", "capacity"]:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(hub, f"set_{key}_sensor")(sens))
    
    # Обробка списку комірок
    if "cells" in config:
        for i, cell_conf in enumerate(config["cells"]):
            sens = await sensor.new_sensor(cell_conf)
            cg.add(hub.set_cell_sensor(i, sens))
