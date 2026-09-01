import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client
from esphome.const import CONF_ID

# Експортуємо простір імен для sensor.py
jbd_bms_ble_ns = cg.esphome_ns.namespace('jbd_bms_ble')
JbdBmsBle = jbd_bms_ble_ns.class_('JbdBmsBle', cg.PollingComponent, ble_client.BLEClientNode)

CONFIG_SCHEMA = cv.All(
    cv.ensure_list(
        cv.Schema({
            cv.GenerateID(): cv.declare_id(JbdBmsBle),
            cv.Required('ble_client_id'): cv.use_id(ble_client.BLEClient),
        }).extend(cv.polling_component_schema('10s'))
    )
)

async def to_code(config):
    for conf in config:
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)
        await ble_client.register_ble_node(var, conf)