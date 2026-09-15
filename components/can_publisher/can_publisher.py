import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

can_publisher_ns = cg.esphome_ns.namespace("can_publisher")
CANPublisher = can_publisher_ns.class_("CANPublisher", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(CANPublisher),
})

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var)
