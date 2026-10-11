import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_CANBUS_ID

DEPENDENCIES = ["canbus"]
AUTO_LOAD = ["canbus"]

can_receiver_ns = cg.esphome_ns.namespace("can_receiver")
CanReceiver = can_receiver_ns.class_("CanReceiver", cg.Component)

CONF_MAPPINGS = "mappings"
CONF_CAN_ID = "can_id"
CONF_TYPE = "type"
CONF_SCRIPT = "script"
CONF_LIGHT = "light"
CONF_SWITCH = "switch"
CONF_NUMBER = "number"
CONF_MODBUS_CONTROLLER = "controller"
CONF_REGISTER = "register"

CAN_RECEIVER_TYPES = [
    "shelly_channel",
    "shelly_all",
    "neopixel_brightness",
    "switch_toggle",
    "number_set",
    "modbus_write",
    "script",
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CanReceiver),
        cv.Required(CONF_CANBUS_ID): cv.use_id(cg.esphome_ns.namespace("canbus").class_("CANBus")),
        cv.Optional(CONF_MAPPINGS, default=[]): cv.ensure_list(
            cv.Schema(
                {
                    cv.Required(CONF_CAN_ID): cv.int_range(min=0, max=0x7FF),
                    cv.Required(CONF_TYPE): cv.one_of(*CAN_RECEIVER_TYPES, lower=True),
                    cv.Optional(CONF_SCRIPT): cv.use_id(cg.esphome_ns.namespace("script").class_("Script")),
                    cv.Optional(CONF_LIGHT): cv.use_id(cg.esphome_ns.namespace("light").class_("LightState")),
                    cv.Optional(CONF_SWITCH): cv.use_id(cg.esphome_ns.namespace("switch_").class_("Switch")),
                    cv.Optional(CONF_NUMBER): cv.use_id(cg.esphome_ns.namespace("number").class_("Number")),
                    cv.Optional(CONF_MODBUS_CONTROLLER): cv.use_id(
                        cg.esphome_ns.namespace("modbus_controller").class_("ModbusController")
                    ),
                    cv.Optional(CONF_REGISTER): cv.int_,
                }
            )
        ),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    canbus = await cg.get_variable(config[CONF_CANBUS_ID])
    cg.add(var.set_canbus(canbus))

    for m in config[CONF_MAPPINGS]:
        can_id = m[CONF_CAN_ID]
        mtype = m[CONF_TYPE]

        script = m.get(CONF_SCRIPT)
        light = m.get(CONF_LIGHT)
        switch = m.get(CONF_SWITCH)
        number = m.get(CONF_NUMBER)
        ctrl = m.get(CONF_MODBUS_CONTROLLER)
        reg = m.get(CONF_REGISTER)

        cg.add(
            var.add_mapping(
                can_id,
                mtype,
                script if script is not None else cg.nullptr,
                light if light is not None else cg.nullptr,
                switch if switch is not None else cg.nullptr,
                number if number is not None else cg.nullptr,
                ctrl if ctrl is not None else cg.nullptr,
                reg if reg is not None else 0,
            )
        )
