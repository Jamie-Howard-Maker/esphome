import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import canbus

DEPENDENCIES = ["canbus"]

CONF_CANBUS_ID = "canbus_id"
CONF_UPDATE_INTERVAL = "update_interval"
CONF_LOG_FRAMES = "log_frames"
CONF_FRAMES = "frames"
CONF_CAN_ID = "can_id"
CONF_VALUES = "values"
CONF_SENSOR = "sensor"
CONF_BINARY_SENSOR = "binary_sensor"
CONF_SCALE = "scale"
CONF_OFFSET = "offset"

can_publisher_ns = cg.esphome_ns.namespace("can_publisher")
CANPublisher = can_publisher_ns.class_("CANPublisher", cg.PollingComponent)

VALUE_SCHEMA = cv.Schema({
    cv.Optional(CONF_SENSOR): cv.use_id(cg.Sensor),
    cv.Optional(CONF_BINARY_SENSOR): cv.use_id(cg.BinarySensor),
    cv.Optional(CONF_SCALE, default=1.0): cv.float_,
    cv.Optional(CONF_OFFSET, default=0.0): cv.float_,
})

FRAME_SCHEMA = cv.Schema({
    cv.Required(CONF_CAN_ID): cv.int_range(min=0, max=0x7FF),
    cv.Required(CONF_VALUES): cv.All(cv.ensure_list(VALUE_SCHEMA), cv.Length(min=1, max=4)),
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(CANPublisher),
    cv.Required(CONF_CANBUS_ID): cv.use_id(canbus.Canbus),
    cv.Optional(CONF_UPDATE_INTERVAL, default="1s"): cv.update_interval,
    cv.Optional(CONF_LOG_FRAMES, default=False): cv.boolean,
    cv.Required(CONF_FRAMES): cv.ensure_list(FRAME_SCHEMA),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    canbus_var = await cg.get_variable(config[CONF_CANBUS_ID])
    cg.add(var.set_canbus(canbus_var))
    cg.add(var.set_log_frames(config[CONF_LOG_FRAMES]))

    for frame in config[CONF_FRAMES]:
        values = []
        for val in frame[CONF_VALUES]:
            scale = val[CONF_SCALE]
            offset = val[CONF_OFFSET]
            if CONF_SENSOR in val:
                sens = await cg.get_variable(val[CONF_SENSOR])
                values.append((sens, None, scale, offset))
            else:
                bsens = await cg.get_variable(val[CONF_BINARY_SENSOR])
                values.append((None, bsens, scale, offset))
        cg.add(var.add_frame(frame[CONF_CAN_ID], values))
