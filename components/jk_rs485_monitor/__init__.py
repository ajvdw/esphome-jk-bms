import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID
from esphome.core import CORE

DEPENDENCIES = ["uart"]
MULTI_CONF = True
CONF_JK_RS485_MONITOR_ID = "jk_rs485_monitor_id"
CONF_RX_TIMEOUT = "rx_timeout"
CONF_PROTOCOL_VERSION = "protocol_version"

jk_rs485_monitor_ns = cg.esphome_ns.namespace("jk_rs485_monitor")

try:
    JkRS485Monitor
    JkRS485MonitorDevice
except NameError:
    JkRS485Monitor = jk_rs485_monitor_ns.class_("JkRS485Monitor", cg.Component, uart.UARTDevice)
    JkRS485MonitorDevice = jk_rs485_monitor_ns.class_("JkRS485MonitorDevice")


ProtocolVersion = jk_rs485_monitor_ns.enum("ProtocolVersion")
PROTOCOL_VERSION_OPTIONS = {
    "JK04": ProtocolVersion.PROTOCOL_VERSION_JK04,
    "JK02_24S": ProtocolVersion.PROTOCOL_VERSION_JK02_24S,
    "JK02_32S": ProtocolVersion.PROTOCOL_VERSION_JK02_32S,
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(JkRS485Monitor),
            cv.Required(CONF_PROTOCOL_VERSION): cv.enum(
                PROTOCOL_VERSION_OPTIONS, upper=True
            ),         
            # Time without new bytes after which an incomplete message is given up
            cv.Optional(
                CONF_RX_TIMEOUT, default="50ms"
            ): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

JK_RS485_MONITOR_COMPONENT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_JK_RS485_MONITOR_ID): cv.use_id(JkRS485Monitor),
    }
)


async def to_code(config):
    cg.add_global(jk_rs485_monitor_ns.using)
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    await uart.register_uart_device(var, config)
    cg.add(var.set_rx_timeout(config[CONF_RX_TIMEOUT]))



def jk_rs485_monitor_device_schema():
    schema = {
        cv.GenerateID(CONF_JK_RS485_MONITOR_ID): cv.use_id(JkRS485Monitor),
    }
    return cv.Schema(schema)


async def register_jk_rs485_bms_device(var, config):
    parent = await cg.get_variable(config[CONF_JK_RS485_MONITOR_ID])
    cg.add(var.set_parent(parent))
                                                   
    cg.add(parent.register_device(var))
