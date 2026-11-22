import esphome.codegen as cg
from esphome.components import jk_rs485_sniffer
import esphome.config_validation as cv
from esphome.const import CONF_DEVICE_ID, CONF_ID, CONF_PLATFORM
from esphome.core import CORE

from ..jk_rs485_sniffer import CONF_JK_RS485_SNIFFER_ID, JK_RS485_SNIFFER_COMPONENT_SCHEMA, jk_rs485_sniffer_ns

AUTO_LOAD = ["jk_rs485_sniffer", "binary_sensor", "sensor", "text_sensor"]
CODEOWNERS = ["@syssi"]
MULTI_CONF = True

CONF_JK_RS485_BMS_ID = "jk_rs485_bms_id"
CONF_RS485_ADDRESS = "rs485_address"

jk_rs485_bms_ns = cg.esphome_ns.namespace("jk_rs485_bms")
JkRS485Bms = jk_rs485_bms_ns.class_("JkRS485Bms", cg.PollingComponent, jk_rs485_sniffer.JkRS485SnifferDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(JkRS485Bms),
            cv.Required(CONF_RS485_ADDRESS): cv.int_,
            # Sub-device (esphome: devices:) all entities of this BMS belong to
            cv.Optional(CONF_DEVICE_ID): cv.sub_device_id,
        }
    )
    .extend(cv.polling_component_schema("5s"))
    .extend(jk_rs485_sniffer.jk_rs485_sniffer_device_schema())
)

def _hub_raw_config(hub_id):
    """Find the raw (unvalidated) config of a jk_rs485_bms hub by its id."""
    hubs = (CORE.raw_config or {}).get("jk_rs485_bms") or []
    if isinstance(hubs, dict):
        hubs = [hubs]
    if hub_id is None:
        # The hub id is auto-filled later on; unambiguous with a single hub only
        return hubs[0] if len(hubs) == 1 else None
    for hub in hubs:
        if isinstance(hub, dict) and str(hub.get(CONF_ID)) == str(hub_id):
            return hub
    return None


def inherit_hub_device_id(value):
    """Pre-validator for the sensor, binary_sensor and text_sensor platforms.

    If the jk_rs485_bms hub declares a ``device_id`` every entity of that hub is placed on this
    sub-device, so ``device_id`` doesn't have to be repeated (and can't be mixed up) per entity.
    This has to happen before the entities are validated: ESPHome's duplicate name check takes
    the device of an entity into account. Works for sub-devices with ``relation: via`` and
    ``relation: child`` alike.

    An entity that explicitly names a different device than its hub is rejected.
    """
    if not isinstance(value, dict):
        return value
    hub = _hub_raw_config(value.get(CONF_JK_RS485_BMS_ID))
    hub_device = hub.get(CONF_DEVICE_ID) if hub else None
    if not hub_device:
        return value

    value = dict(value)
    for key, conf in value.items():
        if key in (CONF_PLATFORM, CONF_JK_RS485_BMS_ID) or not isinstance(conf, dict):
            continue
        entity_device = conf.get(CONF_DEVICE_ID)
        if entity_device in (None, ""):
            value[key] = {**conf, CONF_DEVICE_ID: hub_device}
        elif str(entity_device) != str(hub_device):
            raise cv.Invalid(
                f"'{key}' is placed on device '{entity_device}', but its jk_rs485_bms hub "
                f"'{value.get(CONF_JK_RS485_BMS_ID)}' belongs to device '{hub_device}'. "
                f"Remove the device_id of the entity, it is inherited from the hub.",
                [key, CONF_DEVICE_ID],
            )
    return value


JK_RS485_BMS_COMPONENT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_JK_RS485_BMS_ID): cv.use_id(JkRS485Bms),
    }
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])                                                             
    await cg.register_component(var, config)                                                          
    await jk_rs485_sniffer.register_jk_rs485_bms_device(var, config)                                 
    cg.add(var.set_address(config[CONF_RS485_ADDRESS]))                                            
