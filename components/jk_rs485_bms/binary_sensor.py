import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID, DEVICE_CLASS_CONNECTIVITY, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_JK_RS485_BMS_ID, JK_RS485_BMS_COMPONENT_SCHEMA, inherit_hub_device_id
from .const import (
    CONF_STATUS_BALANCING,
    CONF_STATUS_CHARGING,
    CONF_STATUS_DISCHARGING,
    CONF_STATUS_HEATING,
    CONF_STATUS_ONLINE,
    CONF_STATUS_PRECHARGING,
    PROTECTION_ALARMS,
    TEMPERATURE_ALARMS,
)

DEPENDENCIES = ["jk_rs485_bms"]

ICON_CHARGING = "mdi:battery-charging"
ICON_DISCHARGING = "mdi:power-plug"
ICON_BALANCING = "mdi:battery-heart-variant"
ICON_HEATING = "mdi:radiator"
ICON_ALARM = "mdi:alarm-light"
ICON_TEMP_ALARM = "mdi:thermometer-alert"

STATUS_ICONS = {
    CONF_STATUS_PRECHARGING: ICON_CHARGING,
    CONF_STATUS_CHARGING: ICON_CHARGING,
    CONF_STATUS_DISCHARGING: ICON_DISCHARGING,
    CONF_STATUS_BALANCING: ICON_BALANCING,
    CONF_STATUS_HEATING: ICON_HEATING,
}

# "<option>" -> set_<option>_binary_sensor() of the hub
BINARY_SENSORS = {
    **{key: binary_sensor.binary_sensor_schema(icon=icon) for key, icon in STATUS_ICONS.items()},
    **{key: binary_sensor.binary_sensor_schema(icon=ICON_ALARM) for key in PROTECTION_ALARMS},
    **{key: binary_sensor.binary_sensor_schema(icon=ICON_TEMP_ALARM) for key in TEMPERATURE_ALARMS},
    CONF_STATUS_ONLINE: binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_CONNECTIVITY,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
}

CONFIG_SCHEMA = cv.All(
    inherit_hub_device_id,
    JK_RS485_BMS_COMPONENT_SCHEMA.extend({cv.Optional(key): schema for key, schema in BINARY_SENSORS.items()}),
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_JK_RS485_BMS_ID])
    for key in BINARY_SENSORS:
        if key in config:
            conf = config[key]
            sens = cg.new_Pvariable(conf[CONF_ID])
            await binary_sensor.register_binary_sensor(sens, conf)
            cg.add(getattr(hub, f"set_{key}_binary_sensor")(sens))
