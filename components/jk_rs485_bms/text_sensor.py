import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PASSWORD, ICON_EMPTY, ICON_TIMELAPSE

from . import CONF_JK_RS485_BMS_ID, JK_RS485_BMS_COMPONENT_SCHEMA, inherit_hub_device_id

DEPENDENCIES = ["jk_rs485_bms"]

CONF_BATTERY_TYPE = "battery_type"
CONF_ERRORS = "errors"
CONF_CHARGE_STATUS = "charge_status"
CONF_OPERATION_STATUS = "operation_status"
CONF_DEVICE_TYPE = "device_type"
CONF_SOFTWARE_VERSION = "software_version"
CONF_MANUFACTURER = "manufacturer"
CONF_TOTAL_RUNTIME_FORMATTED = "total_runtime_formatted"
CONF_NETWORK_NODES_AVAILABLE = "network_nodes_available"

CONF_INFO_VENDORID = "info_vendorid"
CONF_INFO_HARDWARE_VERSION = "info_hardware_version"
CONF_INFO_SOFTWARE_VERSION = "info_software_version"
CONF_INFO_DEVICE_NAME = "info_device_name"
CONF_INFO_DEVICE_PASSWORD = "info_device_password"
CONF_INFO_DEVICE_SERIAL_NUMBER = "info_device_serial_number"
CONF_INFO_DEVICE_SETUP_PASSCODE = "info_device_setup_passcode"

ICON_BATTERY_TYPE = "mdi:car-battery"
ICON_ERRORS = "mdi:alert-circle-outline"
ICON_CHARGE_STATUS = "mdi:battery-clock"
ICON_OPERATION_STATUS = "mdi:heart-pulse"
ICON_PASSWORD = "mdi:lock-outline"
ICON_NETWORK = "mdi:lan"
ICON_INFORMATION = "mdi:information"

# "<option>" -> icon; the sensor is set with set_<option>_text_sensor() of the hub
TEXT_SENSOR_ICONS = {
    CONF_OPERATION_STATUS: ICON_OPERATION_STATUS,
    CONF_ERRORS: ICON_ERRORS,
    CONF_CHARGE_STATUS: ICON_CHARGE_STATUS,
    CONF_BATTERY_TYPE: ICON_BATTERY_TYPE,
    CONF_PASSWORD: ICON_PASSWORD,
    CONF_DEVICE_TYPE: ICON_EMPTY,
    CONF_SOFTWARE_VERSION: ICON_EMPTY,
    CONF_MANUFACTURER: ICON_EMPTY,
    CONF_TOTAL_RUNTIME_FORMATTED: ICON_TIMELAPSE,
    CONF_NETWORK_NODES_AVAILABLE: ICON_NETWORK,
    CONF_INFO_VENDORID: ICON_INFORMATION,
    CONF_INFO_HARDWARE_VERSION: ICON_INFORMATION,
    CONF_INFO_SOFTWARE_VERSION: ICON_INFORMATION,
    CONF_INFO_DEVICE_NAME: ICON_INFORMATION,
    CONF_INFO_DEVICE_PASSWORD: ICON_INFORMATION,
    CONF_INFO_DEVICE_SERIAL_NUMBER: ICON_INFORMATION,
    CONF_INFO_DEVICE_SETUP_PASSCODE: ICON_INFORMATION,
}

CONFIG_SCHEMA = cv.All(
    inherit_hub_device_id,
    JK_RS485_BMS_COMPONENT_SCHEMA.extend(
        {
            cv.Optional(key): text_sensor.text_sensor_schema(icon=icon)
            for key, icon in TEXT_SENSOR_ICONS.items()
        }
    ),
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_JK_RS485_BMS_ID])
    for key in TEXT_SENSOR_ICONS:
        if key in config:
            conf = config[key]
            sens = cg.new_Pvariable(conf[CONF_ID])
            await text_sensor.register_text_sensor(sens, conf)
            cg.add(getattr(hub, f"set_{key}_text_sensor")(sens))
