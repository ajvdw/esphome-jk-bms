import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_EMPTY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ICON_COUNTER,
    ICON_EMPTY,
    ICON_TIMELAPSE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_EMPTY,
    UNIT_OHM,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_WATT,
)

from . import CONF_JK_RS485_BMS_ID, JK_RS485_BMS_COMPONENT_SCHEMA, inherit_hub_device_id

DEPENDENCIES = ["jk_rs485_bms"]

MAX_CELLS = 32
NUM_TEMPERATURE_SENSORS = 5

ICON_ACTUAL_BATTERY_CAPACITY = "mdi:battery-50"
ICON_ALARM_LOW_VOLUME = "mdi:volume-high"
ICON_BATTERY_CAPACITY_REMAINING = "mdi:battery-50"
ICON_BATTERY_CAPACITY_REMAINING_DERIVED = "mdi:battery-50"
ICON_BATTERY_CAPACITY_SETTING = "mdi:battery-50"
ICON_BATTERY_STRINGS = "mdi:car-battery"
ICON_CELL_RESISTANCE = "mdi:omega"
ICON_CELL_VOLTAGE_MAX_CELL_NUMBER = "mdi:battery-plus-outline"
ICON_CELL_VOLTAGE_MIN_CELL_NUMBER = "mdi:battery-minus-outline"
ICON_CHARGING_CYCLES = "mdi:battery-sync"
ICON_CLOCK = "mdi:clock-outline"
ICON_CURRENT_DC = "mdi:current-dc"
ICON_DEVICE_ADDRESS = "mdi:identifier"
ICON_DIRECTION = "mdi:swap-horizontal-bold"
ICON_ERRORS_BITMASK = "mdi:alert-circle-outline"
ICON_OPERATION_MODE_BITMASK = "mdi:heart-pulse"

UNIT_AMPERE_HOURS = "Ah"
UNIT_HOURS = "h"
UNIT_SECONDS = "s"


def _sensor(
    unit,
    accuracy_decimals,
    *,
    device_class=DEVICE_CLASS_EMPTY,
    icon=ICON_EMPTY,
    state_class=STATE_CLASS_MEASUREMENT,
):
    return sensor.sensor_schema(
        unit_of_measurement=unit,
        accuracy_decimals=accuracy_decimals,
        device_class=device_class,
        icon=icon,
        state_class=state_class,
    )


# Sensors with their own option: "<option>" -> set_<option>_sensor() of the hub
SENSORS = {
    "balancing_direction": _sensor(UNIT_EMPTY, 0, device_class=DEVICE_CLASS_CURRENT, icon=ICON_DIRECTION),

    "cell_count_real": _sensor(UNIT_EMPTY, 0),
    "cell_voltage_min": _sensor(UNIT_VOLT, 3, device_class=DEVICE_CLASS_VOLTAGE),
    "cell_voltage_max": _sensor(UNIT_VOLT, 3, device_class=DEVICE_CLASS_VOLTAGE),
    "cell_voltage_min_cell_number": _sensor(UNIT_EMPTY, 0, icon=ICON_CELL_VOLTAGE_MIN_CELL_NUMBER),
    "cell_voltage_max_cell_number": _sensor(UNIT_EMPTY, 0, icon=ICON_CELL_VOLTAGE_MAX_CELL_NUMBER),
    "cell_delta_voltage": _sensor(UNIT_VOLT, 3, device_class=DEVICE_CLASS_VOLTAGE),
    "cell_average_voltage": _sensor(UNIT_VOLT, 3, device_class=DEVICE_CLASS_VOLTAGE),
    "cell_resistance_min": _sensor(UNIT_OHM, 3),
    "cell_resistance_max": _sensor(UNIT_OHM, 3),
    "cell_resistance_min_cell_number": _sensor(UNIT_EMPTY, 0, icon=ICON_CELL_RESISTANCE),
    "cell_resistance_max_cell_number": _sensor(UNIT_EMPTY, 0, icon=ICON_CELL_RESISTANCE),

    "temperature_powertube": _sensor(UNIT_CELSIUS, 0, device_class=DEVICE_CLASS_TEMPERATURE),

    "battery_capacity_state_of_charge": _sensor(UNIT_PERCENT, 0, device_class=DEVICE_CLASS_BATTERY, icon=cv.UNDEFINED),
    "battery_soh_valuation": _sensor(UNIT_PERCENT, 0, device_class=DEVICE_CLASS_BATTERY, icon=cv.UNDEFINED),
    "battery_voltage": _sensor(UNIT_VOLT, 2, device_class=DEVICE_CLASS_VOLTAGE),
    "battery_current": _sensor(UNIT_AMPERE, 2, device_class=DEVICE_CLASS_CURRENT, icon=ICON_CURRENT_DC),
    "battery_power": _sensor(UNIT_WATT, 2, device_class=DEVICE_CLASS_POWER),
    "battery_power_charging": _sensor(UNIT_WATT, 2, device_class=DEVICE_CLASS_POWER),
    "battery_power_discharging": _sensor(UNIT_WATT, 2, device_class=DEVICE_CLASS_POWER),
    "battery_capacity_setting": _sensor(UNIT_AMPERE_HOURS, 3, icon=ICON_BATTERY_CAPACITY_SETTING),
    "battery_capacity_remaining": _sensor(UNIT_AMPERE_HOURS, 3, icon=ICON_BATTERY_CAPACITY_REMAINING),
    "battery_capacity_remaining_derived": _sensor(UNIT_AMPERE_HOURS, 1, icon=ICON_BATTERY_CAPACITY_REMAINING_DERIVED),

    "temperature_sensors": _sensor(UNIT_EMPTY, 0),

    "charging_cycles": _sensor(UNIT_EMPTY, 0, icon=ICON_CHARGING_CYCLES),

    "battery_capacity_total_charging_cycle": _sensor(UNIT_AMPERE_HOURS, 0, icon=ICON_COUNTER),
    "battery_strings": _sensor(UNIT_EMPTY, 0, icon=ICON_BATTERY_STRINGS),

    "errors_bitmask": _sensor(UNIT_EMPTY, 0, icon=ICON_ERRORS_BITMASK),

    "operation_mode_bitmask": _sensor(UNIT_EMPTY, 0, icon=ICON_OPERATION_MODE_BITMASK),

    "cell_pressure_difference_protection": _sensor(UNIT_VOLT, 3, device_class=DEVICE_CLASS_VOLTAGE),

    "balancing_opening_pressure_difference": _sensor(UNIT_VOLT, 3, device_class=DEVICE_CLASS_VOLTAGE),

    "powertube_temperature_protection": _sensor(UNIT_CELSIUS, 0, device_class=DEVICE_CLASS_TEMPERATURE),
    "powertube_temperature_protection_recovery": _sensor(UNIT_CELSIUS, 0, device_class=DEVICE_CLASS_TEMPERATURE),

    "temperature_sensor_temperature_protection": _sensor(UNIT_CELSIUS, 0, device_class=DEVICE_CLASS_TEMPERATURE),
    "temperature_sensor_temperature_recovery": _sensor(UNIT_CELSIUS, 0, device_class=DEVICE_CLASS_TEMPERATURE),
    "temperature_sensor_temperature_difference_protection": _sensor(UNIT_CELSIUS, 0, device_class=DEVICE_CLASS_TEMPERATURE),

    "charging_high_temperature_protection": _sensor(UNIT_CELSIUS, 0, device_class=DEVICE_CLASS_TEMPERATURE),

    "discharging_high_temperature_protection": _sensor(UNIT_CELSIUS, 0, device_class=DEVICE_CLASS_TEMPERATURE),

    "charging_low_temperature_protection": _sensor(UNIT_CELSIUS, 0, device_class=DEVICE_CLASS_TEMPERATURE),
    "charging_low_temperature_recovery": _sensor(UNIT_CELSIUS, 0, device_class=DEVICE_CLASS_TEMPERATURE),

    "discharging_low_temperature_protection": _sensor(UNIT_CELSIUS, 0, device_class=DEVICE_CLASS_TEMPERATURE),
    "discharging_low_temperature_recovery": _sensor(UNIT_CELSIUS, 0, device_class=DEVICE_CLASS_TEMPERATURE),

    "current_calibration": _sensor(UNIT_AMPERE, 3, device_class=DEVICE_CLASS_CURRENT, icon=ICON_CURRENT_DC),

    "device_address": _sensor(UNIT_EMPTY, 0, icon=ICON_DEVICE_ADDRESS),

    "sleep_wait_time": _sensor(UNIT_SECONDS, 0, icon=ICON_TIMELAPSE),

    "alarm_low_volume": _sensor(UNIT_PERCENT, 0, icon=ICON_ALARM_LOW_VOLUME),

    "manufacturing_date": _sensor(UNIT_EMPTY, 0),

    "battery_total_runtime": _sensor(UNIT_SECONDS, 0, icon=ICON_TIMELAPSE, state_class=STATE_CLASS_TOTAL_INCREASING),

    "balancing_current": _sensor(UNIT_AMPERE, 2, device_class=DEVICE_CLASS_CURRENT, icon=ICON_CURRENT_DC),

    "heating_current": _sensor(UNIT_AMPERE, 2, device_class=DEVICE_CLASS_CURRENT, icon=ICON_CURRENT_DC),

    "max_discharging_current": _sensor(UNIT_AMPERE, 3, device_class=DEVICE_CLASS_CURRENT, icon=ICON_CURRENT_DC),

    "start_current_calibration": _sensor(UNIT_AMPERE, 3, device_class=DEVICE_CLASS_CURRENT, icon=ICON_CURRENT_DC),

    "actual_battery_capacity": _sensor(UNIT_AMPERE_HOURS, 0, icon=ICON_ACTUAL_BATTERY_CAPACITY),

    "protocol_version": _sensor(UNIT_EMPTY, 0),

    "smart_sleep_time": _sensor(UNIT_HOURS, 0, icon=ICON_CLOCK),

    "battery_total_alarms_count": _sensor(UNIT_EMPTY, 0, icon=ICON_ALARM_LOW_VOLUME),
    "battery_total_alarms_active": _sensor(UNIT_EMPTY, 0, icon=ICON_ALARM_LOW_VOLUME),

    "emergency_time_countdown": _sensor(UNIT_SECONDS, 0, icon=ICON_TIMELAPSE),

    "uart1_protocol_number": _sensor(UNIT_EMPTY, 0),

    "uart2_protocol_number": _sensor(UNIT_EMPTY, 0),

    "discharging_overcurrent_protection_release_time": _sensor(UNIT_SECONDS, 0, icon=ICON_CLOCK),
    "discharging_short_circuit_protection_release_time": _sensor(UNIT_SECONDS, 0, icon=ICON_CLOCK),

    "charging_overcurrent_protection_release_time": _sensor(UNIT_SECONDS, 0, icon=ICON_CLOCK),
    "charging_short_circuit_protection_release_time": _sensor(UNIT_SECONDS, 0, icon=ICON_CLOCK),

    "cell_undervoltage_protection_release_time": _sensor(UNIT_SECONDS, 0, icon=ICON_CLOCK),
    "cell_overvoltage_protection_release_time": _sensor(UNIT_SECONDS, 0, icon=ICON_CLOCK),
}

# Options that are accepted for backwards compatibility but do nothing: the BMS doesn't report these values,
# so no entity is created for them.
IGNORED_SENSORS = {
    "discharging_overcurrent_protection_delay": _sensor(UNIT_SECONDS, 0, icon=ICON_TIMELAPSE),
    "discharging_overcurrent_protection_recovery_delay": _sensor(UNIT_SECONDS, 0, icon=ICON_TIMELAPSE),
}

# Sensors per cell and per temperature sensor: "<prefix>_<number>" -> set_<prefix>_sensor(index) of the hub
CELL_VOLTAGE_SCHEMA = _sensor(UNIT_VOLT, 3, device_class=DEVICE_CLASS_VOLTAGE)
CELL_RESISTANCE_SCHEMA = _sensor(UNIT_OHM, 3, icon=ICON_CELL_RESISTANCE)
TEMPERATURE_SCHEMA = _sensor(UNIT_CELSIUS, 1, device_class=DEVICE_CLASS_TEMPERATURE)

CELL_VOLTAGES = [f"cell_voltage_{i:02d}" for i in range(1, MAX_CELLS + 1)]
CELL_RESISTANCES = [f"cell_resistance_{i:02d}" for i in range(1, MAX_CELLS + 1)]
TEMPERATURES = [f"temperature_sensor_{i}" for i in range(1, NUM_TEMPERATURE_SENSORS + 1)]

CONFIG_SCHEMA = cv.All(
    inherit_hub_device_id,
    JK_RS485_BMS_COMPONENT_SCHEMA.extend(
        {
            **{cv.Optional(key): schema for key, schema in SENSORS.items()},
            **{cv.Optional(key): schema for key, schema in IGNORED_SENSORS.items()},
            **{cv.Optional(key): CELL_VOLTAGE_SCHEMA for key in CELL_VOLTAGES},
            **{cv.Optional(key): CELL_RESISTANCE_SCHEMA for key in CELL_RESISTANCES},
            **{cv.Optional(key): TEMPERATURE_SCHEMA for key in TEMPERATURES},
        }
    ),
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_JK_RS485_BMS_ID])
    for i, key in enumerate(CELL_VOLTAGES):
        if key in config:
            cg.add(hub.set_cell_voltage_sensor(i, await sensor.new_sensor(config[key])))
    for i, key in enumerate(CELL_RESISTANCES):
        if key in config:
            cg.add(hub.set_cell_resistance_sensor(i, await sensor.new_sensor(config[key])))
    for i, key in enumerate(TEMPERATURES):
        if key in config:
            cg.add(hub.set_temperature_sensor(i, await sensor.new_sensor(config[key])))
    for key in SENSORS:
        if key in config:
            cg.add(getattr(hub, f"set_{key}_sensor")(await sensor.new_sensor(config[key])))
