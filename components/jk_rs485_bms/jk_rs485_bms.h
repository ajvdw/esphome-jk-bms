#pragma once

#include <array>
#include <string>
#include <vector>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

#include "../jk_rs485_monitor/jk_rs485_monitor.h"

namespace esphome {
namespace jk_rs485_bms {

// One JK-BMS on the RS485 bus. It receives the replies of the jk_rs485_monitor that come from its own address and
// publishes them as sensors.
class JkRS485Bms : public PollingComponent, public jk_rs485_monitor::JkRS485MonitorDevice {
 public:
  // --- Cell voltages and resistances: one sensor per cell, set by index
  void set_cell_voltage_sensor(uint8_t cell, sensor::Sensor *sensor) { this->cells_[cell].voltage_sensor = sensor; }
  void set_cell_resistance_sensor(uint8_t cell, sensor::Sensor *sensor) {
    this->cells_[cell].resistance_sensor = sensor;
  }
  void set_temperature_sensor(uint8_t index, sensor::Sensor *sensor) { this->temperatures_[index] = sensor; }

  // --- Sensors. SUB_* declares the member <name>_sensor_ and the setter set_<name>_sensor().
  // Cells
  SUB_SENSOR(cell_count_real)
  SUB_SENSOR(cell_voltage_min)
  SUB_SENSOR(cell_voltage_max)
  SUB_SENSOR(cell_voltage_min_cell_number)
  SUB_SENSOR(cell_voltage_max_cell_number)
  SUB_SENSOR(cell_delta_voltage)
  SUB_SENSOR(cell_average_voltage)
  SUB_SENSOR(cell_resistance_min)
  SUB_SENSOR(cell_resistance_max)
  SUB_SENSOR(cell_resistance_min_cell_number)
  SUB_SENSOR(cell_resistance_max_cell_number)

  // Battery
  SUB_SENSOR(battery_voltage)
  SUB_SENSOR(battery_current)
  SUB_SENSOR(battery_power)
  SUB_SENSOR(battery_power_charging)
  SUB_SENSOR(battery_power_discharging)
  SUB_SENSOR(battery_capacity_setting)
  SUB_SENSOR(battery_capacity_remaining)
  SUB_SENSOR(battery_capacity_remaining_derived)
  SUB_SENSOR(battery_capacity_state_of_charge)
  SUB_SENSOR(battery_capacity_total_charging_cycle)
  SUB_SENSOR(battery_soh_valuation)
  SUB_SENSOR(battery_strings)
  SUB_SENSOR(battery_total_runtime)
  SUB_SENSOR(battery_total_alarms_count)
  SUB_SENSOR(battery_total_alarms_active)
  SUB_SENSOR(actual_battery_capacity)
  SUB_SENSOR(charging_cycles)
  SUB_SENSOR(max_discharging_current)

  // Temperatures
  SUB_SENSOR(temperature_powertube)
  SUB_SENSOR(temperature_sensors)

  // Balancing and heating
  SUB_SENSOR(balancing_current)
  SUB_SENSOR(balancing_direction)
  SUB_SENSOR(heating_current)

  // Status
  SUB_SENSOR(errors_bitmask)
  SUB_SENSOR(operation_mode_bitmask)
  SUB_SENSOR(emergency_time_countdown)

  // Protection settings
  SUB_SENSOR(cell_pressure_difference_protection)
  SUB_SENSOR(balancing_opening_pressure_difference)
  SUB_SENSOR(powertube_temperature_protection)
  SUB_SENSOR(powertube_temperature_protection_recovery)
  SUB_SENSOR(temperature_sensor_temperature_protection)
  SUB_SENSOR(temperature_sensor_temperature_recovery)
  SUB_SENSOR(temperature_sensor_temperature_difference_protection)
  SUB_SENSOR(charging_high_temperature_protection)
  SUB_SENSOR(discharging_high_temperature_protection)
  SUB_SENSOR(charging_low_temperature_protection)
  SUB_SENSOR(charging_low_temperature_recovery)
  SUB_SENSOR(discharging_low_temperature_protection)
  SUB_SENSOR(discharging_low_temperature_recovery)
  SUB_SENSOR(discharging_overcurrent_protection_release_time)
  SUB_SENSOR(discharging_short_circuit_protection_release_time)
  SUB_SENSOR(charging_overcurrent_protection_release_time)
  SUB_SENSOR(charging_short_circuit_protection_release_time)
  SUB_SENSOR(cell_undervoltage_protection_release_time)
  SUB_SENSOR(cell_overvoltage_protection_release_time)

  // Device settings
  SUB_SENSOR(current_calibration)
  SUB_SENSOR(start_current_calibration)
  SUB_SENSOR(device_address)
  SUB_SENSOR(sleep_wait_time)
  SUB_SENSOR(smart_sleep_time)
  SUB_SENSOR(alarm_low_volume)
  SUB_SENSOR(manufacturing_date)
  SUB_SENSOR(protocol_version)
  SUB_SENSOR(uart1_protocol_number)
  SUB_SENSOR(uart2_protocol_number)

  // --- Binary sensors: <name>_binary_sensor_ / set_<name>_binary_sensor()
  SUB_BINARY_SENSOR(status_online)
  SUB_BINARY_SENSOR(status_balancing)
  SUB_BINARY_SENSOR(status_precharging)
  SUB_BINARY_SENSOR(status_charging)
  SUB_BINARY_SENSOR(status_discharging)
  SUB_BINARY_SENSOR(status_heating)

  // Alarms of the protection bits, in the order of their bits (bytes 134-136 of the cell info frame)
  SUB_BINARY_SENSOR(alarm_wireres)
  SUB_BINARY_SENSOR(alarm_mosotp)
  SUB_BINARY_SENSOR(alarm_cellquantity)
  SUB_BINARY_SENSOR(alarm_cursensorerr)
  SUB_BINARY_SENSOR(alarm_cellovp)
  SUB_BINARY_SENSOR(alarm_batovp)
  SUB_BINARY_SENSOR(alarm_chocp)
  SUB_BINARY_SENSOR(alarm_chscp)
  SUB_BINARY_SENSOR(alarm_chotp)
  SUB_BINARY_SENSOR(alarm_chutp)
  SUB_BINARY_SENSOR(alarm_cpuauxcommuerr)
  SUB_BINARY_SENSOR(alarm_celluvp)
  SUB_BINARY_SENSOR(alarm_batuvp)
  SUB_BINARY_SENSOR(alarm_dchocp)
  SUB_BINARY_SENSOR(alarm_dchscp)
  SUB_BINARY_SENSOR(alarm_dchotp)
  SUB_BINARY_SENSOR(alarm_chargemos)
  SUB_BINARY_SENSOR(alarm_dischargemos)
  SUB_BINARY_SENSOR(alarm_gpsdisconneted)
  SUB_BINARY_SENSOR(alarm_modifypwdintime)
  SUB_BINARY_SENSOR(alarm_dischargeonfailed)
  SUB_BINARY_SENSOR(alarm_batteryovertemp)
  SUB_BINARY_SENSOR(alarm_temperaturesensoranomaly)
  SUB_BINARY_SENSOR(alarm_plcmoduleanomaly)

  // Alarms of absent temperature sensors (byte 182 of the cell info frame)
  SUB_BINARY_SENSOR(alarm_mostempsensorabsent)
  SUB_BINARY_SENSOR(alarm_battempsensor1absent)
  SUB_BINARY_SENSOR(alarm_battempsensor2absent)
  SUB_BINARY_SENSOR(alarm_battempsensor3absent)
  SUB_BINARY_SENSOR(alarm_battempsensor4absent)
  SUB_BINARY_SENSOR(alarm_battempsensor5absent)

  // --- Text sensors: <name>_text_sensor_ / set_<name>_text_sensor()
  SUB_TEXT_SENSOR(errors)
  SUB_TEXT_SENSOR(charge_status)
  SUB_TEXT_SENSOR(total_runtime_formatted)
  SUB_TEXT_SENSOR(network_nodes_available)
  SUB_TEXT_SENSOR(info_vendorid)
  SUB_TEXT_SENSOR(info_hardware_version)
  SUB_TEXT_SENSOR(info_software_version)
  SUB_TEXT_SENSOR(info_device_name)
  SUB_TEXT_SENSOR(info_device_password)
  SUB_TEXT_SENSOR(info_device_serial_number)
  SUB_TEXT_SENSOR(info_device_setup_passcode)
  // Configurable, but nothing is published to them (the frames don't carry the information)
  SUB_TEXT_SENSOR(operation_status)
  SUB_TEXT_SENSOR(battery_type)
  SUB_TEXT_SENSOR(password)
  SUB_TEXT_SENSOR(device_type)
  SUB_TEXT_SENSOR(software_version)
  SUB_TEXT_SENSOR(manufacturer)

 public:
  void set_address(uint8_t address) { this->address_ = address; }
  uint8_t get_address() const { return this->address_; }

  void setup() override;
  void update() override;
  void dump_config() override;

  void on_jk_rs485_monitor_data(const uint8_t &origin_address, const uint8_t &frame_type,
                                const std::vector<uint8_t> &data, const std::string &nodes_available) override;

 protected:
  static constexpr uint8_t MAX_CELLS = 32;
  static constexpr uint8_t MAX_TEMPERATURES = 5;
  static constexpr uint32_t ONLINE_DATA_MAX_AGE_MS = 60000;
  static constexpr uint32_t OFFLINE_TIMEOUT_MS = 60000;

  struct Cell {
    sensor::Sensor *voltage_sensor{nullptr};
    sensor::Sensor *resistance_sensor{nullptr};
  };

  void decode_jk02_settings_(const std::vector<uint8_t> &data);
  void decode_jk02_cell_info_(const std::vector<uint8_t> &data);
  void decode_device_info_(const std::vector<uint8_t> &data);

  // The BMS is online once a settings frame and a plausible cell info frame were received
  void update_online_state_();
  void track_status_online_();
  void publish_device_unavailable_();

  std::array<binary_sensor::BinarySensor *, 24> protection_alarm_sensors_() const;
  std::array<binary_sensor::BinarySensor *, 6> temperature_alarm_sensors_() const;

  void publish_state_(binary_sensor::BinarySensor *binary_sensor, bool state);
  void publish_state_(sensor::Sensor *sensor, float value);
  void publish_state_(text_sensor::TextSensor *text_sensor, const std::string &state);
  // Publishes an alarm and counts it in the total of alarms (only for alarms that are configured)
  void publish_alarm_state_(binary_sensor::BinarySensor *binary_sensor, bool state);

  static std::string error_bits_to_string_(uint32_t bitmask);
  static std::string charge_status_id_to_string_(uint8_t status);

  uint8_t address_{0};
  uint8_t battery_total_alarms_count_{0};
  uint8_t battery_total_alarms_active_{0};
  std::string nodes_available_;

  Cell cells_[MAX_CELLS];
  sensor::Sensor *temperatures_[MAX_TEMPERATURES]{};

  // Number of cells the BMS is configured for, from the settings frame (0: settings not received yet)
  float cell_count_settings_value_{0};

  // Online tracking
  uint32_t last_response_ms_{0};
  uint32_t last_settings_ms_{0};
  uint32_t last_cellinfo_ms_{0};
  bool offline_published_{false};
  bool settings_ok_{false};
  bool cellinfo_ok_{false};
  bool voltage_ok_{false};
  bool soc_ok_{false};
};

}  // namespace jk_rs485_bms
}  // namespace esphome
