#include "jk_rs485_bms.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace esphome {
namespace jk_rs485_bms {

static const char *const TAG = "jk_rs485_bms";

// Frame types of the replies
static const uint8_t FRAME_TYPE_SETTINGS = 0x01;
static const uint8_t FRAME_TYPE_CELL_INFO = 0x02;
static const uint8_t FRAME_TYPE_DEVICE_INFO = 0x03;

// Layout of the cell info frame: the fields behind the cell voltages are shifted by 16 bytes, the fields behind the
// cell resistances by 32 bytes (8 cells more than the 24 the original frame had, 2 bytes per voltage / resistance).
static const uint8_t SHIFT_AFTER_VOLTAGES = 16;
static const uint8_t SHIFT_AFTER_RESISTANCES = 2 * SHIFT_AFTER_VOLTAGES;

static const size_t MIN_CELL_INFO_SIZE = 228 + SHIFT_AFTER_RESISTANCES;
static const size_t MIN_SETTINGS_SIZE = 287;
static const size_t MIN_DEVICE_INFO_SIZE = 268;

// The names of the bits of the error bitmask and of the alarm bits, bit 0 first
static const uint8_t ERRORS_SIZE = 24;
static const char *const ERRORS[ERRORS_SIZE] = {
    "Wire resistance",
    "MOS OTP",
    "Cell quantity",
    "Current sensor error",
    "Cell OVP",
    "Battery OVP",
    "Charge OCP",
    "Charge SCP",
    "Charge OTP",
    "Charge UTP",
    "CPU aux comm error",
    "Cell UVP",
    "Battery UVP",
    "Discharge OCP",
    "Discharge SCP",
    "Discharge OTP",
    "Charge MOS",
    "Discharge MOS",
    "GPS disconneted",
    "Modify PWD. in time",
    "Discharge On Failed",
    "Battery Over Temp Alarm",
    "Temperature sensor anomaly",
    "PLCModule anomaly",
};

// --- Little endian values of a frame, as float
static uint16_t read_u16(const uint8_t *p) { return (uint16_t) (p[0] | (p[1] << 8)); }

static uint32_t read_u32(const uint8_t *p) {
  return (uint32_t) p[0] | ((uint32_t) p[1] << 8) | ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
}

static float uint16_to_float(const uint8_t *p) { return (float) read_u16(p); }
static float int16_to_float(const uint8_t *p) { return (float) (int16_t) read_u16(p); }
static float uint32_to_float(const uint8_t *p) { return (float) read_u32(p); }
static float int32_to_float(const uint8_t *p) { return (float) (int32_t) read_u32(p); }

static bool check_bit(uint8_t value, uint8_t position) { return (value >> position) & 0x01; }

// Text of a fixed size field of a frame: it ends at the first NUL
static std::string field_to_string(const std::vector<uint8_t> &data, size_t offset, size_t length) {
  std::string text(data.begin() + offset, data.begin() + offset + length);
  text.resize(strlen(text.c_str()));
  return text;
}

static std::string format_total_runtime(float value) {
  int seconds = (int) value;
  const int years = seconds / (24 * 3600 * 365);
  seconds = seconds % (24 * 3600 * 365);
  const int days = seconds / (24 * 3600);
  seconds = seconds % (24 * 3600);
  const int hours = seconds / 3600;
  return (years ? std::to_string(years) + "y " : "") + (days ? std::to_string(days) + "d " : "") +
         (hours ? std::to_string(hours) + "h" : "");
}

// ---------------------------------------------------------------------------------------------------------------
// Component
// ---------------------------------------------------------------------------------------------------------------

void JkRS485Bms::setup() {
  this->offline_published_ = true;
  this->publish_state_(this->status_online_binary_sensor_, false);
  this->publish_state_(this->errors_text_sensor_, "Offline");
}

void JkRS485Bms::update() { this->track_status_online_(); }

void JkRS485Bms::dump_config() {
  ESP_LOGCONFIG(TAG, "JkRS485Bms:");
  ESP_LOGCONFIG(TAG, "  RS485 address: 0x%02X", this->address_);
}

// ---------------------------------------------------------------------------------------------------------------
// Frames from the monitor
// ---------------------------------------------------------------------------------------------------------------

void JkRS485Bms::on_jk_rs485_monitor_data(const uint8_t &origin_address, const uint8_t &frame_type,
                                          const std::vector<uint8_t> &data, const std::string &nodes_available) {
  if (this->nodes_available_ != nodes_available) {
    this->nodes_available_ = nodes_available;
    this->publish_state_(this->network_nodes_available_text_sensor_, this->nodes_available_);
  }

  if (origin_address != this->address_) {
    ESP_LOGD(TAG, "This BMS address is: %d  and address received %d ==> IDLE", this->address_, origin_address);
    return;
  }

  this->last_response_ms_ = millis();
  this->offline_published_ = false;

  ESP_LOGD(TAG, "This BMS address is: %d  and address received %d ==> WORKING (frame type:%d)", this->address_,
           origin_address, frame_type);
  switch (frame_type) {
    case FRAME_TYPE_SETTINGS:
      this->decode_jk02_settings_(data);
      break;
    case FRAME_TYPE_CELL_INFO:
      if (this->cell_count_settings_value_ > 0) {
        this->decode_jk02_cell_info_(data);
      } else {
        ESP_LOGI(TAG, "Frame type 0x%02X received from address 0x%02X. But 0x01 frame type must be processed first",
                 frame_type, origin_address);
      }
      break;
    case FRAME_TYPE_DEVICE_INFO:
      ESP_LOGI(TAG, "Decoding DEVICE info frame");
      this->decode_device_info_(data);
      break;
    default:
      ESP_LOGW(TAG, "Unsupported FRAME TYPE (0x%02X)", frame_type);
  }

  if (frame_type == FRAME_TYPE_CELL_INFO) {
    this->update_online_state_();
  }
}

void JkRS485Bms::update_online_state_() {
  if (!this->settings_ok_) {
    ESP_LOGI(TAG, "===== [BMS 0x%02X] ONLINE GATE | settings=0 (waiting for 0x01) =====", this->address_);
    return;
  }

  const uint32_t now = millis();
  const bool settings_fresh = (now - this->last_settings_ms_) <= ONLINE_DATA_MAX_AGE_MS;
  const bool cellinfo_fresh = (now - this->last_cellinfo_ms_) <= ONLINE_DATA_MAX_AGE_MS;
  if (this->cellinfo_ok_ && this->voltage_ok_ && this->soc_ok_ && settings_fresh && cellinfo_fresh) {
    this->offline_published_ = false;
    this->publish_state_(this->status_online_binary_sensor_, true);
  } else {
    ESP_LOGI(TAG, "===== [BMS 0x%02X] ONLINE GATE | NOT READY =====", this->address_);
  }
}

void JkRS485Bms::track_status_online_() {
  if (this->last_response_ms_ == 0) {
    return;
  }

  if ((millis() - this->last_response_ms_) > OFFLINE_TIMEOUT_MS && !this->offline_published_) {
    ESP_LOGI(TAG, "===== [BMS 0x%02X] ONLINE -> OFFLINE =====", this->address_);
    this->publish_device_unavailable_();
    this->offline_published_ = true;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Decoding
// ---------------------------------------------------------------------------------------------------------------

void JkRS485Bms::decode_jk02_settings_(const std::vector<uint8_t> &data) {
  if (data.size() < MIN_SETTINGS_SIZE) {
    ESP_LOGW(TAG, "Settings frame too short (%d bytes, need >= %d) - ignoring", (int) data.size(),
             (int) MIN_SETTINGS_SIZE);
    return;
  }

  ESP_LOGI(TAG, "Decoding settings  frame.... [ADDRESS: %02X] %d bytes received", this->address_, (int) data.size());

  // Only these settings are used, the rest of the frame is not decoded
  this->publish_state_(this->powertube_temperature_protection_sensor_, int32_to_float(&data[106]) * 0.1f);
  this->publish_state_(this->powertube_temperature_protection_recovery_sensor_, int32_to_float(&data[110]) * 0.1f);
  this->cell_count_settings_value_ = uint32_to_float(&data[114]);
  this->publish_state_(this->smart_sleep_time_sensor_, (float) data[286]);

  this->settings_ok_ = true;
  this->last_settings_ms_ = millis();
  this->cellinfo_ok_ = false;
  this->voltage_ok_ = false;
  this->soc_ok_ = false;
}

void JkRS485Bms::decode_jk02_cell_info_(const std::vector<uint8_t> &data) {
  if (data.size() < MIN_CELL_INFO_SIZE) {
    ESP_LOGW(TAG, "Cell info frame too short (%d bytes, need >= %d) - ignoring", (int) data.size(),
             (int) MIN_CELL_INFO_SIZE);
    return;
  }

  // Index offset of everything behind the cell resistances
  const size_t tail = SHIFT_AFTER_RESISTANCES;

  ESP_LOGI(TAG, "Decoding cell info frame.... [ADDRESS: %02X] %d bytes received", this->address_, (int) data.size());

  // --- Cells
  uint8_t cells = MAX_CELLS;
  // The count comes from the settings frame: ignore a value that doesn't fit the frame
  const float cells_from_settings = this->cell_count_settings_value_;
  if (cells_from_settings >= 1 && cells_from_settings <= cells) {
    cells = (uint8_t) cells_from_settings;
  }

  float cell_voltage_min = 100.0f;
  float cell_voltage_max = -100.0f;
  float cell_resistance_min = 1000.0f;
  float cell_resistance_max = -1000.0f;
  uint8_t cell_count_real = 0;
  uint8_t cell_resistance_min_cell_number = 0;
  uint8_t cell_resistance_max_cell_number = 0;

  for (uint8_t i = 0; i < cells; i++) {
    const float cell_voltage = uint16_to_float(&data[i * 2 + 6]) * 0.001f;
    const float cell_resistance = uint16_to_float(&data[i * 2 + 64 + SHIFT_AFTER_VOLTAGES]) * 0.001f;
    if (cell_voltage > 0) {
      cell_count_real++;
      cell_voltage_min = std::min(cell_voltage_min, cell_voltage);
      cell_voltage_max = std::max(cell_voltage_max, cell_voltage);

      if (cell_resistance < cell_resistance_min) {
        cell_resistance_min = cell_resistance;
        cell_resistance_min_cell_number = i;
      }
      if (cell_resistance > cell_resistance_max) {
        cell_resistance_max = cell_resistance;
        cell_resistance_max_cell_number = i;
      }
    }

    ESP_LOGD(TAG, "[ADDRESS: %02X]  %02d --> V: %fV  R: %fohm", this->address_, i, cell_voltage, cell_resistance);
    this->publish_state_(this->cells_[i].voltage_sensor, cell_voltage);
    this->publish_state_(this->cells_[i].resistance_sensor, cell_resistance);
  }

  this->publish_state_(this->cell_count_real_sensor_, (float) cell_count_real);

  if (cell_count_real > 0) {
    this->publish_state_(this->cell_voltage_min_sensor_, cell_voltage_min);
    this->publish_state_(this->cell_voltage_max_sensor_, cell_voltage_max);
    this->publish_state_(this->cell_resistance_min_sensor_, cell_resistance_min);
    this->publish_state_(this->cell_resistance_max_sensor_, cell_resistance_max);
    this->publish_state_(this->cell_resistance_max_cell_number_sensor_, (float) cell_resistance_max_cell_number + 1);
    this->publish_state_(this->cell_resistance_min_cell_number_sensor_, (float) cell_resistance_min_cell_number + 1);
  } else {
    this->publish_state_(this->cell_voltage_min_sensor_, NAN);
    this->publish_state_(this->cell_voltage_max_sensor_, NAN);
    this->publish_state_(this->cell_resistance_min_sensor_, NAN);
    this->publish_state_(this->cell_resistance_max_sensor_, NAN);
    this->publish_state_(this->cell_resistance_max_cell_number_sensor_, NAN);
    this->publish_state_(this->cell_resistance_min_cell_number_sensor_, NAN);
  }

  this->publish_state_(this->cell_average_voltage_sensor_, uint16_to_float(&data[58 + SHIFT_AFTER_VOLTAGES]) * 0.001f);
  this->publish_state_(this->cell_delta_voltage_sensor_, uint16_to_float(&data[60 + SHIFT_AFTER_VOLTAGES]) * 0.001f);
  this->publish_state_(this->cell_voltage_max_cell_number_sensor_, (float) data[62 + SHIFT_AFTER_VOLTAGES] + 1);
  this->publish_state_(this->cell_voltage_min_cell_number_sensor_, (float) data[63 + SHIFT_AFTER_VOLTAGES] + 1);

  // --- Battery
  this->publish_state_(this->temperature_powertube_sensor_, int16_to_float(&data[112 + tail]) * 0.1f);

  const float battery_voltage = uint32_to_float(&data[118 + tail]) * 0.001f;
  this->publish_state_(this->battery_voltage_sensor_, battery_voltage);
  this->voltage_ok_ = !std::isnan(battery_voltage) && !std::isinf(battery_voltage);

  const float current = int32_to_float(&data[126 + tail]) * 0.001f;
  this->publish_state_(this->battery_current_sensor_, current);

  const float power = battery_voltage * current;
  this->publish_state_(this->battery_power_sensor_, power);
  this->publish_state_(this->battery_power_charging_sensor_, std::max(0.0f, power));
  this->publish_state_(this->battery_power_discharging_sensor_, std::abs(std::min(0.0f, power)));

  this->publish_state_(this->temperatures_[0], int16_to_float(&data[130 + tail]) * 0.1f);
  this->publish_state_(this->temperatures_[1], int16_to_float(&data[132 + tail]) * 0.1f);

  // --- Alarms: 3 bytes with 8 alarm bits each
  this->battery_total_alarms_count_ = 0;
  this->battery_total_alarms_active_ = 0;
  const auto alarms = this->protection_alarm_sensors_();
  for (uint8_t i = 0; i < alarms.size(); i++) {
    this->publish_alarm_state_(alarms[i], check_bit(data[134 + i / 8], i % 8));
  }

  const uint32_t raw_errors_bitmask = read_u32(&data[134 + tail]);
  this->publish_state_(this->errors_bitmask_sensor_, (float) raw_errors_bitmask);
  this->publish_state_(this->errors_text_sensor_, this->error_bits_to_string_(raw_errors_bitmask));

  // --- Balancing
  this->publish_state_(this->balancing_current_sensor_, int16_to_float(&data[138 + tail]) * 0.001f);

  const uint8_t balancing_direction = data[140 + tail];
  this->publish_state_(this->balancing_direction_sensor_, (float) balancing_direction);
  this->publish_state_(this->status_balancing_binary_sensor_, balancing_direction == 1 || balancing_direction == 2);

  // --- Capacity and state
  const float soc = (float) data[141 + tail];
  this->publish_state_(this->battery_capacity_state_of_charge_sensor_, soc);
  this->soc_ok_ = !std::isnan(soc) && !std::isinf(soc) && soc >= 0.0f && soc <= 100.0f;
  this->cellinfo_ok_ = true;
  this->last_cellinfo_ms_ = millis();

  this->publish_state_(this->battery_capacity_remaining_sensor_, int32_to_float(&data[142 + tail]) * 0.001f);
  this->publish_state_(this->battery_capacity_setting_sensor_, int32_to_float(&data[146 + tail]) * 0.001f);
  this->publish_state_(this->charging_cycles_sensor_, uint32_to_float(&data[150 + tail]));
  this->publish_state_(this->battery_capacity_total_charging_cycle_sensor_,
                       uint32_to_float(&data[154 + tail]) * 0.001f);
  this->publish_state_(this->battery_soh_valuation_sensor_, uint32_to_float(&data[158 + tail]));
  this->publish_state_(this->status_precharging_binary_sensor_, check_bit(data[159 + tail], 0));

  const float total_runtime = uint32_to_float(&data[162 + tail]);
  this->publish_state_(this->battery_total_runtime_sensor_, total_runtime);
  this->publish_state_(this->total_runtime_formatted_text_sensor_, format_total_runtime(total_runtime));

  this->publish_state_(this->status_charging_binary_sensor_, check_bit(data[166 + tail], 0));
  this->publish_state_(this->status_discharging_binary_sensor_, check_bit(data[167 + tail], 0));

  this->publish_state_(this->discharging_overcurrent_protection_release_time_sensor_,
                       uint16_to_float(&data[170 + tail]));
  this->publish_state_(this->discharging_short_circuit_protection_release_time_sensor_,
                       uint16_to_float(&data[172 + tail]));
  this->publish_state_(this->charging_overcurrent_protection_release_time_sensor_,
                       uint16_to_float(&data[174 + tail]));
  this->publish_state_(this->charging_short_circuit_protection_release_time_sensor_,
                       uint16_to_float(&data[176 + tail]));
  this->publish_state_(this->cell_undervoltage_protection_release_time_sensor_, uint16_to_float(&data[178 + tail]));
  this->publish_state_(this->cell_overvoltage_protection_release_time_sensor_, uint16_to_float(&data[180 + tail]));

  this->publish_state_(this->status_heating_binary_sensor_, check_bit(data[183 + tail], 0));
  this->publish_state_(this->emergency_time_countdown_sensor_, uint16_to_float(&data[186 + tail]));
  this->publish_state_(this->heating_current_sensor_, int16_to_float(&data[204 + tail]) * 0.001f);

  this->publish_state_(this->temperatures_[2], int16_to_float(&data[222 + tail]) * 0.1f);
  this->publish_state_(this->temperatures_[3], int16_to_float(&data[224 + tail]) * 0.1f);
  this->publish_state_(this->temperatures_[4], int16_to_float(&data[226 + tail]) * 0.1f);

  // A temperature sensor is absent if its bit is not set
  const auto absent = this->temperature_alarm_sensors_();
  for (uint8_t i = 0; i < absent.size(); i++) {
    this->publish_alarm_state_(absent[i], !check_bit(data[182 + tail], i));
  }

  this->publish_state_(this->battery_total_alarms_count_sensor_, (float) this->battery_total_alarms_count_);
  this->publish_state_(this->battery_total_alarms_active_sensor_, (float) this->battery_total_alarms_active_);

  if (current < 0.0f) {
    this->publish_state_(this->charge_status_text_sensor_, "Bulk");
  } else if (data.size() > (size_t) (248 + tail)) {
    // Charge status reported by the BMS (0x00: Bulk, 0x01: Absorption, 0x02: Float)
    this->publish_state_(this->charge_status_text_sensor_, this->charge_status_id_to_string_(data[248 + tail]));
  }
}

void JkRS485Bms::decode_device_info_(const std::vector<uint8_t> &data) {
  if (data.size() < MIN_DEVICE_INFO_SIZE) {
    ESP_LOGW(TAG, "Device info frame too short (%d bytes, need >= %d) - ignoring", (int) data.size(),
             (int) MIN_DEVICE_INFO_SIZE);
    return;
  }

  ESP_LOGI(TAG, "Device info frame (%d bytes) received", (int) data.size());

  this->publish_state_(this->info_vendorid_text_sensor_, field_to_string(data, 6, 16));
  this->publish_state_(this->info_hardware_version_text_sensor_, field_to_string(data, 22, 8));
  this->publish_state_(this->info_software_version_text_sensor_, field_to_string(data, 30, 8));
  this->publish_state_(this->info_device_name_text_sensor_, field_to_string(data, 46, 16));
  this->publish_state_(this->info_device_password_text_sensor_, field_to_string(data, 62, 16));
  this->publish_state_(this->info_device_serial_number_text_sensor_, field_to_string(data, 86, 11));
  this->publish_state_(this->info_device_setup_passcode_text_sensor_, field_to_string(data, 118, 16));

  this->publish_state_(this->uart1_protocol_number_sensor_, (float) data[178]);
  this->publish_state_(this->uart2_protocol_number_sensor_, (float) data[212]);
}

// ---------------------------------------------------------------------------------------------------------------
// Alarms
// ---------------------------------------------------------------------------------------------------------------

// In the order of the bits of bytes 134-136 of the cell info frame (see ERRORS)
std::array<binary_sensor::BinarySensor *, 24> JkRS485Bms::protection_alarm_sensors_() const {
  return {
      this->alarm_wireres_binary_sensor_,
      this->alarm_mosotp_binary_sensor_,
      this->alarm_cellquantity_binary_sensor_,
      this->alarm_cursensorerr_binary_sensor_,
      this->alarm_cellovp_binary_sensor_,
      this->alarm_batovp_binary_sensor_,
      this->alarm_chocp_binary_sensor_,
      this->alarm_chscp_binary_sensor_,
      this->alarm_chotp_binary_sensor_,
      this->alarm_chutp_binary_sensor_,
      this->alarm_cpuauxcommuerr_binary_sensor_,
      this->alarm_celluvp_binary_sensor_,
      this->alarm_batuvp_binary_sensor_,
      this->alarm_dchocp_binary_sensor_,
      this->alarm_dchscp_binary_sensor_,
      this->alarm_dchotp_binary_sensor_,
      this->alarm_chargemos_binary_sensor_,
      this->alarm_dischargemos_binary_sensor_,
      this->alarm_gpsdisconneted_binary_sensor_,
      this->alarm_modifypwdintime_binary_sensor_,
      this->alarm_dischargeonfailed_binary_sensor_,
      this->alarm_batteryovertemp_binary_sensor_,
      this->alarm_temperaturesensoranomaly_binary_sensor_,
      this->alarm_plcmoduleanomaly_binary_sensor_,
  };
}

// In the order of the bits of byte 182 of the cell info frame: the power tube and the 5 battery sensors
std::array<binary_sensor::BinarySensor *, 6> JkRS485Bms::temperature_alarm_sensors_() const {
  return {
      this->alarm_mostempsensorabsent_binary_sensor_,  this->alarm_battempsensor1absent_binary_sensor_,
      this->alarm_battempsensor2absent_binary_sensor_, this->alarm_battempsensor3absent_binary_sensor_,
      this->alarm_battempsensor4absent_binary_sensor_, this->alarm_battempsensor5absent_binary_sensor_,
  };
}

std::string JkRS485Bms::error_bits_to_string_(const uint32_t mask) {
  std::string errors;
  for (uint8_t i = 0; i < ERRORS_SIZE; i++) {
    if (mask & (1u << i)) {
      if (!errors.empty()) {
        errors.append(";");
      }
      errors.append(ERRORS[i]);
    }
  }
  return errors;
}

std::string JkRS485Bms::charge_status_id_to_string_(const uint8_t status) {
  switch (status) {
    case 0x00:
      return "Bulk";
    case 0x01:
      return "Absorption";
    case 0x02:
      return "Float";
    default:
      return "Unknown";
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Publishing
// ---------------------------------------------------------------------------------------------------------------

void JkRS485Bms::publish_device_unavailable_() {
  for (auto *binary_sensor : this->protection_alarm_sensors_()) {
    this->publish_state_(binary_sensor, false);
  }
  for (auto *binary_sensor : this->temperature_alarm_sensors_()) {
    this->publish_state_(binary_sensor, false);
  }
  for (auto *binary_sensor : {this->status_online_binary_sensor_, this->status_balancing_binary_sensor_,
                              this->status_precharging_binary_sensor_, this->status_charging_binary_sensor_,
                              this->status_discharging_binary_sensor_, this->status_heating_binary_sensor_}) {
    this->publish_state_(binary_sensor, false);
  }

  this->settings_ok_ = false;
  this->cellinfo_ok_ = false;
  this->voltage_ok_ = false;
  this->soc_ok_ = false;
  this->last_settings_ms_ = 0;
  this->last_cellinfo_ms_ = 0;
  this->cell_count_settings_value_ = 0;

  this->publish_state_(this->errors_text_sensor_, "Offline");
  this->publish_state_(this->charge_status_text_sensor_, "Offline");
  this->publish_state_(this->total_runtime_formatted_text_sensor_, "NAN");

  for (auto *sensor : {
           this->cell_count_real_sensor_,
           this->cell_voltage_min_sensor_,
           this->cell_voltage_max_sensor_,
           this->cell_voltage_min_cell_number_sensor_,
           this->cell_voltage_max_cell_number_sensor_,
           this->cell_resistance_min_sensor_,
           this->cell_resistance_max_sensor_,
           this->cell_resistance_min_cell_number_sensor_,
           this->cell_resistance_max_cell_number_sensor_,
           this->battery_capacity_state_of_charge_sensor_,
           this->battery_soh_valuation_sensor_,
           this->balancing_current_sensor_,
           this->balancing_direction_sensor_,
           this->cell_delta_voltage_sensor_,
           this->cell_average_voltage_sensor_,
           this->temperature_powertube_sensor_,
           this->battery_voltage_sensor_,
           this->battery_current_sensor_,
           this->battery_power_sensor_,
           this->battery_power_charging_sensor_,
           this->battery_power_discharging_sensor_,
           this->battery_capacity_setting_sensor_,
           this->battery_capacity_remaining_sensor_,
           this->battery_capacity_remaining_derived_sensor_,
           this->temperature_sensors_sensor_,
           this->charging_cycles_sensor_,
           this->battery_capacity_total_charging_cycle_sensor_,
           this->battery_strings_sensor_,
           this->errors_bitmask_sensor_,
           this->operation_mode_bitmask_sensor_,
           this->cell_pressure_difference_protection_sensor_,
           this->balancing_opening_pressure_difference_sensor_,
           this->powertube_temperature_protection_sensor_,
           this->powertube_temperature_protection_recovery_sensor_,
           this->temperature_sensor_temperature_protection_sensor_,
           this->temperature_sensor_temperature_recovery_sensor_,
           this->temperature_sensor_temperature_difference_protection_sensor_,
           this->charging_high_temperature_protection_sensor_,
           this->discharging_high_temperature_protection_sensor_,
           this->charging_low_temperature_protection_sensor_,
           this->charging_low_temperature_recovery_sensor_,
           this->discharging_low_temperature_protection_sensor_,
           this->discharging_low_temperature_recovery_sensor_,
           this->current_calibration_sensor_,
           this->device_address_sensor_,
           this->sleep_wait_time_sensor_,
           this->alarm_low_volume_sensor_,
           this->manufacturing_date_sensor_,
           this->battery_total_runtime_sensor_,
           this->start_current_calibration_sensor_,
           this->actual_battery_capacity_sensor_,
           this->protocol_version_sensor_,
       }) {
    this->publish_state_(sensor, NAN);
  }

  for (auto *temperature_sensor : this->temperatures_) {
    this->publish_state_(temperature_sensor, NAN);
  }
  for (auto &cell : this->cells_) {
    this->publish_state_(cell.voltage_sensor, NAN);
    this->publish_state_(cell.resistance_sensor, NAN);
  }
}

void JkRS485Bms::publish_state_(binary_sensor::BinarySensor *binary_sensor, bool state) {
  if (binary_sensor == nullptr) {
    return;
  }
  binary_sensor->publish_state(state);
}

void JkRS485Bms::publish_state_(sensor::Sensor *sensor, float value) {
  if (sensor == nullptr) {
    return;
  }
  if (std::isinf(value)) {
    ESP_LOGW(TAG, "Sensor value is infinite, not publishing.");
    return;
  }
  sensor->publish_state(value);
}

void JkRS485Bms::publish_state_(text_sensor::TextSensor *text_sensor, const std::string &state) {
  if (text_sensor == nullptr) {
    return;
  }
  text_sensor->publish_state(state);
}

void JkRS485Bms::publish_alarm_state_(binary_sensor::BinarySensor *binary_sensor, bool state) {
  if (binary_sensor == nullptr) {
    return;
  }
  this->battery_total_alarms_count_++;
  if (state) {
    this->battery_total_alarms_active_++;
  }
  binary_sensor->publish_state(state);
}

}  // namespace jk_rs485_bms
}  // namespace esphome
