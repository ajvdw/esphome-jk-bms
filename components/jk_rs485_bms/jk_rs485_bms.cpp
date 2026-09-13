#include "jk_rs485_bms.h"

namespace esphome {
namespace jk_rs485_bms {

float uint32_to_float(const uint8_t* byteArray) {

    uint32_t uintValue = (static_cast<uint32_t>(byteArray[0]) << 0) |
                         (static_cast<uint32_t>(byteArray[1]) << 8) |
                         (static_cast<uint32_t>(byteArray[2]) << 16) |
                         (static_cast<uint32_t>(byteArray[3]) << 24);

    float floatValue = static_cast<float>(uintValue);

    return floatValue;
}

float int32_to_float(const uint8_t* byteArray) {

    int32_t intValue = (static_cast<int32_t>(byteArray[0]) << 0) |
                       (static_cast<int32_t>(byteArray[1]) << 8) |
                       (static_cast<int32_t>(byteArray[2]) << 16)|
                       (static_cast<int32_t>(byteArray[3]) << 24);

    float floatValue = static_cast<float>(intValue);

    return floatValue;
}

float uint16_to_float(const uint8_t *byteArray) {

  uint16_t uintValue = (static_cast<uint16_t>(byteArray[0]) << 0) | (static_cast<uint16_t>(byteArray[1]) << 8);

  float floatValue = static_cast<float>(uintValue);

  return floatValue;
}

float int16_to_float(const uint8_t *byteArray) {

  int16_t intValue = (static_cast<int16_t>(byteArray[0]) << 0) | (static_cast<int16_t>(byteArray[1]) << 8);

  float floatValue = static_cast<float>(intValue);

  return floatValue;
}

static const char *const TAG = "jk_rs485_bms";

static const uint8_t MAX_NO_RESPONSE_COUNT = 10;

static const uint8_t FUNCTION_READ_ALL = 0x06;
static const uint8_t FUNCTION_WRITE_REGISTER = 0x02;

static const uint8_t FRAME_VERSION_JK04 = 0x01;
static const uint8_t FRAME_VERSION_JK02_24S = 0x02;
static const uint8_t FRAME_VERSION_JK02_32S = 0x03;

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

static const uint8_t OPERATION_MODES_SIZE = 4;
static const char *const OPERATION_MODES[OPERATION_MODES_SIZE] = {
    "Charging enabled",
    "Discharging enabled",
    "Balancer enabled",
    "Battery dropped",
};

static const uint8_t BATTERY_TYPES_SIZE = 3;
static const char *const BATTERY_TYPES[BATTERY_TYPES_SIZE] = {
    "Lithium Iron Phosphate",
    "Ternary Lithium",
    "Lithium Titanate",
};

void JkRS485Bms::set_sniffer_parent(jk_rs485_sniffer::JkRS485Sniffer* parent) {
    if (parent == nullptr) {
        ESP_LOGE(TAG, "Trying to set parent to null");
    } else {
        ESP_LOGD(TAG, "Setting parent");
    }
    this->parent_ = parent;
}

jk_rs485_sniffer::JkRS485Sniffer* JkRS485Bms::get_sniffer_parent(void){
    ESP_LOGD(TAG, "Get sniffer parent");
    return(this->parent_);
}

void JkRS485Bms::trigger_bms2sniffer_event(std::string event, std::uint8_t frame_type) {
  if (this->parent_ != nullptr) {
    this->parent_->handle_bms2sniffer_event(this->address_, event, frame_type);
  }
}

void JkRS485Bms::on_jk_rs485_sniffer_data(const uint8_t &origin_address, const uint8_t &frame_type,
                                          const std::vector<uint8_t> &data,
                                          const std::string &nodes_available_received) {

  if (this->nodes_available != nodes_available_received) {
    this->nodes_available = nodes_available_received;
    this->publish_state_(this->network_nodes_available_text_sensor_, this->nodes_available);
  }

  if (origin_address == this->address_) {
    this->last_response_ms_ = millis();
    this->offline_published_ = false;

    ESP_LOGD(TAG, "This BMS address is: %d  and address received %d ==> WORKING (frame type:%d)",
             this->address_, origin_address, frame_type);
    switch (frame_type) {
      case 0x01:
        if (this->protocol_version_ == PROTOCOL_VERSION_JK04) {

        } else {
          this->decode_jk02_settings_(data);
        }
        break;
      case 0x02:
        if (this->protocol_version_ == PROTOCOL_VERSION_JK04) {

        } else {
          if (this->cell_count_settings_value_ > 0) {
            this->decode_jk02_cell_info_(data);
          } else {
            ESP_LOGI(TAG, "Frame type 0x%02X received from address 0x%02X. But 0x01 frame type must be processed first", frame_type,origin_address);
          }

        }
        break;
      case 0x03:
        ESP_LOGI(TAG, "Decoding DEVICE info frame");
        this->decode_device_info_(data);
        break;
      default:
        ESP_LOGW(TAG, "Unsupported FRAME TYPE (0x%02X)", frame_type);
        ESP_LOGD(TAG, "  %s", format_hex_pretty(&data.front(), 150).c_str());
    }

    if (frame_type == 0x02) {
      if (!this->settings_ok_) {
        ESP_LOGI(TAG, "===== [BMS 0x%02X] ONLINE GATE | settings=0 (waiting for 0x01) =====", this->address_);
        return;
      }
      const uint32_t now = millis();
      const bool settings_fresh = (now - this->last_settings_ms_) <= ONLINE_DATA_MAX_AGE_MS;
      const bool cellinfo_fresh = (now - this->last_cellinfo_ms_) <= ONLINE_DATA_MAX_AGE_MS;
  ESP_LOGI(TAG, "===== [BMS 0x%02X] ONLINE GATE | settings=%d cellinfo=%d voltage=%d soc=%d fresh_s=%d fresh_c=%d =====",
           this->address_,
           (int) this->settings_ok_,
           (int) this->cellinfo_ok_,
           (int) this->voltage_ok_,
           (int) this->soc_ok_,
           (int) settings_fresh,
           (int) cellinfo_fresh);
      if (this->settings_ok_ && this->cellinfo_ok_ && this->voltage_ok_ && this->soc_ok_ && settings_fresh &&
          cellinfo_fresh) {
        this->reset_status_online_tracker_();
      } else {
        ESP_LOGI(TAG, "===== [BMS 0x%02X] ONLINE GATE | NOT READY =====", this->address_);
      }
    }

  } else {
    ESP_LOGD(TAG, "This BMS address is: %d  and address received %d ==> IDLE", this->address_, origin_address);
  }
}

void JkRS485Bms::decode_jk02_cell_info_(const std::vector<uint8_t> &data) {

  uint8_t frame_version = FRAME_VERSION_JK02_24S;
  uint8_t offset = 0;
  if (this->protocol_version_ == PROTOCOL_VERSION_JK02_32S) {
    frame_version = FRAME_VERSION_JK02_32S;
    offset = 16;
  }

  const size_t min_len = 228 + offset;
  if (data.size() < min_len) {
    ESP_LOGW(TAG, "Cell info frame too short (%d bytes, need >= %d) - ignoring", data.size(), min_len);
    return;
  }

  ESP_LOGI(TAG, "Decoding cell info frame.... [ADDRESS: %02X] %d bytes received", this->address_, data.size());

  float temp_param_value;

  uint8_t cells = 24 + (offset / 2);
  float cell_voltage_min = 100.0f;
  float cell_voltage_max = -100.0f;
  float cell_resistance_min = 1000.0f;
  float cell_resistance_max = -1000.0f;
  uint8_t cell_count_real = 0;
  uint8_t cell_voltage_min_cell_number = 0;
  uint8_t cell_voltage_max_cell_number = 0;
  uint8_t cell_resistance_min_cell_number = 0;
  uint8_t cell_resistance_max_cell_number = 0;
  float cell_voltage;
  float cell_resistance;

  uint8_t cells_from_settings = (uint8_t) this->cell_count_settings_value_;

  if (cells_from_settings>0){
    cells=cells_from_settings;
  }

  for (uint8_t i = 0; i < cells; i++) {
    cell_voltage    = uint16_to_float(&data[i * 2 + 6]) * 0.001f;
    cell_resistance = uint16_to_float(&data[(i * 2 + 64 + offset)]) * 0.001f;
    if (cell_voltage > 0){
      cell_count_real++;
      if (cell_voltage < cell_voltage_min) {
        cell_voltage_min = cell_voltage;
      }
      if (cell_voltage > cell_voltage_max) {
        cell_voltage_max = cell_voltage;
      }

      if (cell_resistance < cell_resistance_min) {
        cell_resistance_min = cell_resistance;
        cell_resistance_min_cell_number=i;
      }
      if (cell_resistance > cell_resistance_max) {
        cell_resistance_max = cell_resistance;
        cell_resistance_max_cell_number=i;
      }
    }

    ESP_LOGD(TAG, "[ADDRESS: %02X]  %02d --> V: %fV",this->address_,i, cell_voltage);
    if(this->address_==1 && i==2){
      this->publish_state_(this->cells_[i].cell_voltage_sensor_, cell_voltage);
    } else {
      this->publish_state_(this->cells_[i].cell_voltage_sensor_, cell_voltage);
    }
    ESP_LOGD(TAG, "                  --> R: %fohm",cell_resistance);
    this->publish_state_(this->cells_[i].cell_resistance_sensor_, cell_resistance);

  }

  this->publish_state_(this->cell_count_real_sensor_, (float) cell_count_real);

  if (cell_count_real > 0) {
    this->publish_state_(this->cell_voltage_min_sensor_, cell_voltage_min);
    this->publish_state_(this->cell_voltage_max_sensor_, cell_voltage_max);
    this->publish_state_(this->cell_resistance_min_sensor_, cell_resistance_min);
    this->publish_state_(this->cell_resistance_max_sensor_, cell_resistance_max);
    this->publish_state_(this->cell_resistance_max_cell_number_sensor_, (float) cell_resistance_max_cell_number+1);
    this->publish_state_(this->cell_resistance_min_cell_number_sensor_, (float) cell_resistance_min_cell_number+1);
  } else {
    this->publish_state_(this->cell_voltage_min_sensor_, NAN);
    this->publish_state_(this->cell_voltage_max_sensor_, NAN);
    this->publish_state_(this->cell_resistance_min_sensor_, NAN);
    this->publish_state_(this->cell_resistance_max_sensor_, NAN);
    this->publish_state_(this->cell_resistance_max_cell_number_sensor_, NAN);
    this->publish_state_(this->cell_resistance_min_cell_number_sensor_, NAN);
  }

  this->publish_state_(this->cell_average_voltage_sensor_, uint16_to_float(&data[58+offset]) * 0.001f);

  this->publish_state_(this->cell_delta_voltage_sensor_, uint16_to_float(&data[60+offset]) * 0.001f);

  this->publish_state_(this->cell_voltage_max_cell_number_sensor_, (float) data[62 + offset] + 1);

  this->publish_state_(this->cell_voltage_min_cell_number_sensor_, (float) data[63 + offset] + 1);

  offset = offset * 2;

  if (frame_version == FRAME_VERSION_JK02_32S) {
    temp_param_value=int16_to_float(&data[112+offset]) * 0.1f;
    this->publish_state_(this->temperature_powertube_sensor_, temp_param_value);

  }

  float battery_voltage =  uint32_to_float(&data[118+offset]) * 0.001f;
  this->publish_state_(this->battery_voltage_sensor_, battery_voltage);
  this->voltage_ok_ = !std::isnan(battery_voltage) && !std::isinf(battery_voltage);

  float current = int32_to_float(&data[126+offset]) * 0.001f;
  this->publish_state_(this->battery_current_sensor_, current);

  float power = battery_voltage * current;
  this->publish_state_(this->battery_power_sensor_, power);
  this->publish_state_(this->battery_power_charging_sensor_, std::max(0.0f, power));
  this->publish_state_(this->battery_power_discharging_sensor_,
                       std::abs(std::min(0.0f, power)));

  temp_param_value=int16_to_float(&data[130+offset]) * 0.1f;
  this->publish_state_(this->temperatures_[0].temperature_sensor_,temp_param_value);

  temp_param_value=int16_to_float(&data[132+offset]) * 0.1f;
  this->publish_state_(this->temperatures_[1].temperature_sensor_,temp_param_value);

  if (frame_version == FRAME_VERSION_JK02_32S) {
    this->battery_total_alarms_count_ = 0;
    this->battery_total_alarms_active_ = 0;
    this->publish_alarm_state_(this->alarm_wireres_binary_sensor_, this->check_bit_of_byte_(data[134], 0));
    this->publish_alarm_state_(this->alarm_mosotp_binary_sensor_, this->check_bit_of_byte_(data[134], 1));
    this->publish_alarm_state_(this->alarm_cellquantity_binary_sensor_, this->check_bit_of_byte_(data[134], 2));
    this->publish_alarm_state_(this->alarm_cursensorerr_binary_sensor_, this->check_bit_of_byte_(data[134], 3));
    this->publish_alarm_state_(this->alarm_cellovp_binary_sensor_, this->check_bit_of_byte_(data[134], 4));
    this->publish_alarm_state_(this->alarm_batovp_binary_sensor_, this->check_bit_of_byte_(data[134], 5));
    this->publish_alarm_state_(this->alarm_chocp_binary_sensor_, this->check_bit_of_byte_(data[134], 6));
    this->publish_alarm_state_(this->alarm_chscp_binary_sensor_, this->check_bit_of_byte_(data[134], 7));

  }

  if (frame_version == FRAME_VERSION_JK02_32S) {

  } else {

    this->publish_state_(this->temperature_powertube_sensor_, int16_to_float(&data[134+offset]) * 0.1f);
  }

  if (frame_version == FRAME_VERSION_JK02_32S) {
    this->publish_alarm_state_(this->alarm_chotp_binary_sensor_, this->check_bit_of_byte_(data[135], 0));
    this->publish_alarm_state_(this->alarm_chutp_binary_sensor_, this->check_bit_of_byte_(data[135], 1));
    this->publish_alarm_state_(this->alarm_cpuauxcommuerr_binary_sensor_, this->check_bit_of_byte_(data[135], 2));
    this->publish_alarm_state_(this->alarm_celluvp_binary_sensor_, this->check_bit_of_byte_(data[135], 3));
    this->publish_alarm_state_(this->alarm_batuvp_binary_sensor_, this->check_bit_of_byte_(data[135], 4));
    this->publish_alarm_state_(this->alarm_dchocp_binary_sensor_, this->check_bit_of_byte_(data[135], 5));
    this->publish_alarm_state_(this->alarm_dchscp_binary_sensor_, this->check_bit_of_byte_(data[135], 6));
    this->publish_alarm_state_(this->alarm_dchotp_binary_sensor_, this->check_bit_of_byte_(data[135], 7));

  }

  if (frame_version == FRAME_VERSION_JK02_32S) {
    this->publish_alarm_state_(this->alarm_chargemos_binary_sensor_, this->check_bit_of_byte_(data[136], 0));
    this->publish_alarm_state_(this->alarm_dischargemos_binary_sensor_, this->check_bit_of_byte_(data[136], 1));
    this->publish_alarm_state_(this->alarm_gpsdisconneted_binary_sensor_, this->check_bit_of_byte_(data[136], 2));
    this->publish_alarm_state_(this->alarm_modifypwdintime_binary_sensor_, this->check_bit_of_byte_(data[136], 3));
    this->publish_alarm_state_(this->alarm_dischargeonfailed_binary_sensor_, this->check_bit_of_byte_(data[136], 4));
    this->publish_alarm_state_(this->alarm_batteryovertemp_binary_sensor_, this->check_bit_of_byte_(data[136], 5));
    this->publish_alarm_state_(this->alarm_temperaturesensoranomaly_binary_sensor_, this->check_bit_of_byte_(data[136], 6));
    this->publish_alarm_state_(this->alarm_plcmoduleanomaly_binary_sensor_, this->check_bit_of_byte_(data[136], 7));
  }

  if (frame_version == FRAME_VERSION_JK02_32S) {

    uint32_t raw_errors_bitmask = (uint32_t(data[134 + 3 + offset])<<24) | (uint32_t(data[134 + 2 + offset])<<16) | (uint32_t(data[134 + 1 + offset])<<8) | (uint32_t(data[134 + 0 + offset])<<0);

    this->publish_state_(this->errors_bitmask_sensor_, (float) raw_errors_bitmask);
    this->publish_state_(this->errors_text_sensor_, this->error_bits_to_string_(raw_errors_bitmask));
  }

  this->publish_state_(this->balancing_current_sensor_, int16_to_float(&data[138+offset]) * 0.001f);

  this->publish_state_(this->balancing_direction_sensor_, (data[140 + offset]));
  if (data[140 + offset] == 1 or data[140 + offset] == 2) {
    this->publish_state_(this->status_balancing_binary_sensor_, (bool) 1);
  } else {
    this->publish_state_(this->status_balancing_binary_sensor_, (bool) 0);
  }

  float soc = (float) data[141 + offset];
  this->publish_state_(this->battery_capacity_state_of_charge_sensor_, soc);
  this->soc_ok_ = !std::isnan(soc) && !std::isinf(soc) && soc >= 0.0f && soc <= 100.0f;
  this->cellinfo_ok_ = true;
  this->last_cellinfo_ms_ = millis();

  this->publish_state_(this->battery_capacity_remaining_sensor_, int32_to_float(&data[142+offset]) * 0.001f);

  this->publish_state_(this->battery_capacity_setting_sensor_, int32_to_float(&data[146+offset]) * 0.001f);

  this->publish_state_(this->charging_cycles_sensor_, uint32_to_float(&data[150+offset]));

  this->publish_state_(this->battery_capacity_total_charging_cycle_sensor_, uint32_to_float(&data[154+offset])*0.001f);

  temp_param_value=uint32_to_float(&data[158+offset]);

  this->publish_state_(this->battery_soh_valuation_sensor_, temp_param_value);

  this->publish_state_(this->status_precharging_binary_sensor_, this->check_bit_of_byte_(data[159 + offset], 0));

  temp_param_value=uint32_to_float(&data[162+offset]);

  this->publish_state_(this->battery_total_runtime_sensor_, temp_param_value);

  this->publish_state_(this->total_runtime_formatted_text_sensor_, format_total_runtime_(temp_param_value));

  this->publish_state_(this->status_charging_binary_sensor_, this->check_bit_of_byte_(data[166 + offset], 0));

  this->publish_state_(this->status_discharging_binary_sensor_, this->check_bit_of_byte_(data[167 + offset], 0));

  this->publish_state_(this->discharging_overcurrent_protection_release_time_sensor_, uint16_to_float(&data[170+offset]));

  this->publish_state_(this->discharging_short_circuit_protection_release_time_sensor_, uint16_to_float(&data[172+offset]));

  this->publish_state_(this->charging_overcurrent_protection_release_time_sensor_, uint16_to_float(&data[174+offset]));

  this->publish_state_(this->charging_short_circuit_protection_release_time_sensor_, uint16_to_float(&data[176+offset]));

  this->publish_state_(this->cell_undervoltage_protection_release_time_sensor_, uint16_to_float(&data[178+offset]));

  this->publish_state_(this->cell_overvoltage_protection_release_time_sensor_, uint16_to_float(&data[180+offset]));

  this->publish_state_(this->status_heating_binary_sensor_, this->check_bit_of_byte_(data[183 + offset], 0));

  temp_param_value = uint16_to_float(&data[186+offset]);
  this->publish_state_(this->emergency_time_countdown_sensor_, temp_param_value);

  this->publish_state_(this->heating_current_sensor_, int16_to_float(&data[204+offset])  * 0.001f);

  this->publish_state_(this->temperatures_[2].temperature_sensor_, int16_to_float(&data[222+offset]) * 0.1f);
  this->publish_state_(this->temperatures_[3].temperature_sensor_, int16_to_float(&data[224+offset]) * 0.1f);
  this->publish_state_(this->temperatures_[4].temperature_sensor_, int16_to_float(&data[226+offset]) * 0.1f);

  if (frame_version == FRAME_VERSION_JK02_32S) {

    this->publish_alarm_state_(this->alarm_mostempsensorabsent_binary_sensor_,  !this->check_bit_of_byte_(data[(182 + offset)], 0));
    this->publish_alarm_state_(this->alarm_battempsensor1absent_binary_sensor_, !this->check_bit_of_byte_(data[(182 + offset)], 1));
    this->publish_alarm_state_(this->alarm_battempsensor2absent_binary_sensor_, !this->check_bit_of_byte_(data[(182 + offset)], 2));
    this->publish_alarm_state_(this->alarm_battempsensor3absent_binary_sensor_, !this->check_bit_of_byte_(data[(182 + offset)], 3));
    this->publish_alarm_state_(this->alarm_battempsensor4absent_binary_sensor_, !this->check_bit_of_byte_(data[(182 + offset)], 4));
    this->publish_alarm_state_(this->alarm_battempsensor5absent_binary_sensor_, !this->check_bit_of_byte_(data[(182 + offset)], 5));

  }

  if (frame_version == FRAME_VERSION_JK02_32S) {
    this->publish_state_(this->battery_total_alarms_count_sensor_, (float) this->battery_total_alarms_count_);
    this->publish_state_(this->battery_total_alarms_active_sensor_, (float) this->battery_total_alarms_active_);
  }
  this->status_notification_received_ = true;
  this->trigger_bms2sniffer_event("WORKING ! #####",02);
}

void JkRS485Bms::decode_jk02_settings_(const std::vector<uint8_t> &data) {

  const size_t min_len = 287;
  if (data.size() < min_len) {
    ESP_LOGW(TAG, "Settings frame too short (%d bytes, need >= %d) - ignoring", data.size(), min_len);
    return;
  }

  ESP_LOGI(TAG, "Decoding settings  frame.... [ADDRESS: %02X] %d bytes received", this->address_, data.size());

  float temp_param_value;

  temp_param_value = uint32_to_float(&data[6]) * 0.001f;

  temp_param_value = uint32_to_float(&data[10]) * 0.001f;

  temp_param_value = uint32_to_float(&data[14]) * 0.001f;

  temp_param_value = uint32_to_float(&data[18]) * 0.001f;

  temp_param_value = uint32_to_float(&data[22]) * 0.001f;

  temp_param_value = uint32_to_float(&data[26]) * 0.001f;

  temp_param_value = uint32_to_float(&data[30]) * 0.001f;

  temp_param_value = uint32_to_float(&data[34]) * 0.001f;

  temp_param_value = uint32_to_float(&data[38]) * 0.001f;

  temp_param_value = uint32_to_float(&data[42]) * 0.001f;

  temp_param_value = uint32_to_float(&data[46]) * 0.001f;

  temp_param_value = uint32_to_float(&data[50]) * 0.001f;

  temp_param_value = uint32_to_float(&data[54]);

  temp_param_value = uint32_to_float(&data[58]);

  temp_param_value = uint32_to_float(&data[62]) * 0.001f;

  temp_param_value = uint32_to_float(&data[66]);

  temp_param_value = uint32_to_float(&data[70]);

  temp_param_value = uint32_to_float(&data[74]);

  temp_param_value = uint32_to_float(&data[78]) * 0.001f;

  temp_param_value=int32_to_float(&data[82])*0.1f;

  temp_param_value=int32_to_float(&data[86])*0.1f;

  temp_param_value=int32_to_float(&data[90])*0.1f;

  temp_param_value=int32_to_float(&data[94])*0.1f;

  temp_param_value=int32_to_float(&data[98])*0.1f;

  temp_param_value=int32_to_float(&data[102])*0.1f;

  temp_param_value=int32_to_float(&data[106])*0.1f;

  this->publish_state_(this->powertube_temperature_protection_sensor_, temp_param_value);

  temp_param_value=int32_to_float(&data[110])*0.1f;
  this->publish_state_(this->powertube_temperature_protection_recovery_sensor_, temp_param_value);

  temp_param_value=uint32_to_float(&data[114]);
  this->cell_count_settings_value_ = temp_param_value;

  ESP_LOGI(TAG, "  Balancer switch: %s", ((bool) data[126]) ? "on" : "off");

  temp_param_value=uint32_to_float(&data[134])*0.001f;

  temp_param_value=uint32_to_float(&data[138])*0.001f;

  temp_param_value=uint32_to_float(&data[274]);
  ESP_LOGI(TAG, "  Precharging time from discharged: %f s", temp_param_value);

  this->publish_state_(this->smart_sleep_time_sensor_, (uint8_t) (data[286]));

  this->settings_ok_ = true;
  this->last_settings_ms_ = millis();
  this->cellinfo_ok_ = false;
  this->voltage_ok_ = false;
  this->soc_ok_ = false;
  this->trigger_bms2sniffer_event("WORKING ! #####",01);
}

void JkRS485Bms::setup() {

  this->offline_published_ = true;
  this->publish_state_(this->status_online_binary_sensor_, false);
  this->publish_state_(this->errors_text_sensor_, "Offline");
}

void JkRS485Bms::update() { this->track_status_online_(); }

void JkRS485Bms::decode_device_info_(const std::vector<uint8_t> &data) {

  const size_t min_len = 268;
  if (data.size() < min_len) {
    ESP_LOGW(TAG, "Device info frame too short (%d bytes, need >= %d) - ignoring", data.size(), min_len);
    return;
  }

  ESP_LOGI(TAG, "Device info frame (%d bytes) received", data.size());
  ESP_LOGVV(TAG, "  %s", format_hex_pretty(&data.front(), 160).c_str());
  ESP_LOGVV(TAG, "  %s", format_hex_pretty(&data.front() + 160, data.size() - 160).c_str());

  this->publish_state_(this->info_vendorid_text_sensor_, std::string(data.begin() + 6, data.begin() + 6 + 16).c_str());
  this->publish_state_(this->info_hardware_version_text_sensor_, std::string(data.begin() + 22, data.begin() + 22 + 8).c_str());
  this->publish_state_(this->info_software_version_text_sensor_, std::string(data.begin() + 30, data.begin() + 30 + 8).c_str());
  this->publish_state_(this->info_device_name_text_sensor_, std::string(data.begin() + 46, data.begin() + 46 + 16).c_str());
  this->publish_state_(this->info_device_password_text_sensor_, std::string(data.begin() + 62, data.begin() + 62 + 16).c_str());
  this->publish_state_(this->info_device_serial_number_text_sensor_, std::string(data.begin() + 86, data.begin() + 86 + 11).c_str());
  this->publish_state_(this->info_device_setup_passcode_text_sensor_, std::string(data.begin() + 118, data.begin() + 118 + 16).c_str());

  this->publish_state_(this->uart1_protocol_number_sensor_, (uint8_t) data[178]);
  this->publish_state_(this->uart2_protocol_number_sensor_, (uint8_t) data[212]);

  this->trigger_bms2sniffer_event("WORKING ! #####",03);
}

void JkRS485Bms::track_status_online_() {
  if (this->last_response_ms_ == 0) {
    return;
  }

  const uint32_t now = millis();
  if ((now - this->last_response_ms_) > OFFLINE_TIMEOUT_MS && !this->offline_published_) {
    ESP_LOGI(TAG, "===== [BMS 0x%02X] ONLINE -> OFFLINE =====", this->address_);
    this->publish_device_unavailable_();
    this->offline_published_ = true;
  }
}

void JkRS485Bms::reset_status_online_tracker_() {
  this->no_response_count_ = 0;
  this->offline_published_ = false;
  ESP_LOGI(TAG, "===== [BMS 0x%02X] OFFLINE -> ONLINE | V=%f SOC=%f cells_real=%f cells_cfg=%f =====",
           this->address_,
           this->battery_voltage_sensor_ ? this->battery_voltage_sensor_->state : NAN,
           this->battery_capacity_state_of_charge_sensor_ ? this->battery_capacity_state_of_charge_sensor_->state : NAN,
           this->cell_count_real_sensor_ ? this->cell_count_real_sensor_->state : NAN,
           this->cell_count_settings_value_);
  this->publish_state_(this->status_online_binary_sensor_, true);
}

void JkRS485Bms::publish_device_unavailable_() {

    this->publish_state_(status_online_binary_sensor_, false);
    this->publish_state_(status_balancing_binary_sensor_, false);
    this->publish_state_(status_precharging_binary_sensor_, false);
    this->publish_state_(status_charging_binary_sensor_, false);
    this->publish_state_(status_discharging_binary_sensor_, false);
    this->publish_state_(status_heating_binary_sensor_, false);

    this->publish_state_(alarm_wireres_binary_sensor_, false);
    this->publish_state_(alarm_mosotp_binary_sensor_, false);
    this->publish_state_(alarm_cellquantity_binary_sensor_, false);
    this->publish_state_(alarm_cursensorerr_binary_sensor_, false);
    this->publish_state_(alarm_cellovp_binary_sensor_, false);
    this->publish_state_(alarm_batovp_binary_sensor_, false);
    this->publish_state_(alarm_chocp_binary_sensor_, false);
    this->publish_state_(alarm_chscp_binary_sensor_, false);
    this->publish_state_(alarm_chotp_binary_sensor_, false);
    this->publish_state_(alarm_chutp_binary_sensor_, false);
    this->publish_state_(alarm_cpuauxcommuerr_binary_sensor_, false);
    this->publish_state_(alarm_celluvp_binary_sensor_, false);
    this->publish_state_(alarm_batuvp_binary_sensor_, false);
    this->publish_state_(alarm_dchocp_binary_sensor_, false);
    this->publish_state_(alarm_dchscp_binary_sensor_, false);
    this->publish_state_(alarm_dchotp_binary_sensor_, false);
    this->publish_state_(alarm_chargemos_binary_sensor_, false);
    this->publish_state_(alarm_dischargemos_binary_sensor_, false);
    this->publish_state_(alarm_gpsdisconneted_binary_sensor_, false);
    this->publish_state_(alarm_modifypwdintime_binary_sensor_, false);
    this->publish_state_(alarm_dischargeonfailed_binary_sensor_, false);
    this->publish_state_(alarm_batteryovertemp_binary_sensor_, false);
    this->publish_state_(alarm_temperaturesensoranomaly_binary_sensor_, false);
    this->publish_state_(alarm_plcmoduleanomaly_binary_sensor_, false);
    this->publish_state_(alarm_mostempsensorabsent_binary_sensor_, false);
    this->publish_state_(alarm_battempsensor1absent_binary_sensor_, false);
    this->publish_state_(alarm_battempsensor2absent_binary_sensor_, false);
    this->publish_state_(alarm_battempsensor3absent_binary_sensor_, false);
    this->publish_state_(alarm_battempsensor4absent_binary_sensor_, false);
    this->publish_state_(alarm_battempsensor5absent_binary_sensor_, false);

  this->settings_ok_ = false;
  this->cellinfo_ok_ = false;
  this->voltage_ok_ = false;
  this->soc_ok_ = false;
  this->last_settings_ms_ = 0;
  this->last_cellinfo_ms_ = 0;

  this->cell_count_settings_value_ = 0;

  this->publish_state_(status_online_binary_sensor_, false);
  this->publish_state_(errors_text_sensor_, "Offline");
  this->publish_state_(cell_count_real_sensor_, NAN);
  this->publish_state_(cell_voltage_min_sensor_, NAN);
  this->publish_state_(cell_voltage_max_sensor_, NAN);
  this->publish_state_(cell_voltage_min_cell_number_sensor_, NAN);
  this->publish_state_(cell_voltage_max_cell_number_sensor_, NAN);
  this->publish_state_(cell_resistance_min_sensor_, NAN);
  this->publish_state_(cell_resistance_max_sensor_, NAN);
  this->publish_state_(cell_resistance_min_cell_number_sensor_, NAN);
  this->publish_state_(cell_resistance_max_cell_number_sensor_, NAN);
  this->publish_state_(battery_capacity_state_of_charge_sensor_, NAN);
  this->publish_state_(battery_soh_valuation_sensor_, NAN);
  this->publish_state_(balancing_current_sensor_, NAN);
  this->publish_state_(balancing_direction_sensor_, NAN);
  this->publish_state_(cell_delta_voltage_sensor_, NAN);
  this->publish_state_(cell_average_voltage_sensor_, NAN);
  this->publish_state_(temperature_powertube_sensor_, NAN);
  this->publish_state_(temperature_sensor_1_sensor_, NAN);
  this->publish_state_(temperature_sensor_2_sensor_, NAN);
  this->publish_state_(battery_voltage_sensor_, NAN);
  this->publish_state_(battery_current_sensor_, NAN);
  this->publish_state_(battery_power_sensor_, NAN);
  this->publish_state_(battery_power_charging_sensor_, NAN);
  this->publish_state_(battery_power_discharging_sensor_, NAN);
  this->publish_state_(battery_capacity_setting_sensor_, NAN);
  this->publish_state_(battery_capacity_remaining_sensor_, NAN);
  this->publish_state_(battery_capacity_remaining_derived_sensor_, NAN);
  this->publish_state_(temperature_sensors_sensor_, NAN);
  this->publish_state_(charging_cycles_sensor_, NAN);
  this->publish_state_(battery_capacity_total_charging_cycle_sensor_, NAN);
  this->publish_state_(battery_strings_sensor_, NAN);
  this->publish_state_(errors_bitmask_sensor_, NAN);
  this->publish_state_(operation_mode_bitmask_sensor_, NAN);
  this->publish_state_(cell_voltage_overvoltage_delay_sensor_, NAN);

  this->publish_state_(cell_voltage_undervoltage_delay_sensor_, NAN);
  this->publish_state_(cell_pressure_difference_protection_sensor_, NAN);
  this->publish_state_(discharging_overcurrent_protection_sensor_, NAN);
  this->publish_state_(discharging_overcurrent_delay_sensor_, NAN);
  this->publish_state_(charging_overcurrent_protection_sensor_, NAN);
  this->publish_state_(charging_overcurrent_delay_sensor_, NAN);
  this->publish_state_(balancing_opening_pressure_difference_sensor_, NAN);
  this->publish_state_(powertube_temperature_protection_sensor_, NAN);
  this->publish_state_(powertube_temperature_protection_recovery_sensor_, NAN);
  this->publish_state_(temperature_sensor_temperature_protection_sensor_, NAN);
  this->publish_state_(temperature_sensor_temperature_recovery_sensor_, NAN);
  this->publish_state_(temperature_sensor_temperature_difference_protection_sensor_, NAN);
  this->publish_state_(charging_high_temperature_protection_sensor_, NAN);
  this->publish_state_(discharging_high_temperature_protection_sensor_, NAN);
  this->publish_state_(charging_low_temperature_protection_sensor_, NAN);
  this->publish_state_(charging_low_temperature_recovery_sensor_, NAN);
  this->publish_state_(discharging_low_temperature_protection_sensor_, NAN);
  this->publish_state_(discharging_low_temperature_recovery_sensor_, NAN);
  this->publish_state_(battery_capacity_total_setting_sensor_, NAN);
  this->publish_state_(charging_sensor_, NAN);
  this->publish_state_(discharging_sensor_, NAN);
  this->publish_state_(current_calibration_sensor_, NAN);
  this->publish_state_(device_address_sensor_, NAN);
  this->publish_state_(sleep_wait_time_sensor_, NAN);
  this->publish_state_(alarm_low_volume_sensor_, NAN);
  this->publish_state_(password_sensor_, NAN);
  this->publish_state_(manufacturing_date_sensor_, NAN);
  this->publish_state_(battery_total_runtime_sensor_, NAN);
  this->publish_state_(total_runtime_formatted_text_sensor_, "NAN");

  this->publish_state_(start_current_calibration_sensor_, NAN);
  this->publish_state_(actual_battery_capacity_sensor_, NAN);
  this->publish_state_(protocol_version_sensor_, NAN);

  for (auto &cell : this->cells_) {
    this->publish_state_(cell.cell_voltage_sensor_, NAN);
  }

}

void JkRS485Bms::publish_state_(binary_sensor::BinarySensor *binary_sensor, const bool &state) {
  if (binary_sensor == nullptr)
    return;

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
  if (text_sensor == nullptr){
    ESP_LOGVV(TAG, "Object is nullptr");
    return;
  }

  text_sensor->publish_state(state);
}

void JkRS485Bms::publish_alarm_state_(binary_sensor::BinarySensor *binary_sensor, const bool &state) {
  if (binary_sensor == nullptr) {
    ESP_LOGVV(TAG, "Object is nullptr");
    return;
  }
  battery_total_alarms_count_++;
  if (state) {
    battery_total_alarms_active_++;
  }
  binary_sensor->publish_state(state);
}
std::string JkRS485Bms::error_bits_to_string_(const uint32_t mask) {
  bool first = true;
  std::string errors_list = "";

  if (mask) {
    for (int i = 0; i < ERRORS_SIZE; i++) {
      if (mask & (1 << i)) {
        if (first) {
          first = false;
        } else {
          errors_list.append(";");
        }
        errors_list.append(ERRORS[i]);
      }
    }
  }

  return errors_list;
}

std::string JkRS485Bms::mode_bits_to_string_(const uint16_t mask) {
  bool first = true;
  std::string modes_list = "";

  if (mask) {
    for (int i = 0; i < OPERATION_MODES_SIZE; i++) {
      if (mask & (1 << i)) {
        if (first) {
          first = false;
        } else {
          modes_list.append(";");
        }
        modes_list.append(OPERATION_MODES[i]);
      }
    }
  }

  return modes_list;
}

void JkRS485Bms::dump_config() {

}

}
}
