#include "jk_rs485_sniffer.h"

namespace esphome {
namespace jk_rs485_sniffer {

static const char *const TAG = "jk_rs485_sniffer";

static const uint16_t JKPB_RS485_MASTER_SHORT_REQUEST_SIZE = 8;
static const uint16_t JKPB_RS485_MASTER_REQUEST_SIZE = 11;
static const uint16_t JKPB_RS485_RESPONSE_SIZE = 308;

static const uint16_t JKPB_RS485_NUMBER_OF_ELEMENTS_TO_COMPUTE_CHECKSUM = 299;
static const uint16_t JKPB_RS485_FRAME_TYPE_ADDRESS = 4;
static const uint16_t JKPB_RS485_FRAME_TYPE_ADDRESS_FOR_FRAME_TYPE_x01 = 264;
static const uint16_t JKPB_RS485_FRAME_COUNTER_ADDRESS = 5;
static const uint16_t JKPB_RS485_CHECKSUM_INDEX = 299;
static const uint16_t JKPB_RS485_ADDRESS_OF_RS485_ADDRESS = 300;

// Maximum time between a live data frame and the settings frame of the same BMS that follows it
static const uint32_t MAX_DELAY_LIVE_FRAME_TO_SETTINGS_FRAME_MILLISECONDS = 600;

// Maximum time between a request and the response that belongs to it
static const uint32_t MAX_RESPONSE_DELAY_AFTER_REQUEST_MILLISECONDS = 1000;

// Request function code (byte 3 of a request) -> frame type of the response; 0 if unknown
static uint8_t request_code_to_frame_type(uint8_t code) {
  switch (code) {
    case 0x1E:
      return 1;  // device settings
    case 0x20:
      return 2;  // cell info
    case 0x1C:
      return 3;  // device info
    default:
      return 0;
  }
}

static const uint16_t MIN_SILENCE_MILLISECONDS = 150;
static const uint16_t MIN_SILENCE_NEEDED_BEFORE_SPEAKING_MILLISECONDS = 250;

static const uint32_t TIME_BETWEEN_CELL_INFO_REQUESTS_MILLISECONDS       =  5000;
static const uint32_t TIME_BETWEEN_DEVICE_SETTINGS_REQUESTS_MILLISECONDS = 10000;
static const uint32_t TIME_BETWEEN_DEVICE_INFO_REQUESTS_MILLISECONDS     = 3600000;

static const uint16_t SILENCE_BEFORE_ACTING_AS_MASTER = 2000;
static const uint16_t SILENCE_BEFORE_REUSING_NETWORK_ACTING_AS_MASTER=400;
static const uint16_t TIME_BEFORE_NEXT_POOLING_MILLISECONDS=750;
static const uint16_t TIME_BETWEEN_CONSECUTIVE_REQUEST_SENDINGS_TO_SAME_SLAVE=2500;

static const uint16_t TIME_BETWEEN_NETWORK_SCAN_MILLISECONDS=500;
static const uint16_t NO_MESSAGE_RECEIVED_TIME_SET_AS_UNAVAILABLE_MILLISECONDS = 10000;

enum RxParseResult : uint8_t {
  RX_NO_DATA = 0,
  RX_PARSED_SHORT = 1,
  RX_PARSED_REQUEST = 2,
  RX_PARSED_RESPONSE = 3,
  RX_NEED_MORE = 4,
  RX_RESYNC = 5,
  RX_CHECKSUM_FAIL = 6,
};

static const char *rx_result_to_string(uint8_t value) {
  switch (value) {
    case RX_NO_DATA:
      return "no_data";
    case RX_PARSED_SHORT:
      return "parsed_short";
    case RX_PARSED_REQUEST:
      return "parsed_request";
    case RX_PARSED_RESPONSE:
      return "parsed_response";
    case RX_NEED_MORE:
      return "need_more";
    case RX_RESYNC:
      return "resync";
    case RX_CHECKSUM_FAIL:
      return "checksum_fail";
    default:
      return "unknown";
  }   
}

std::vector<unsigned char> pattern_response_header = {0x55, 0xAA, 0xEB, 0x90};

static const uint16_t CRC16_MODBUS_POLY_REVERSED = 0xA001;

uint16_t crc16_c(const uint8_t data[], const uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if ((crc & 0x1) == 1) {
                crc = (crc >> 1) ^ CRC16_MODBUS_POLY_REVERSED;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return (((crc & 0x00FF)<<8) | ((crc & 0xFF00))>>8);
}

uint16_t chksum(const uint8_t data[], const uint16_t len) {
  uint16_t checksum = 0;
  for (uint16_t i = 0; i < len; i++) {
    checksum = checksum + data[i];
  }
  return checksum;
}

bool JkRS485Sniffer::accept_response_(uint8_t address, uint8_t frame_type, const std::vector<uint8_t> &data,
                                      uint32_t now) {
  if (address <= 15) {
    this->rs485_network_node[address].last_message_received = now;
  }
  if (address == 0) {
    last_master_activity = now;
  } else if (address > 15) {
    ESP_LOGW(TAG, "Response type 0x%02X DROPPED: address 0x%02X is out of range (max 15)", frame_type, address);
    return false;
  } else {
    this->set_node_availability(address, 1);
  }

  this->rx_frames_ok_++;
  ESP_LOGD(TAG, "Frame received from SLAVE (type: 0x%02X, %d bytes) %02X address", frame_type, (int) data.size(), address);
  ESP_LOGVV(TAG, "  %s", format_hex_pretty(&data.front(), data.size()).c_str());

  bool found = false;
  for (auto *device : this->devices_) {
    device->on_jk_rs485_sniffer_data(address, frame_type, data, this->nodes_available);
    found = true;
  }

  if (!found) {
    ESP_LOGW(TAG, "Got JkRS485 but no recipients to send [frame type:0x%02X] 0x%02X!", frame_type, address);
  }
  return true;
}

void JkRS485Sniffer::resolve_pending_live_frame_(int16_t address_of_following_settings_frame, uint32_t now) {
  if (!this->pending_live_frame_.valid) {
    return;
  }
  this->pending_live_frame_.valid = false;

  uint8_t address = this->pending_live_frame_.address_in_frame;
  if (address_of_following_settings_frame >= 0 &&
      (now - this->pending_live_frame_.received_ms) <= MAX_DELAY_LIVE_FRAME_TO_SETTINGS_FRAME_MILLISECONDS) {
    if (address != (uint8_t) address_of_following_settings_frame) {
      ESP_LOGI(TAG, "Live data frame (type 0x02) re-attributed from address 0x%02X to 0x%02X: the settings frame that follows it is from 0x%02X",
               address, (uint8_t) address_of_following_settings_frame, (uint8_t) address_of_following_settings_frame);
    }
    address = (uint8_t) address_of_following_settings_frame;
  }
  this->accept_response_(address, 0x02, this->pending_live_frame_.data, now);
  this->pending_live_frame_.data.clear();
}

void JkRS485Sniffer::remember_request_(uint8_t address, uint8_t frame_type) {
  if (frame_type == 0) {
    this->last_request_address_ = -1;
    return;
  }
  this->last_request_address_ = address;
  this->last_request_frame_type_ = frame_type;
  this->last_request_ms_ = millis();
}

void JkRS485Sniffer::handle_bms2sniffer_event(std::uint8_t slave_address, std::string event, std::uint8_t frame_type){

  ESP_LOGD(TAG,"Received Event from BMS.. [address:0x%02X] @ %d -->  %s", slave_address, frame_type, event.c_str());
  const uint32_t now=millis();

  if (slave_address > 15) {
    ESP_LOGE(TAG, "slave_address 0x%02X out of range (max 15), ignoring event", slave_address);
    return;
  }

  if (frame_type==1){
    this->rs485_network_node[slave_address].last_device_settings_request_received_OK=now;
    this->rs485_network_node[slave_address].counter_device_settings_received++;
    ESP_LOGD(TAG, "updated last_device_settings_request_received_OK");
  } else if (frame_type==2){
    this->rs485_network_node[slave_address].last_cell_info_request_received_OK=now;
    this->rs485_network_node[slave_address].counter_cell_info_received++;
    ESP_LOGD(TAG, "updated last_cell_info_request_received_OK");
  } else if (frame_type==3){
    this->rs485_network_node[slave_address].last_device_info_request_received_OK=now;
    this->rs485_network_node[slave_address].counter_device_info_received++;
    ESP_LOGD(TAG, "updated last_device_info_request_received_OK");
  } else {

  }

  this->last_jk_rs485_network_activity_=now;
  if (this->act_as_master==true){
    this->last_message_received_acting_as_master=now;
  }
}

void JkRS485Sniffer::send_request_to_slave(uint8_t address, uint8_t frame_type){

    uint8_t frame[11];
    frame[0] = address ;
    frame[1] = 0x10;
    frame[2] = 0x16;

    if (frame_type==1){
      frame[3] = 0x1E;
    } else if (frame_type==2){
      frame[3] = 0x20;
    } else if (frame_type==3){
      frame[3] = 0x1C;
    } else {
      return;
    }
    frame[4] = 0x00;
    frame[5] = 0x01;
    frame[6] = 0x02;
    frame[7] = 0x00;
    frame[8] = 0x00;

    uint16_t computed_checksum = crc16_c(frame, 9);
    frame[9] = ((computed_checksum & 0xFF00)>>8);
    frame[10] = ((computed_checksum & 0x00FF)>>0);

    ESP_LOGV(TAG, "MESSAGE REQUEST TO SEND>>: %s",format_hex_pretty(frame, 11).c_str());

    std::vector<uint8_t> data_to_send(frame, frame + sizeof(frame) / sizeof(frame[0]));

    if (talk_pin_needed_) { this->talk_pin_->digital_write(1); }
    delayMicroseconds(50);
    this->write_array(data_to_send);
    this->flush();
    if (talk_pin_needed_) { this->talk_pin_->digital_write(0); }
    delayMicroseconds(50);

    this->remember_request_(address, frame_type);

    const uint32_t now=millis();
    this->rs485_network_node[address].last_request_sent=now;
    this->last_jk_rs485_network_activity_=now;

}

bool JkRS485Sniffer::decide_next_frame_type(uint8_t node, uint32_t now) {
  auto &n = this->rs485_network_node[node];
  if (now - n.last_device_info_request_received_OK > TIME_BETWEEN_DEVICE_INFO_REQUESTS_MILLISECONDS || n.last_device_info_request_received_OK == 0) {
    this->pooling_index.frame_type = 3;
    return true;
  }
  if (now - n.last_device_settings_request_received_OK > TIME_BETWEEN_DEVICE_SETTINGS_REQUESTS_MILLISECONDS || n.last_device_settings_request_received_OK == 0) {
    this->pooling_index.frame_type = 1;
    return true;
  }
  if (now - n.last_cell_info_request_received_OK > TIME_BETWEEN_CELL_INFO_REQUESTS_MILLISECONDS || n.last_cell_info_request_received_OK == 0) {
    this->pooling_index.frame_type = 2;
    return true;
  }
  return false;
}

bool JkRS485Sniffer::calculate_next_pooling(void){

  bool found=false;

  const uint32_t now=millis();

  if (this->rs485_network_node[this->pooling_index.node_address].available && (now-this->rs485_network_node[this->pooling_index.node_address].last_request_sent)>TIME_BETWEEN_CONSECUTIVE_REQUEST_SENDINGS_TO_SAME_SLAVE){
    found = this->decide_next_frame_type(this->pooling_index.node_address, now);
  }

  if (found==false){

    uint8_t found_index=0;
    for (uint8_t j = this->pooling_index.node_address+1; j < 16; ++j) {
      if (rs485_network_node[j].available && (now-this->rs485_network_node[j].last_request_sent)>TIME_BETWEEN_CONSECUTIVE_REQUEST_SENDINGS_TO_SAME_SLAVE) {
        found = this->decide_next_frame_type(j, now);
      }
      if (found==true){
        found_index=j;
        break;
      }
    }

    if (found==false){
      for (uint8_t j = 1; j <= this->pooling_index.node_address; ++j) {
        if (rs485_network_node[j].available && (now-this->rs485_network_node[j].last_request_sent)>TIME_BETWEEN_CONSECUTIVE_REQUEST_SENDINGS_TO_SAME_SLAVE) {
          found = this->decide_next_frame_type(j, now);
        }
        if (found==true){
          found_index=j;
          break;
        }
      }
    }

    if (found==true){
      this->pooling_index.node_address=found_index;
    } else {

    }
  }

  if (found==true){
    const uint32_t now=millis();
    ESP_LOGI(TAG, "POOLING NEXT AVAILABLE... [address:0x%02X] @ %d [%u,%u,%u]",this->pooling_index.node_address,this->pooling_index.frame_type,
                                                          now-this->rs485_network_node[this->pooling_index.node_address].last_device_settings_request_received_OK,
                                                          now-this->rs485_network_node[this->pooling_index.node_address].last_cell_info_request_received_OK,
                                                          now-this->rs485_network_node[this->pooling_index.node_address].last_device_info_request_received_OK);
  } else {

  }

  return(found);
}

int JkRS485Sniffer::found_next_node_to_discover(void){
  int found_index=-1;
  for (uint8_t j = this->pooling_index.scan_address+1; j < 16; ++j) {
      if (!rs485_network_node[j].available) {
          found_index=j;
          break;
      }
  }
  if (found_index==-1){
    for (uint8_t j = 1; j <= this->pooling_index.scan_address; ++j) {
        if (!rs485_network_node[j].available) {
            found_index=j;
            break;
        }
    }
  }
  return(found_index);
}

void JkRS485Sniffer::loop() {
  uint32_t now = millis();

  if (this->pending_live_frame_.valid &&
      (now - this->pending_live_frame_.received_ms) > MAX_DELAY_LIVE_FRAME_TO_SETTINGS_FRAME_MILLISECONDS) {
    this->resolve_pending_live_frame_(-1, now);  // no settings frame followed
  }

  if (this->rx_buffer_.size()>=RX_BUFFER_MAX_SIZE){
    ESP_LOGW(TAG, "### Buffer cleared buffer size: %d",this->rx_buffer_.size());
    this->rx_buffer_.clear();
  }

  if (this->available()){
    if ((now-this->last_jk_rs485_network_activity_)>MIN_SILENCE_MILLISECONDS){
      if (this->act_as_master==false){
        ESP_LOGD(TAG, "SILENCE: %f ms",(float)(now-this->last_jk_rs485_network_activity_));
      } else {
        ESP_LOGI(TAG, "SILENCE: %f ms",(float)(now-this->last_jk_rs485_network_activity_));
      }

    }

    uint8_t byte;
    while (this->available() && (this->rx_buffer_.size()<RX_BUFFER_MAX_SIZE)) {
      this->read_byte(&byte);
      this->rx_buffer_.push_back(byte);
    }
    now = millis();
    this->last_jk_rs485_network_activity_ = now;

    uint8_t response=0;
    uint16_t original_buffer_size=rx_buffer_.size();
    uint8_t cont_manage=0;
    bool changed=true;
    ESP_LOGD(TAG, "..........................................");

    do {
        cont_manage++;
        ESP_LOGV(TAG, "Buffer before number %d:    %s",cont_manage,format_hex_pretty(&this->rx_buffer_.front(), this->rx_buffer_.size()).c_str());
        response=this->manage_rx_buffer_();
        ESP_LOGV(TAG, "Response:            %d (%s)",response, rx_result_to_string(response));
        if (original_buffer_size==rx_buffer_.size()){
           changed=false;
        } else {
           changed=true;
           original_buffer_size=this->rx_buffer_.size();
        }
    } while (cont_manage<5 && changed==true && original_buffer_size>=JKPB_RS485_MASTER_SHORT_REQUEST_SIZE);

    if (original_buffer_size==0){
      ESP_LOGV(TAG,     "Buffer empty");
    }

  } else {

    if ((now-this->last_jk_rs485_network_activity_)>MIN_SILENCE_NEEDED_BEFORE_SPEAKING_MILLISECONDS){

      if (now-last_master_activity>SILENCE_BEFORE_ACTING_AS_MASTER){
        if (this->act_as_master==false){

          this->act_as_master=true;
          this->set_node_availability(0,0);
          ESP_LOGI(TAG, "NO JK MASTER DETECTED IN THE NETWORK. ESP WILL ACT AS MASTER");
        }
      }

      if (this->act_as_master) {
        if (now-last_message_received_acting_as_master>SILENCE_BEFORE_REUSING_NETWORK_ACTING_AS_MASTER){

          this->last_message_received_acting_as_master=now;

          bool scan_sent=false;

          if (now-this->last_network_scan>TIME_BETWEEN_NETWORK_SCAN_MILLISECONDS){
            int found_index=-1;
            found_index=this->found_next_node_to_discover();

            if (found_index==-1){

              ESP_LOGD(TAG, "SCANNING TO DISCOVER...ALL NODES ARE AVAILABLE");
            } else {
              ESP_LOGD(TAG, "SCANNING TO DISCOVER...0x%02X [%s]",found_index,this->nodes_available_to_string().c_str());
              this->pooling_index.scan_address=found_index;
              this->send_request_to_slave(found_index,2);

              this->last_network_scan=now;
              scan_sent=true;
            }
          }

          if (scan_sent==false){
            if (this->nodes_available_number>0 && now-this->last_jk_rs485_pooling_trial_>TIME_BEFORE_NEXT_POOLING_MILLISECONDS){
              this->last_jk_rs485_pooling_trial_=now;

              if (this->calculate_next_pooling()==true){
                this->send_request_to_slave(this->pooling_index.node_address,this->pooling_index.frame_type);

              }
            }
          }
        }

      } else {

        for (uint8_t cont=0;cont<16;cont++){
          if (this->rs485_network_node[cont].available==true){

            if (now-rs485_network_node[cont].last_device_info_request_received_OK>TIME_BETWEEN_DEVICE_INFO_REQUESTS_MILLISECONDS){

              send_request_to_slave(cont,03);

              break;
            }

          }
        }

        for (uint8_t cont=0;cont<16;cont++){
          if (now-this->rs485_network_node[cont].last_message_received>NO_MESSAGE_RECEIVED_TIME_SET_AS_UNAVAILABLE_MILLISECONDS){
            this->set_node_availability(cont,0);
          }

          if (this->rs485_network_node[cont].available && cont>0){
            if (this->rs485_network_node[cont].counter_device_info_received==0){
              this->rs485_network_node[cont].last_device_info_request_received_OK=0;
            }
          }

        }

      }
    }
  }
}

std::string JkRS485Sniffer::nodes_available_to_string() {
    std::string bufferHex;
    bufferHex.reserve(17);
    uint8_t number_of_nodes_available=0;
    for (uint8_t cont = 0; cont < 16; cont++) {

        if (this->rs485_network_node[cont].available) {
            bufferHex.push_back('1');
            number_of_nodes_available++;
        } else {
            bufferHex.push_back('0');
        }
    }

    bufferHex.push_back('\0');
    this->nodes_available_number=number_of_nodes_available;
    return bufferHex;
}

void JkRS485Sniffer::set_node_availability(uint8_t address,bool value){

  if (address > 15) {
    ESP_LOGE(TAG, "address 0x%02X out of range (max 15), ignoring", address);
    return;
  }

  if (this->rs485_network_node[address].available==value){

  } else {
    uint8_t previous_value=this->rs485_network_node[address].available;
    this->rs485_network_node[address].available=value;

    std::string previous=this->nodes_available;
    this->nodes_available=this->nodes_available_to_string();
    ESP_LOGI(TAG, "NODES AVAILABLE CHANGED: address 0x%02X (%d->%d) [%s] --> [%s]",address,previous_value,value,previous.c_str(),this->nodes_available.c_str());
  }
}

void JkRS485Sniffer::detected_master_activity_now(void){
  const uint32_t now = millis();

  if (this->act_as_master){
    this->act_as_master=false;
    ESP_LOGI(TAG, "JK MASTER DETECTED IN THE NETWORK");
  }
  this->last_master_activity=now;
}

uint8_t JkRS485Sniffer::manage_rx_buffer_(void) {

  uint8_t address = 0;

  const uint32_t now = millis();
  const size_t response_size = JKPB_RS485_RESPONSE_SIZE;
  const size_t header_size = pattern_response_header.size();
  const size_t short_size = JKPB_RS485_MASTER_SHORT_REQUEST_SIZE;
  const size_t request_size = JKPB_RS485_MASTER_REQUEST_SIZE;
  uint8_t result = RX_NEED_MORE;

  if (this->rx_buffer_.empty()) {
    return(RX_NO_DATA);
  }

  auto erase_prefix = [&](size_t count) {
    if (count == 0) {
      return;
    }
    if (count >= this->rx_buffer_.size()) {
      this->rx_buffer_.clear();
      return;
    }
    this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + count);
  };

  auto preamble_it = std::search(this->rx_buffer_.begin(), this->rx_buffer_.end(),
                                  pattern_response_header.begin(), pattern_response_header.end());
  const bool preamble_found = (preamble_it != this->rx_buffer_.end());
  const size_t preamble_index =
      preamble_found ? std::distance(this->rx_buffer_.begin(), preamble_it) : this->rx_buffer_.size();

  auto try_parse_short_request = [&]() -> bool {
    if (this->rx_buffer_.size() < short_size) {
      return false;
    }
    if (preamble_found) {
      return false;
    }

    const uint8_t *raw = this->rx_buffer_.data();
    uint16_t computed_checksum = crc16_c(raw, 6);
    uint16_t remote_checksum = ((uint16_t(raw[6]) << 8) | (uint16_t(raw[7]) << 0));
    if (computed_checksum != remote_checksum) {
      this->rx_short_checksum_fail_++;
      ESP_LOGV(TAG, "CHECKSUM failed! 0x%04X != 0x%04X (short_fail=%u)", computed_checksum,
               remote_checksum, this->rx_short_checksum_fail_);
      result = RX_CHECKSUM_FAIL;
      return false;
    }

    address = raw[0];
    ESP_LOGD(TAG, "Answer received for MASTER (type: SHORT REQUEST for address %02X, %d bytes)", address,
             short_size - 1);
    erase_prefix(short_size - 1);
    result = RX_PARSED_SHORT;
    return true;
  };

  auto try_parse_request = [&]() -> bool {
    if (this->rx_buffer_.size() < request_size) {
      return false;
    }
    const bool try_with_master_request_size = !preamble_found || (preamble_index >= request_size);

    if (!try_with_master_request_size) {
      return false;
    }

    const uint8_t *raw = this->rx_buffer_.data();
    uint16_t computed_checksum = crc16_c(raw, 9);
    uint16_t remote_checksum = ((uint16_t(raw[9]) << 8) | (uint16_t(raw[10]) << 0));
    if (computed_checksum != remote_checksum) {
      this->rx_request_checksum_fail_++;
      ESP_LOGV(TAG, "CHECKSUM failed! 0x%04X != 0x%04X (req_fail=%u)", computed_checksum,
               remote_checksum, this->rx_request_checksum_fail_);
      result = RX_CHECKSUM_FAIL;
      return false;
    }

    address = raw[0];
    ESP_LOGI(TAG, "REAL master is speaking to address 0x%02X (request)", address);
    this->remember_request_(address, request_code_to_frame_type(raw[3]));
    this->rs485_network_node[0].last_message_received = now;
    this->detected_master_activity_now();
    this->set_node_availability(0, 1);
    erase_prefix(request_size);
    result = RX_PARSED_REQUEST;
    return true;
  };

  auto try_parse_response = [&]() -> bool {
    if (this->rx_buffer_.size() < response_size) {
      return false;
    }

    if (preamble_found) {
      if (preamble_index > 0) {
        erase_prefix(preamble_index);
        this->rx_preamble_drop_++;
        result = RX_RESYNC;
      }
      if (this->rx_buffer_.size() < response_size) {
        return true;
      }
    } else {

      size_t keep = (header_size > 0) ? header_size - 1 : 0;
      if (this->rx_buffer_.size() > keep) {
        erase_prefix(this->rx_buffer_.size() - keep);
      }
      this->rx_preamble_drop_++;
      ESP_LOGV(TAG, "No preamble found, dropping buffer (preamble_drop=%u)", this->rx_preamble_drop_);
      result = RX_RESYNC;
      return true;
    }

    const uint8_t *raw = this->rx_buffer_.data();
    uint8_t computed_checksum = chksum(raw, JKPB_RS485_NUMBER_OF_ELEMENTS_TO_COMPUTE_CHECKSUM);
    uint8_t remote_checksum = raw[JKPB_RS485_CHECKSUM_INDEX];

    if (raw[JKPB_RS485_FRAME_TYPE_ADDRESS]==1) {
      address = raw[JKPB_RS485_FRAME_TYPE_ADDRESS_FOR_FRAME_TYPE_x01 + 6];
    } else {
      address = raw[JKPB_RS485_ADDRESS_OF_RS485_ADDRESS];
    }

    if (computed_checksum != remote_checksum) {
      this->rx_response_checksum_fail_++;
      ESP_LOGW(TAG, "CHECKSUM failed! 0x%02X != 0x%02X (resp_fail=%u)", (int)computed_checksum, (int)remote_checksum, (unsigned int)this->rx_response_checksum_fail_);

      auto it_next = std::search(this->rx_buffer_.begin() + 1, this->rx_buffer_.end(),
                                 pattern_response_header.begin(), pattern_response_header.end());
      size_t index_next = std::distance(this->rx_buffer_.begin(), it_next);
      erase_prefix(index_next);
      result = RX_CHECKSUM_FAIL;
      return true;
    }

    // The request this response answers tells which slave it comes from. This is more reliable than the
    // address embedded in the frame, which is not correct for every frame type (e.g. live data of a second BMS
    // arrived as address 0 and was decoded into the first BMS).
    bool address_from_request = false;
    const uint8_t address_in_frame = address;
    const int16_t request_address_seen = this->last_request_address_;
    const uint8_t request_type_seen = this->last_request_frame_type_;
    const uint32_t request_age_ms = now - this->last_request_ms_;
    if (this->last_request_address_ >= 0) {
      const bool request_matches = (raw[JKPB_RS485_FRAME_TYPE_ADDRESS] == this->last_request_frame_type_) &&
                                   ((now - this->last_request_ms_) <= MAX_RESPONSE_DELAY_AFTER_REQUEST_MILLISECONDS);
      if (request_matches) {
        if (address != (uint8_t) this->last_request_address_) {
          ESP_LOGW(TAG, "Frame type 0x%02X contains address 0x%02X but it answers a request to address 0x%02X: using 0x%02X",
                   raw[JKPB_RS485_FRAME_TYPE_ADDRESS], address, (uint8_t) this->last_request_address_,
                   (uint8_t) this->last_request_address_);
        }
        address = (uint8_t) this->last_request_address_;
        address_from_request = true;
      } else {
        ESP_LOGD(TAG, "Frame type 0x%02X does not match the last request (type 0x%02X, %u ms ago): using address in frame 0x%02X",
                 raw[JKPB_RS485_FRAME_TYPE_ADDRESS], this->last_request_frame_type_,
                 (unsigned int) (now - this->last_request_ms_), address);
      }
      this->last_request_address_ = -1;  // a request is answered only once
    }

    // One line per valid response: shows what arrives on the bus and which address is used for it
    char voltage_text[24] = "";
    if (raw[JKPB_RS485_FRAME_TYPE_ADDRESS] == 2) {
      const uint32_t raw_voltage = raw[150] | (raw[151] << 8) | (raw[152] << 16) | ((uint32_t) raw[153] << 24);
      snprintf(voltage_text, sizeof(voltage_text), " | pack %.2f V", raw_voltage * 0.001f);
    }
    char request_text[48];
    if (request_address_seen >= 0) {
      snprintf(request_text, sizeof(request_text), "to 0x%02X, type 0x%02X, %u ms ago", (unsigned int) request_address_seen,
               (unsigned int) request_type_seen, (unsigned int) request_age_ms);
    } else {
      snprintf(request_text, sizeof(request_text), "none seen");
    }
    ESP_LOGI(TAG, "RX response type 0x%02X | address in frame: 0x%02X (byte 270: 0x%02X, byte 300: 0x%02X) | last request: %s | origin used: 0x%02X%s",
             raw[JKPB_RS485_FRAME_TYPE_ADDRESS], address_in_frame,
             raw[JKPB_RS485_FRAME_TYPE_ADDRESS_FOR_FRAME_TYPE_x01 + 6], raw[JKPB_RS485_ADDRESS_OF_RS485_ADDRESS],
             request_text, address, voltage_text);

    const uint8_t frame_type = raw[JKPB_RS485_FRAME_TYPE_ADDRESS];

    // A frame waiting for its settings frame is resolved first (frame order on the bus is kept)
    this->resolve_pending_live_frame_(frame_type == 1 ? (int16_t) address : (int16_t) -1, now);

    std::vector<uint8_t> data(this->rx_buffer_.begin(), this->rx_buffer_.begin() + response_size);

    if (frame_type == 2 && address_in_frame != 0 && !this->live_frame_address_is_reliable_) {
      this->live_frame_address_is_reliable_ = true;
      ESP_LOGI(TAG, "Live data frames carry a sender address (0x%02X): not pairing them with settings frames", address_in_frame);
    }

    if (frame_type == 2 && !address_from_request && !this->live_frame_address_is_reliable_) {
      // The address of a live data frame can't be trusted: hold it until the settings frame that follows
      this->pending_live_frame_.valid = true;
      this->pending_live_frame_.data = std::move(data);
      this->pending_live_frame_.address_in_frame = address;
      this->pending_live_frame_.received_ms = now;
    } else if (!this->accept_response_(address, frame_type, data, now)) {
      this->rx_buffer_.clear();
      result = RX_PARSED_RESPONSE;
      return true;
    }

    erase_prefix(response_size);
    result = RX_PARSED_RESPONSE;
    return true;
  };

  if (try_parse_short_request()) {
    return(result);
  }
  if (try_parse_request()) {
    return(result);
  }
  if (try_parse_response()) {
    return(result);
  }

  return(result);
}

void JkRS485Sniffer::dump_config() {
  ESP_LOGCONFIG(TAG, "JkRS485Sniffer:");
  ESP_LOGCONFIG(TAG, "  RX timeout: %d ms", this->rx_timeout_);
  ESP_LOGCONFIG(TAG, "  RX frames OK: %u", this->rx_frames_ok_);
  ESP_LOGCONFIG(TAG, "  RX preamble drops: %u", this->rx_preamble_drop_);
  ESP_LOGCONFIG(TAG, "  RX checksum failures: short=%u request=%u response=%u",
                this->rx_short_checksum_fail_, this->rx_request_checksum_fail_, this->rx_response_checksum_fail_);
}
float JkRS485Sniffer::get_setup_priority() const {

  return setup_priority::BUS - 1.0f;
}

}
}
