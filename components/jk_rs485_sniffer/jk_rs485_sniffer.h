#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"

namespace esphome {

namespace jk_rs485_sniffer {

enum ProtocolVersion {
  PROTOCOL_VERSION_JK04,
  PROTOCOL_VERSION_JK02_24S,
  PROTOCOL_VERSION_JK02_32S,
};

class JkRS485SnifferDevice;

class JkRS485Sniffer : public uart::UARTDevice, public Component {
 public:
  JkRS485Sniffer() = default;

  void setup() override {
    for (uint8_t cont = 0; cont < 16; cont++) {
      rs485_network_node[cont].available = false;
      rs485_network_node[cont].last_message_received = 0;
    }

    nodes_available.reserve(17);
    for (uint8_t cont = 0; cont < 16; cont++) {
      nodes_available.push_back('0');
    }
    nodes_available.push_back('\0');

    this->rx_buffer_.reserve(RX_BUFFER_MAX_SIZE);
  }

  void loop() override;

  void dump_config() override;

  void register_device(JkRS485SnifferDevice *device) { this->devices_.push_back(device); }

  float get_setup_priority() const override;

  void set_rx_timeout(uint16_t rx_timeout) { rx_timeout_ = rx_timeout; }

 protected:
  ProtocolVersion protocol_version_{PROTOCOL_VERSION_JK02_32S};

  uint8_t manage_rx_buffer_(void);
  void set_node_availability(uint8_t address,bool value);

  // A slave only answers requests addressed to it, so the address of the request a response belongs to is the
  // origin of that response. The address embedded in the response frame is only used when no matching request
  // was seen (it turned out not to be reliable for all frame types).
  void remember_request_(uint8_t address, uint8_t frame_type);
  int16_t last_request_address_{-1};
  uint8_t last_request_frame_type_{0};
  uint32_t last_request_ms_{0};

  // On some buses the live data frames (type 0x02) carry no usable sender address (byte 300 is 0 in every frame).
  // The settings frame (type 0x01) does: its payload holds the configured address of the BMS, and the BMS sends it
  // right after its live data frame. A live data frame is therefore held back until the next frame arrives and is
  // credited to the address of that settings frame. Without a settings frame it is dispatched with the address it
  // carries, after a short timeout.
  struct PendingLiveFrame {
    bool valid{false};
    std::vector<uint8_t> data;
    uint8_t address_in_frame{0};
    uint32_t received_ms{0};
  };
  PendingLiveFrame pending_live_frame_;
  // Set as soon as a live data frame carries a non-zero address: the address in live data frames can be trusted
  // on this bus and the pairing with settings frames is not used.
  bool live_frame_address_is_reliable_{false};
  bool accept_response_(uint8_t address, uint8_t frame_type, const std::vector<uint8_t> &data, uint32_t now);
  void resolve_pending_live_frame_(int16_t address_of_following_settings_frame, uint32_t now);
  std::string nodes_available_to_string();

  static constexpr size_t RX_BUFFER_MAX_SIZE = 4 * 308;

  std::vector<uint8_t> rx_buffer_;
  uint16_t rx_timeout_{50};
  uint32_t last_jk_rs485_network_activity_{0};
  uint32_t rx_short_checksum_fail_{0};
  uint32_t rx_request_checksum_fail_{0};
  uint32_t rx_response_checksum_fail_{0};
  uint32_t rx_preamble_drop_{0};
  uint32_t rx_frames_ok_{0};
  std::vector<JkRS485SnifferDevice *> devices_;

  struct struct_rs485_network_node {
     bool available;
     uint32_t last_message_received;
  };
  struct struct_rs485_network_node rs485_network_node[16];

  std::string nodes_available;
};

class JkRS485SnifferDevice {
 public:
  void set_parent(JkRS485Sniffer *parent) { parent_ = parent; }
  virtual void on_jk_rs485_sniffer_data(const uint8_t &origin_address, const uint8_t &frame_type, const std::vector<uint8_t> &data, const std::string &nodes_available) = 0;

 protected:
  friend JkRS485Sniffer;

  JkRS485Sniffer *parent_{nullptr};
};

}
}
