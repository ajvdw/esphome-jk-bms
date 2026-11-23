#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"

#include "jk_rs485_monitor_protocol.h"

namespace esphome {

namespace jk_rs485_monitor {

enum ProtocolVersion {
  PROTOCOL_VERSION_JK04,
  PROTOCOL_VERSION_JK02_24S,
  PROTOCOL_VERSION_JK02_32S,
};

class JkRS485MonitorDevice;

// Listens to the RS485 bus, it never transmits.
//
// Every message that is received is logged in hex and decoded. A master (address 0) sends requests to slaves, the
// slave a request is addressed to is the origin of the reply that follows within the reply timeout (100 ms).
// Replies are handed over to the registered devices with that origin address.
//
// Messages that are understood:
//  - NW frames: 4E 57 <length> <terminal id> <command> <source> <transport> <information> <record> 68 <checksum>
//  - legacy requests (11 bytes) and their acknowledge (8 bytes), both with CRC16
//  - legacy replies: 55 AA EB 90 ..., 308 bytes
class JkRS485Monitor : public uart::UARTDevice, public Component {
 public:
  JkRS485Monitor() = default;

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
    this->rx_times_.reserve(RX_BUFFER_MAX_SIZE);
  }

  void loop() override;

  void dump_config() override;

  void register_device(JkRS485MonitorDevice *device) { this->devices_.push_back(device); }

  float get_setup_priority() const override;

  // Time without new bytes after which an incomplete message is given up
  void set_rx_timeout(uint16_t rx_timeout) { rx_timeout_ = rx_timeout; }

  // Maximum time between a request and the start of the reply of the slave the request was addressed to
  void set_reply_timeout(uint16_t reply_timeout) { this->origin_tracker_.set_max_reply_delay_ms(reply_timeout); }

 protected:
  ProtocolVersion protocol_version_{PROTOCOL_VERSION_JK02_32S};

  static constexpr size_t RX_BUFFER_MAX_SIZE = 4 * 308;

  // --- receiving
  bool parse_head_();
  void consume_(size_t count);
  size_t find_next_header_(size_t from) const;
  void flush_unknown_();
  void flush_incomplete_(const char *reason);

  // --- handling of the messages that were recognized
  void handle_nw_frame_(const NwFrame &frame, size_t size);
  void handle_legacy_request_(size_t size);
  void handle_legacy_acknowledge_(size_t size);
  void handle_legacy_reply_(size_t size);
  void log_invalid_(const char *what, size_t size);

  // Starts the log of a message: header line followed by the hex dump
  void log_message_(const char *label, const uint8_t *data, size_t size, uint32_t start_ms, uint32_t end_ms);

  // Decides who sent a reply (see the reply timeout). Returns false if that can't be told.
  bool resolve_origin_(uint32_t start_ms, uint32_t end_ms, uint8_t &origin);

  void dispatch_legacy_reply_(uint8_t origin, uint8_t frame_type, const std::vector<uint8_t> &data, uint32_t now);
  void dispatch_nw_reply_(uint8_t origin, const NwFrame &frame, uint32_t now);

  void note_master_activity_(uint32_t now);
  void note_slave_activity_(uint8_t address, uint32_t now);

  std::string nodes_available_to_string();
  void set_node_availability(uint8_t address, bool value);

  // Bytes that were received but not yet handled, with the time (millis) each byte was read
  std::vector<uint8_t> rx_buffer_;
  std::vector<uint32_t> rx_times_;
  uint32_t last_byte_ms_{0};

  // Bytes that don't belong to any message that is understood
  std::vector<uint8_t> unknown_run_;
  uint32_t unknown_run_start_ms_{0};
  uint32_t unknown_run_end_ms_{0};

  ReplyOriginTracker origin_tracker_;

  uint16_t rx_timeout_{50};
  uint32_t message_counter_{0};
  bool have_previous_message_{false};
  uint32_t previous_message_end_ms_{0};

  uint32_t rx_nw_ok_{0};
  uint32_t rx_nw_invalid_{0};
  uint32_t rx_legacy_ok_{0};
  uint32_t rx_legacy_invalid_{0};
  uint32_t rx_unknown_bytes_{0};
  uint32_t rx_incomplete_{0};
  uint32_t rx_replies_without_origin_{0};

  std::vector<JkRS485MonitorDevice *> devices_;

  struct struct_rs485_network_node {
    bool available;
    uint32_t last_message_received;
  };
  struct struct_rs485_network_node rs485_network_node[16];

  std::string nodes_available;
};

class JkRS485MonitorDevice {
 public:
  void set_parent(JkRS485Monitor *parent) { parent_ = parent; }

  // A reply in the legacy format (55 AA EB 90 ..., frame type in data[4]) from the slave with origin_address
  virtual void on_jk_rs485_monitor_data(const uint8_t &origin_address, const uint8_t &frame_type,
                                        const std::vector<uint8_t> &data, const std::string &nodes_available) = 0;

  // A reply in the NW format from the slave with origin_address
  virtual void on_jk_rs485_monitor_nw_frame(const uint8_t &, const NwFrame &, const std::string &) {}

 protected:
  friend JkRS485Monitor;

  JkRS485Monitor *parent_{nullptr};
};

}  // namespace jk_rs485_monitor
}  // namespace esphome
