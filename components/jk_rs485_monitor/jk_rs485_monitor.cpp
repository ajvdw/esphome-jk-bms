#include "jk_rs485_monitor.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace esphome {
namespace jk_rs485_monitor {

static const char *const TAG = "jk_rs485_monitor";

// Legacy messages
static const size_t LEGACY_ACKNOWLEDGE_SIZE = 8;
static const size_t LEGACY_REQUEST_SIZE = 11;
static const size_t LEGACY_REPLY_SIZE = 308;
static const size_t LEGACY_REPLY_CHECKSUM_INDEX = 299;
static const size_t LEGACY_REPLY_FRAME_TYPE_INDEX = 4;
static const uint8_t LEGACY_REPLY_HEADER[4] = {0x55, 0xAA, 0xEB, 0x90};
static const uint8_t LEGACY_FUNCTION_WRITE_REGISTERS = 0x10;
static const uint8_t MAX_ADDRESS = 15;  // the nodes_ array has MAX_ADDRESS + 1 entries

static const size_t HEX_BYTES_PER_LINE = 32;
static const size_t UNKNOWN_RUN_MAX_SIZE = 64;

static const uint16_t NO_MESSAGE_RECEIVED_TIME_SET_AS_UNAVAILABLE_MILLISECONDS = 10000;

static const char *legacy_frame_type_to_string(uint8_t frame_type) {
  switch (frame_type) {
    case 0x01:
      return "settings";
    case 0x02:
      return "live data";
    case 0x03:
      return "device info";
    default:
      return "unknown";
  }
}

// Logs the bytes in rows of HEX_BYTES_PER_LINE: a line of the log is limited in length
static void log_hex_rows(const uint8_t *data, size_t size) {
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
  char text[format_hex_pretty_size(HEX_BYTES_PER_LINE)];
  for (size_t offset = 0; offset < size; offset += HEX_BYTES_PER_LINE) {
    const size_t count = std::min(HEX_BYTES_PER_LINE, size - offset);
    format_hex_pretty_to(text, data + offset, count, ' ');
    ESP_LOGD(TAG, "  [%04u] %s", (unsigned int) offset, text);
  }
#else
  (void) data;
  (void) size;
#endif
}

// ---------------------------------------------------------------------------------------------------------------
// Receiving
// ---------------------------------------------------------------------------------------------------------------

void JkRS485Monitor::loop() {
  const uint32_t now = millis();
  bool received = false;

  while (this->available() && this->rx_buffer_.size() < RX_BUFFER_MAX_SIZE) {
    uint8_t byte;
    this->read_byte(&byte);
    this->rx_buffer_.push_back(byte);
    this->rx_times_.push_back(now);
    received = true;
  }
  if (received) {
    this->last_byte_ms_ = now;
  }

  while (this->parse_head_()) {
  }

  // What is left is the start of a message. If the rest doesn't follow it is given up.
  if (!this->rx_buffer_.empty()) {
    if (this->rx_buffer_.size() >= RX_BUFFER_MAX_SIZE) {
      this->flush_incomplete_("buffer full");
    } else if ((now - this->last_byte_ms_) > this->rx_timeout_) {
      this->flush_incomplete_("timeout");
    }
  }
  if (!this->unknown_run_.empty() && (now - this->last_byte_ms_) > this->rx_timeout_) {
    this->flush_unknown_();
  }

  if (!received) {
    // Nodes that stay silent for too long are no longer available
    for (uint8_t address = 0; address < MAX_NODES; address++) {
      if (now - this->nodes_[address].last_message_received >
          NO_MESSAGE_RECEIVED_TIME_SET_AS_UNAVAILABLE_MILLISECONDS) {
        this->set_node_availability(address, 0);
      }
    }
  }
}

void JkRS485Monitor::consume_(size_t count) {
  count = std::min(count, this->rx_buffer_.size());
  this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + count);
  this->rx_times_.erase(this->rx_times_.begin(), this->rx_times_.begin() + count);
}

// Position (>= from) of the next start of an NW frame or legacy reply, or the buffer size if there is none
size_t JkRS485Monitor::find_next_header_(size_t from) const {
  const uint8_t *p = this->rx_buffer_.data();
  const size_t n = this->rx_buffer_.size();
  for (size_t i = from; i + 1 < n; i++) {
    if (p[i] == NW_START_HEADER_1 && p[i + 1] == NW_START_HEADER_2) {
      return i;
    }
    if (p[i] == LEGACY_REPLY_HEADER[0] && p[i + 1] == LEGACY_REPLY_HEADER[1] &&
        (i + 3 >= n || (p[i + 2] == LEGACY_REPLY_HEADER[2] && p[i + 3] == LEGACY_REPLY_HEADER[3]))) {
      return i;
    }
  }
  return n;
}

// Looks at the start of the receive buffer. Returns true if bytes were taken out of it, false if more bytes are
// needed to decide what it is.
bool JkRS485Monitor::parse_head_() {
  if (this->rx_buffer_.empty()) {
    return false;
  }
  const uint8_t *p = this->rx_buffer_.data();
  const size_t n = this->rx_buffer_.size();

  // --- NW frame
  if (p[0] == NW_START_HEADER_1) {
    NwFrame frame;
    const NwParseStatus status = nw_parse(p, n, RX_BUFFER_MAX_SIZE, frame);
    switch (status) {
      case NwParseStatus::OK:
        this->flush_unknown_();
        this->handle_nw_frame_(frame, frame.total_size);
        this->consume_(frame.total_size);
        return true;
      case NwParseStatus::NEED_MORE:
        return false;
      case NwParseStatus::BAD_END_MARKER:
      case NwParseStatus::BAD_CHECKSUM: {
        // A complete frame that is invalid. It is dropped, but a valid frame that starts inside of it is kept.
        const size_t drop = std::min(frame.total_size, this->find_next_header_(2));
        this->flush_unknown_();
        this->rx_nw_invalid_++;
        this->log_invalid_(nw_parse_status_to_string(status), drop);
        if (status == NwParseStatus::BAD_CHECKSUM) {
          ESP_LOGD(TAG, "  NW checksum received 0x%08X, computed 0x%04X", (unsigned int) frame.checksum,
                   (unsigned int) frame.checksum_computed);
        }
        this->consume_(drop);
        return true;
      }
      case NwParseStatus::BAD_LENGTH:
        // The length can't be trusted: the start header is treated as unknown bytes and the next one is looked for
        this->rx_nw_invalid_++;
        ESP_LOGD(TAG, "NW start header with invalid length %u (max %u), resynchronizing", (unsigned int) frame.length,
                 (unsigned int) (RX_BUFFER_MAX_SIZE - 2));
        for (size_t i = 0; i < 2; i++) {
          if (this->unknown_run_.empty()) {
            this->unknown_run_start_ms_ = this->rx_times_[0];
          }
          this->unknown_run_end_ms_ = this->rx_times_[0];
          this->unknown_run_.push_back(p[0]);
          this->rx_unknown_bytes_++;
          this->consume_(1);
          p = this->rx_buffer_.data();
        }
        return true;
      case NwParseStatus::NOT_NW:
        break;  // 0x4E that is not followed by 0x57: not an NW frame
    }
  }

  // --- legacy reply: 55 AA EB 90 ... (308 bytes)
  if (p[0] == LEGACY_REPLY_HEADER[0] && std::memcmp(p, LEGACY_REPLY_HEADER, std::min<size_t>(n, 4)) == 0) {
    if (n < LEGACY_REPLY_SIZE) {
      return false;
    }
    const uint8_t computed_checksum = (uint8_t) chksum(p, LEGACY_REPLY_CHECKSUM_INDEX);
    if (computed_checksum != p[LEGACY_REPLY_CHECKSUM_INDEX]) {
      const size_t drop = std::min(LEGACY_REPLY_SIZE, this->find_next_header_(1));
      this->flush_unknown_();
      this->rx_legacy_invalid_++;
      this->log_invalid_("checksum mismatch", drop);
      ESP_LOGD(TAG, "  legacy reply checksum received 0x%02X, computed 0x%02X", (unsigned int) p[LEGACY_REPLY_CHECKSUM_INDEX],
               (unsigned int) computed_checksum);
      this->consume_(drop);
      return true;
    }
    this->flush_unknown_();
    this->handle_legacy_reply_(LEGACY_REPLY_SIZE);
    this->consume_(LEGACY_REPLY_SIZE);
    return true;
  }

  // --- legacy request (11 bytes) and acknowledge (8 bytes): <address> 0x10 ... <crc16>
  if (p[0] <= MAX_ADDRESS && (n < 2 || p[1] == LEGACY_FUNCTION_WRITE_REGISTERS)) {
    if (n >= LEGACY_ACKNOWLEDGE_SIZE &&
        crc16_c(p, 6) == uint16_t((uint16_t(p[6]) << 8) | uint16_t(p[7]))) {
      this->flush_unknown_();
      this->handle_legacy_acknowledge_(LEGACY_ACKNOWLEDGE_SIZE);
      this->consume_(LEGACY_ACKNOWLEDGE_SIZE);
      return true;
    }
    if (n >= LEGACY_REQUEST_SIZE) {
      if (crc16_c(p, 9) == uint16_t((uint16_t(p[9]) << 8) | uint16_t(p[10]))) {
        this->flush_unknown_();
        this->handle_legacy_request_(LEGACY_REQUEST_SIZE);
        this->consume_(LEGACY_REQUEST_SIZE);
        return true;
      }
    } else {
      return false;  // could still become a legacy message
    }
  }

  // --- not understood: the byte is kept for the log of unknown bytes
  if (this->unknown_run_.empty()) {
    this->unknown_run_start_ms_ = this->rx_times_[0];
  }
  this->unknown_run_end_ms_ = this->rx_times_[0];
  this->unknown_run_.push_back(p[0]);
  this->rx_unknown_bytes_++;
  this->consume_(1);
  if (this->unknown_run_.size() >= UNKNOWN_RUN_MAX_SIZE) {
    this->flush_unknown_();
  }
  return true;
}

void JkRS485Monitor::flush_unknown_() {
  if (this->unknown_run_.empty()) {
    return;
  }
  this->log_message_("UNKNOWN bytes", this->unknown_run_.data(), this->unknown_run_.size(), this->unknown_run_start_ms_,
                     this->unknown_run_end_ms_);
  this->unknown_run_.clear();
}

void JkRS485Monitor::flush_incomplete_(const char *reason) {
  this->flush_unknown_();
  const size_t size = this->rx_buffer_.size();
  char label[48];
  snprintf(label, sizeof(label), "INCOMPLETE message (%s)", reason);
  this->log_message_(label, this->rx_buffer_.data(), size, this->rx_times_.front(), this->rx_times_.back());
  this->rx_incomplete_++;
  this->rx_buffer_.clear();
  this->rx_times_.clear();
}

// ---------------------------------------------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------------------------------------------

// Header line of a message followed by its bytes in hex
void JkRS485Monitor::log_message_(const char *label, const uint8_t *data, size_t size, uint32_t start_ms,
                                  uint32_t end_ms) {
  this->message_counter_++;

  char gap_text[80];
  if (this->have_previous_message_) {
    int32_t gap = int32_t(start_ms - this->previous_message_end_ms_);
    snprintf(gap_text, sizeof(gap_text), "%d ms after the previous message", (int) std::max<int32_t>(gap, 0));
  } else {
    snprintf(gap_text, sizeof(gap_text), "first message");
  }
  ESP_LOGD(TAG, "RX #%u %s, %u bytes in %u ms, %s", (unsigned int) this->message_counter_, label, (unsigned int) size,
           (unsigned int) (end_ms - start_ms), gap_text);
  log_hex_rows(data, size);

  this->have_previous_message_ = true;
  this->previous_message_end_ms_ = end_ms;
}

void JkRS485Monitor::log_invalid_(const char *what, size_t size) {
  char label[64];
  snprintf(label, sizeof(label), "INVALID frame (%s)", what);
  this->log_message_(label, this->rx_buffer_.data(), size, this->rx_times_.front(), this->rx_times_[size - 1]);
}

// ---------------------------------------------------------------------------------------------------------------
// Messages
// ---------------------------------------------------------------------------------------------------------------

void JkRS485Monitor::handle_nw_frame_(const NwFrame &frame, size_t size) {
  const uint8_t *data = this->rx_buffer_.data();
  const uint32_t start_ms = this->rx_times_.front();
  const uint32_t end_ms = this->rx_times_[size - 1];

  const char *label;
  switch (frame.transport) {
    case NW_TRANSPORT_REQUEST:
      label = "NW request";
      break;
    case NW_TRANSPORT_REPLY:
      label = "NW reply";
      break;
    case NW_TRANSPORT_REPORT:
      label = "NW report";
      break;
    default:
      label = "NW frame";
      break;
  }
  this->log_message_(label, data, size, start_ms, end_ms);
  this->rx_nw_ok_++;

  ESP_LOGD(TAG, "  NW length=%u terminal=0x%08X command=0x%02X (%s) source=0x%02X (%s) transport=0x%02X (%s)",
           (unsigned int) frame.length, (unsigned int) frame.terminal_id, (unsigned int) frame.command,
           nw_command_to_string(frame.command), (unsigned int) frame.source, nw_source_to_string(frame.source),
           (unsigned int) frame.transport, nw_transport_to_string(frame.transport));
  ESP_LOGD(TAG, "  NW information=%u bytes, record=0x%08X, end=0x%02X, checksum=0x%08X", (unsigned int) frame.information.size(),
           (unsigned int) frame.record_number, (unsigned int) frame.end_marker, (unsigned int) frame.checksum);

  // A request shows that the master is active
  if (frame.transport == NW_TRANSPORT_REQUEST) {
    this->note_master_activity_(millis());
  }
  this->dispatch_nw_frame_(frame);
}

void JkRS485Monitor::handle_legacy_request_(size_t size) {
  const uint8_t *data = this->rx_buffer_.data();
  const uint32_t start_ms = this->rx_times_.front();
  const uint32_t end_ms = this->rx_times_[size - 1];

  this->log_message_("legacy request", data, size, start_ms, end_ms);
  this->rx_legacy_ok_++;
  ESP_LOGD(TAG, "  legacy request to address 0x%02X, function 0x%02X, register 0x%02X%02X",
           (unsigned int) data[0], (unsigned int) data[1], (unsigned int) data[2], (unsigned int) data[3]);

  this->note_master_activity_(millis());
}

void JkRS485Monitor::handle_legacy_acknowledge_(size_t size) {
  const uint8_t *data = this->rx_buffer_.data();

  this->log_message_("legacy acknowledge", data, size, this->rx_times_.front(), this->rx_times_[size - 1]);
  this->rx_legacy_ok_++;
  ESP_LOGD(TAG, "  legacy acknowledge, address in the message 0x%02X, function 0x%02X, register 0x%02X%02X",
           (unsigned int) data[0], (unsigned int) data[1], (unsigned int) data[2], (unsigned int) data[3]);
}

void JkRS485Monitor::handle_legacy_reply_(size_t size) {
  const uint8_t *data = this->rx_buffer_.data();
  const uint8_t frame_type = data[LEGACY_REPLY_FRAME_TYPE_INDEX];

  char label[48];
  snprintf(label, sizeof(label), "legacy reply, frame type %u (%s)", (unsigned int) frame_type,
           legacy_frame_type_to_string(frame_type));
  this->log_message_(label, data, size, this->rx_times_.front(), this->rx_times_[size - 1]);
  this->rx_legacy_ok_++;

  // The origin is the pack id, it is defined for the frame types 1 and 2
  if (!legacy_frame_type_has_pack_id(frame_type)) {
    this->rx_replies_not_handed_over_++;
    ESP_LOGW(TAG, "  frame type %u has no pack id, reply not handed over", (unsigned int) frame_type);
    return;
  }

  // The pack id is 7 bytes before the end of the frame. If that byte is 0x10, the pack id is the byte before it.
  size_t pack_id_index = size - LEGACY_REPLY_PACK_ID_OFFSET_FROM_END;
  uint8_t pack_id = data[pack_id_index];
  if (pack_id == 0x10) {
    pack_id_index--;
    pack_id = data[pack_id_index];
  }

  if (pack_id > MAX_ADDRESS) {
    this->rx_replies_not_handed_over_++;
    ESP_LOGW(TAG, "  pack id 0x%02X (byte %u) is out of range (max %u), reply not handed over", (unsigned int) pack_id,
             (unsigned int) pack_id_index, (unsigned int) MAX_ADDRESS);
    return;
  }
  ESP_LOGD(TAG, "  pack id 0x%02X (byte %u): origin 0x%02X", (unsigned int) pack_id, (unsigned int) pack_id_index,
           (unsigned int) pack_id);

  this->dispatch_legacy_reply_(pack_id, frame_type, std::vector<uint8_t>(data, data + size), millis());
}

void JkRS485Monitor::note_master_activity_(uint32_t now) {
  this->nodes_[0].last_message_received = now;
  this->set_node_availability(0, 1);
}

void JkRS485Monitor::note_slave_activity_(uint8_t address, uint32_t now) {
  if (address > MAX_ADDRESS) {
    return;
  }
  this->nodes_[address].last_message_received = now;
  if (address != 0) {
    this->set_node_availability(address, 1);
  }
}

void JkRS485Monitor::dispatch_legacy_reply_(uint8_t origin, uint8_t frame_type, const std::vector<uint8_t> &data,
                                            uint32_t now) {
  this->note_slave_activity_(origin, now);

  if (this->devices_.empty()) {
    ESP_LOGW(TAG, "Got a reply [frame type:0x%02X] from 0x%02X but no recipients to send it to!", (unsigned int) frame_type,
             (unsigned int) origin);
    return;
  }
  for (auto *device : this->devices_) {
    device->on_jk_rs485_monitor_data(origin, frame_type, data, this->nodes_available_);
  }
}

void JkRS485Monitor::dispatch_nw_frame_(const NwFrame &frame) {
  for (auto *device : this->devices_) {
    device->on_jk_rs485_monitor_nw_frame(frame, this->nodes_available_);
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Nodes
// ---------------------------------------------------------------------------------------------------------------

std::string JkRS485Monitor::nodes_available_to_string() const {
  std::string text;
  text.reserve(MAX_NODES);
  for (const Node &node : this->nodes_) {
    text.push_back(node.available ? '1' : '0');
  }
  return text;
}

void JkRS485Monitor::set_node_availability(uint8_t address, bool value) {
  if (address > MAX_ADDRESS) {
    ESP_LOGE(TAG, "address 0x%02X out of range (max %u), ignoring", address, (unsigned int) MAX_ADDRESS);
    return;
  }

  if (this->nodes_[address].available != value) {
    uint8_t previous_value = this->nodes_[address].available;
    this->nodes_[address].available = value;

    std::string previous = this->nodes_available_;
    this->nodes_available_ = this->nodes_available_to_string();
    ESP_LOGI(TAG, "NODES AVAILABLE CHANGED: address 0x%02X (%d->%d) [%s] --> [%s]", address, previous_value, value,
             previous.c_str(), this->nodes_available_.c_str());
  }
}

void JkRS485Monitor::dump_config() {
  ESP_LOGCONFIG(TAG, "JkRS485Monitor:");
  ESP_LOGCONFIG(TAG, "  RX timeout: %d ms", this->rx_timeout_);
  ESP_LOGCONFIG(TAG, "  RX messages: %u", (unsigned int) this->message_counter_);
  ESP_LOGCONFIG(TAG, "  RX NW frames: ok=%u invalid=%u", (unsigned int) this->rx_nw_ok_, (unsigned int) this->rx_nw_invalid_);
  ESP_LOGCONFIG(TAG, "  RX legacy messages: ok=%u invalid=%u", (unsigned int) this->rx_legacy_ok_,
                (unsigned int) this->rx_legacy_invalid_);
  ESP_LOGCONFIG(TAG, "  RX unknown bytes: %u, incomplete messages: %u", (unsigned int) this->rx_unknown_bytes_,
                (unsigned int) this->rx_incomplete_);
  ESP_LOGCONFIG(TAG, "  Replies not handed over (no pack id): %u", (unsigned int) this->rx_replies_not_handed_over_);
}

float JkRS485Monitor::get_setup_priority() const { return setup_priority::BUS - 1.0f; }

}  // namespace jk_rs485_monitor
}  // namespace esphome
