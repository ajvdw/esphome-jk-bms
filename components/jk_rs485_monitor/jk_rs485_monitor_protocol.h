#pragma once

// Protocol layer of the JK RS485 monitor. This file has no ESPHome dependencies, so it can be unit tested on a PC
// (see tests/test_monitor_protocol.cpp).

#include <cstddef>
#include <cstdint>
#include <vector>

namespace esphome {
namespace jk_rs485_monitor {

// ---------------------------------------------------------------------------------------------------------------
// Checksums
// ---------------------------------------------------------------------------------------------------------------

// CRC16 (Modbus) of the legacy requests, returned in the byte order they are sent: high byte first
inline uint16_t crc16_c(const uint8_t data[], const uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      if ((crc & 0x1) == 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }
  return (((crc & 0x00FF) << 8) | ((crc & 0xFF00)) >> 8);
}

// Arithmetic sum of bytes
inline uint16_t chksum(const uint8_t data[], const size_t len) {
  uint16_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum = checksum + data[i];
  }
  return checksum;
}

// ---------------------------------------------------------------------------------------------------------------
// NW frame
//
//   Offset    Length   Field          Description
//   0         2        Start header   0x4E 0x57 (ASCII "NW")
//   2         2        Frame length   Number of bytes from offset 2 to the end of the checksum
//   4         4        Terminal ID    BMS id / serial number (0 for broadcast / single)
//   8         1        Command word   0x01 Activate, 0x02 Write, 0x03 Read, 0x05 Pair, 0x06 Read all data
//   9         1        Frame source   0x00 BMS, 0x01 Bluetooth, 0x02 GPS, 0x03 PC
//   10        1        Transport      0x00 Request, 0x01 Reply, 0x02 Active (unsolicited) report
//   11        N        Information    [Data ID (1 byte)] + [value (X bytes)] pairs
//   11 + N    4        Record number  Rolling packet sequence counter
//   15 + N    1        End marker     0x68
//   16 + N    4        Checksum       0x00 0x00 + 16 bit sum of all bytes from the start header up to the end marker
//
// All multi byte fields are big endian.
// ---------------------------------------------------------------------------------------------------------------

static constexpr uint8_t NW_START_HEADER_1 = 0x4E;
static constexpr uint8_t NW_START_HEADER_2 = 0x57;
static constexpr uint8_t NW_END_MARKER = 0x68;

// Frame without information field: header 2 + length 2 + terminal 4 + command/source/transport 3 + record 4 +
// end marker 1 + checksum 4
static constexpr size_t NW_MIN_FRAME_SIZE = 20;
static constexpr size_t NW_INFORMATION_OFFSET = 11;

static constexpr uint8_t NW_COMMAND_ACTIVATE = 0x01;
static constexpr uint8_t NW_COMMAND_WRITE = 0x02;
static constexpr uint8_t NW_COMMAND_READ = 0x03;
static constexpr uint8_t NW_COMMAND_PAIR = 0x05;
static constexpr uint8_t NW_COMMAND_READ_ALL = 0x06;

static constexpr uint8_t NW_TRANSPORT_REQUEST = 0x00;
static constexpr uint8_t NW_TRANSPORT_REPLY = 0x01;
static constexpr uint8_t NW_TRANSPORT_REPORT = 0x02;

struct NwFrame {
  uint16_t length{0};  // value of the length field
  uint32_t terminal_id{0};
  uint8_t command{0};
  uint8_t source{0};
  uint8_t transport{0};
  std::vector<uint8_t> information;  // [Data ID] + [value] pairs, not split up
  uint32_t record_number{0};
  uint8_t end_marker{0};
  uint32_t checksum{0};            // as received
  uint16_t checksum_computed{0};   // sum of the bytes from the start header up to the end marker
  size_t total_size{0};            // bytes of the frame including the start header (length field + 2)
};

enum class NwParseStatus : uint8_t {
  OK,
  NOT_NW,            // the data doesn't start with the NW start header
  NEED_MORE,         // start header found but the frame is incomplete
  BAD_LENGTH,        // length field too small or larger than max_frame_size
  BAD_END_MARKER,
  BAD_CHECKSUM,
};

inline const char *nw_parse_status_to_string(NwParseStatus status) {
  switch (status) {
    case NwParseStatus::OK:
      return "ok";
    case NwParseStatus::NOT_NW:
      return "not an NW frame";
    case NwParseStatus::NEED_MORE:
      return "incomplete";
    case NwParseStatus::BAD_LENGTH:
      return "invalid length";
    case NwParseStatus::BAD_END_MARKER:
      return "invalid end marker";
    case NwParseStatus::BAD_CHECKSUM:
      return "checksum mismatch";
  }
  return "unknown";
}

inline const char *nw_command_to_string(uint8_t value) {
  switch (value) {
    case NW_COMMAND_ACTIVATE:
      return "Activate";
    case NW_COMMAND_WRITE:
      return "Write";
    case NW_COMMAND_READ:
      return "Read";
    case NW_COMMAND_PAIR:
      return "Pair";
    case NW_COMMAND_READ_ALL:
      return "Read all data";
    default:
      return "unknown";
  }
}

inline const char *nw_source_to_string(uint8_t value) {
  switch (value) {
    case 0x00:
      return "BMS";
    case 0x01:
      return "Bluetooth";
    case 0x02:
      return "GPS";
    case 0x03:
      return "PC";
    default:
      return "unknown";
  }
}

inline const char *nw_transport_to_string(uint8_t value) {
  switch (value) {
    case NW_TRANSPORT_REQUEST:
      return "Request";
    case NW_TRANSPORT_REPLY:
      return "Reply";
    case NW_TRANSPORT_REPORT:
      return "Active report";
    default:
      return "unknown";
  }
}

// Decodes the NW frame at the start of data.
//  - OK: `out` holds the decoded frame, out.total_size bytes were used.
//  - BAD_END_MARKER / BAD_CHECKSUM: the frame is complete but invalid; `out` is filled as far as possible.
//  - BAD_LENGTH: out.total_size is 0.
// max_frame_size limits the accepted size of a frame (the length field is 16 bit and can't be trusted).
inline NwParseStatus nw_parse(const uint8_t *data, size_t size, size_t max_frame_size, NwFrame &out) {
  out = NwFrame();
  if (size >= 1 && data[0] != NW_START_HEADER_1) {
    return NwParseStatus::NOT_NW;
  }
  if (size >= 2 && data[1] != NW_START_HEADER_2) {
    return NwParseStatus::NOT_NW;
  }
  if (size < 4) {
    return NwParseStatus::NEED_MORE;
  }

  out.length = (uint16_t(data[2]) << 8) | data[3];
  const size_t total = size_t(out.length) + 2;
  if (total < NW_MIN_FRAME_SIZE || total > max_frame_size) {
    return NwParseStatus::BAD_LENGTH;
  }
  if (size < total) {
    return NwParseStatus::NEED_MORE;
  }

  out.total_size = total;
  out.terminal_id = (uint32_t(data[4]) << 24) | (uint32_t(data[5]) << 16) | (uint32_t(data[6]) << 8) | data[7];
  out.command = data[8];
  out.source = data[9];
  out.transport = data[10];

  const size_t information_size = total - NW_MIN_FRAME_SIZE;
  out.information.assign(data + NW_INFORMATION_OFFSET, data + NW_INFORMATION_OFFSET + information_size);

  const size_t record_offset = NW_INFORMATION_OFFSET + information_size;
  out.record_number = (uint32_t(data[record_offset]) << 24) | (uint32_t(data[record_offset + 1]) << 16) |
                      (uint32_t(data[record_offset + 2]) << 8) | data[record_offset + 3];
  out.end_marker = data[record_offset + 4];

  const size_t checksum_offset = record_offset + 5;
  out.checksum = (uint32_t(data[checksum_offset]) << 24) | (uint32_t(data[checksum_offset + 1]) << 16) |
                 (uint32_t(data[checksum_offset + 2]) << 8) | data[checksum_offset + 3];
  out.checksum_computed = chksum(data, checksum_offset);  // start header up to and including the end marker

  if (out.end_marker != NW_END_MARKER) {
    return NwParseStatus::BAD_END_MARKER;
  }
  if (out.checksum != out.checksum_computed) {
    return NwParseStatus::BAD_CHECKSUM;
  }
  return NwParseStatus::OK;
}

// ---------------------------------------------------------------------------------------------------------------
// Legacy reply: 55 AA EB 90 <frame type> ... (308 bytes)
//
// The origin of a reply is the pack id. It is stored in the frame types 1 (settings) and 2 (live data), in the byte
// that is 8 bytes before the end of the frame: index 300 of the 308 bytes.
// ---------------------------------------------------------------------------------------------------------------

static constexpr uint8_t LEGACY_FRAME_TYPE_SETTINGS = 1;
static constexpr uint8_t LEGACY_FRAME_TYPE_LIVE_DATA = 2;
static constexpr uint8_t LEGACY_FRAME_TYPE_DEVICE_INFO = 3;
static constexpr size_t LEGACY_REPLY_PACK_ID_OFFSET_FROM_END = 8;

// The pack id is defined for the frame types 1 and 2
inline bool legacy_frame_type_has_pack_id(uint8_t frame_type) {
  return frame_type == LEGACY_FRAME_TYPE_SETTINGS || frame_type == LEGACY_FRAME_TYPE_LIVE_DATA;
}

// Index of the pack id in a frame of frame_length bytes (frame_length >= LEGACY_REPLY_PACK_ID_OFFSET_FROM_END)
inline size_t legacy_reply_pack_id_index(size_t frame_length) {
  return frame_length - LEGACY_REPLY_PACK_ID_OFFSET_FROM_END;
}

inline uint8_t legacy_reply_pack_id(const uint8_t *frame, size_t frame_length) {
  return frame[legacy_reply_pack_id_index(frame_length)];
}

}  // namespace jk_rs485_monitor
}  // namespace esphome
