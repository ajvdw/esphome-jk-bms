# esphome-jk-bms

ESPHome components to monitor Jikong Battery Management Systems (JK-BMS) via RS-485
monitor your new JK-PBx BMSs via internal RS485 network using 1 only ESP.

![Visualized in Home Assistant](https://github.com/ajvdw/wifi-jk-bms/blob/main/media/screenshot_bms.png?raw=true)

## Multiple BMSs as sub-devices
 
Each BMS can be shown as its own device in Home Assistant. Declare the sub-devices in the `esphome:` block and set
`device_id` **once on the BMS**. All sensors, binary sensors and text sensors of that BMS are placed on this device
automatically, so `device_id` doesn't have to be repeated per entity:
 
```yaml
esphome:
  devices:
    - id: bms_device_0
      name: BMS-right
      relation: child      # or: via
    - id: bms_device_1
      name: BMS-left
      relation: child
 
jk_rs485_bms:
  - id: bms0
    device_id: bms_device_0
    rs485_address: 0x00
    jk_rs485_monitor_id: monitor0
  - id: bms1
    device_id: bms_device_1
    rs485_address: 0x01
    jk_rs485_monitor_id: monitor0
 
sensor:
  - platform: jk_rs485_bms
    jk_rs485_bms_id: bms1
    battery_voltage:
      name: "battery voltage"      # ends up on bms_device_1
```
 
* `relation` is required on sub-devices since ESPHome 2026.11. Both `via` (a separate physical device reached through
  this node) and `child` (a logical part of this node) are supported.
* An entity that sets a `device_id` different from its BMS is rejected, to catch copy/paste mixups between BMSs.
* Without a `device_id` on the BMS the previous behaviour is kept: `device_id` is set per entity.

## References
* https://www.youtube.com/@OffGridGarageAustralia
* https://github.com/syssi/esphome-jk-bms
* https://github.com/txubelaxu/esphome-jk-bms
* https://github.com/jblance/jkbms
* https://github.com/jblance/mpp-solar/issues/112
* https://github.com/jblance/mpp-solar/blob/master/mppsolar/protocols/jk232.py
* https://github.com/jblance/mpp-solar/blob/master/mppsolar/protocols/jk485.py
* https://github.com/sshoecraft/jktool
* https://github.com/Louisvdw/dbus-serialbattery/blob/master/etc/dbus-serialbattery/jkbms.py
* https://blog.ja-ke.tech/2020/02/07/ltt-power-bms-chinese-protocol.html

## Monitor

`jk_rs485_monitor` (formerly `jk_rs485_sniffer`) only listens to the RS485 bus, it never transmits. Configure the
`uart:` with an `rx_pin` only (no `tx_pin`, no flow control pin). The BMSs must be polled by a master on the bus.

```yaml
jk_rs485_monitor:
  - id: monitor0
    protocol_version: "JK02_32S"
    uart_id: uart_0
    rx_timeout: 50ms        # an incomplete message is given up after this time without new bytes
```

The BMS components refer to it with `jk_rs485_monitor_id: monitor0`.

### Who sent a reply?

The pack id. The frame types 1 (settings) and 2 (live data) of the legacy replies contain it in the byte that is 7
bytes before the end of the frame (index 301 of the 308 bytes). If that byte is `0x10`, the pack id is the byte before
it (index 300). It is the origin address of the reply: the `jk_rs485_bms` with that `rs485_address` receives it.
Requests, timing and the other address fields of a frame are not used.

* A pack id above 15 is invalid: the reply is logged with a warning and ignored.
* Frame type 3 (device info) has no pack id: it is logged with a warning and not handed over.
* The pack id is logged for every reply together with the index it was read from (`pack id 0x01 (byte 301): origin
  0x01`, or byte 300 after the `0x10` rule). If all your replies show `0x00`, the BMSs do not fill it in and every
  reply is for address 0.

### Logging

Every message that is received is logged in hex at `DEBUG` level, requests as well as replies, followed by its decoded
fields and, for a reply, its pack id. Bytes that are not understood, invalid frames (checksum, end marker,
length) and incomplete messages are logged in hex as well. A message is logged in rows of 32 bytes.

```
RX #1 NW request, 21 bytes in 0 ms, first message
  [0000] 4E 57 00 13 00 00 00 00 06 03 00 00 00 00 00 00 68 00 00 01 29
  NW length=19 terminal=0x00000000 command=0x06 (Read all data) source=0x03 (PC) transport=0x00 (Request)
  NW information=1 bytes, record=0x00000000, end=0x68, checksum=0x00000129
RX #2 legacy reply, frame type 2 (live data), 308 bytes in 0 ms, 60 ms after the previous message
  [0000] 55 AA EB 90 02 01 E4 0C E5 0C E6 0C E7 0C E8 0C E9 0C EA 0C EB 0C EC 0C ED 0C EE 0C EF 0C F0 0C
  ...
  pack id 0x01 (byte 301): origin 0x01
```

### Messages that are decoded

NW frames, all multi byte fields are big endian:

| Offset | Length | Field | Description |
|---|---|---|---|
| 0 | 2 | Start header | `0x4E 0x57` ("NW") |
| 2 | 2 | Frame length | bytes from offset 2 to the end of the checksum |
| 4 | 4 | Terminal ID | BMS id / serial number (0 for broadcast / single) |
| 8 | 1 | Command word | 0x01 Activate, 0x02 Write, 0x03 Read, 0x05 Pair, 0x06 Read all data |
| 9 | 1 | Frame source | 0x00 BMS, 0x01 Bluetooth, 0x02 GPS, 0x03 PC |
| 10 | 1 | Transport type | 0x00 Request, 0x01 Reply, 0x02 Active report |
| 11 | N | Information field | `[Data ID (1 byte)] + [value (X bytes)]` pairs |
| 11 + N | 4 | Record number | rolling packet sequence counter |
| 15 + N | 1 | End marker | `0x68` |
| 16 + N | 4 | Checksum | `0x00 0x00` + 16 bit sum of all bytes from `0x4E` up to and including `0x68` |

A frame is accepted if the length, the end marker and the checksum are correct. The information field is passed on as
it is; it is not split into data ids and values.

The legacy messages are still understood: requests (`<address> 10 16 <code> .. <crc16>`, 11 bytes), their acknowledge
(8 bytes) and the replies (`55 AA EB 90 ...`, 308 bytes). The entities of `jk_rs485_bms` are filled from these replies.

### Migrating from `jk_rs485_sniffer`

Rename `jk_rs485_sniffer:` to `jk_rs485_monitor:` and `jk_rs485_sniffer_id:` to `jk_rs485_monitor_id:`. The `talk_pin`
option does not exist any more.

### Tests

The protocol layer (NW decoding and the origin rule) has no ESPHome dependencies and is tested on a PC:

```bash
g++ -std=c++17 -Wall -Wextra -I components/jk_rs485_monitor tests/test_monitor_protocol.cpp -o /tmp/test_monitor && /tmp/test_monitor
```
