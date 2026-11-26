# esphome-jk-bms

ESPHome components to monitor Jikong Battery Management Systems (JK-BMS) via RS-485
monitor your new JK-PBx BMSs via internal RS485 network using 1 only ESP.

![Visualized in Home Assistant](https://github.com/ajvdw/esphome-jk-bms/blob/main/media/battery-card.png?raw=true)

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

### Migrating from `jk_rs485_sniffer`

Rename `jk_rs485_sniffer:` to `jk_rs485_monitor:` and `jk_rs485_sniffer_id:` to `jk_rs485_monitor_id:`. The `talk_pin`
option does not exist any more.

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
