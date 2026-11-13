# UWB Serial Driver (ROS 2, C++)

ROS 2 C++ package that reads UWB results from a Tag or Anchor ESP32 serial port,
validates and parses each line, then publishes structured measurements. The
implementation uses three layers:

- `src/serial_port.cpp`: POSIX serial connection and non-blocking line reads.
- `src/parser.cpp`: Makerfabs multi-anchor and JSON record validation/decoding.
- `src/uwb_serial_node.cpp`: ROS parameters, reconnect/error handling, topics.

## Topics

- `/uwb/measurements` (`std_msgs/msg/String`): normalized JSON record per input
  line, including tag/anchor IDs, sequence, device timestamp, quality values,
  ranging timestamps, clock offset, and payload when supplied by firmware.
  Each measurement has both `anchor_id` (index) and `anchor_radio_id`
  (numeric radio address `0xA0 + index`, serialized as a decimal JSON number).
- `/uwb/tag_<tag_id>/anchor_<anchor_id>/range`
  (`std_msgs/msg/Float32`): one topic for each valid tag/anchor pair; value is in meters. The normalized measurement topic carries the host receive stamp.
- `/uwb/anchor_<index>/tag_<tag_id>/reported_range`
  (`std_msgs/msg/Float32`): a range report as observed by the connected Anchor
  board. The range was measured by the Tag and relayed over UWB; it is not a
  second, independent measurement made by that Anchor.
- `/uwb/tag_<index>/status` or `/uwb/anchor_<index>/status`
  (`std_msgs/msg/String`): connection status associated with the board role
  detected from its Serial record.
- `/uwb/payload` (`std_msgs/msg/String`): application payload when present.
- `/uwb/status` (`std_msgs/msg/String`): JSON connection and parser status.

## Supported input

The updated multi-anchor Tag and Anchor sketches print ranges in meters,
separated by semicolons:

```text
T0,mask:F,seq:7,beacon:12,range_m:(1.25;0.34;2.17;1.22),ancid:(0,1,2,3),device_role:tag,device_index:0,device_radio_id:176
T0,mask:F,seq:7,fail:2,range_m:(1.25;0.34;2.17;1.22),ancid:(0,1,2,3),device_role:anchor,device_index:3,device_radio_id:163
```

The parser also remains compatible with the original format, where ranges are
comma-separated integer centimeters (for example
`range:(125,0,234),ancid:(0,-1,2)`). Semicolon-separated range values are
interpreted as meters; the explicit `range_m:` label also indicates meters.
Only anchors marked present by both the mask and anchor ID list are published.
Therefore the example with four responding anchors uses
mask `F`; mask `5` would mark only anchor indices 0 and 2 as valid.
New firmware appends `device_role`, `device_index`, and `device_radio_id`, so the
node can distinguish a connected Tag from an Anchor and identify that board.
Legacy lines without these fields are treated as Tag records.
The Anchor output's `fail:<count>` is preserved in `/uwb/measurements`.
The Tag-to-Anchor UWB `REPORT` frame now also carries each of the 8 ranges as a
32-bit IEEE-754 float in meters, little-endian. This changes the binary REPORT
payload from 2-byte integer centimeters to 4-byte float meters. The REPORT data
is 43 bytes before the radio FCS: the existing 11-byte header followed by eight
float values; transmitted length is 45 bytes including the 2-byte FCS. Update
the Tag and Anchor firmware together; old and new firmware are not
wire-compatible. The Tag derives each meter value directly from its DW3000
time-of-flight calculation before sending the report.
In this sketch, `T0` is the configured tag index (`TAG_INDEX`), and the printed
anchor IDs are indices `0..7`, not radio addresses `0xA0..0xA7`. The normalized
measurement message includes both `anchor_id` (index) and `anchor_radio_id`
(radio address `0xA0 + index`). It also includes the connected board identity
as `device_role`, `device_index`, and `device_radio_id` (`0xB0 + tag index` for
the current Tag sketch, `0xA0 + anchor index` for the Anchor firmware). Radio
IDs are represented as decimal integers in Serial and ROS JSON.
The current firmware output does **not** contain DW3000 quality metrics,
per-anchor ranging timestamps, or an application payload; those fields remain
absent until firmware sends them.

For forward-compatible firmware, the node also accepts a JSON line, for example:

```json
{"protocol":"uwb_json_v1","device_role":"anchor","device_index":3,"device_radio_id":163,"tag_id":0,"sequence":7,"timestamp_ms":1234,"measurements":[{"anchor_id":2,"range_m":1.25,"valid":true,"quality":{"fp_power_dbm":-75.2,"rx_power_dbm":-72.8,"multipath_indicator":0.08},"timestamps":{"poll_tx":100000,"poll_rx":120000,"response_tx":125000,"response_rx":145000},"clock_offset":0.0001}],"payload":{"temperature_c":24.5}}
```

DW3000 timestamps and device uptime are forwarded as metadata, not used as the
ROS receive timestamp. Payload can carry sensor values or opaque application
data. This package receives and publishes payload; it does not transmit ROS
commands back to the ESP32.

## Build and run (Ubuntu / ROS 2)

Place this package under a ROS 2 workspace's `src` directory:

```bash
mkdir -p ~/uwb_ws/src
cp -r "code ros/uwb_serial_driver" ~/uwb_ws/src/
cd ~/uwb_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select uwb_serial_driver
source install/setup.bash
```

Run with the serial settings supplied as ROS parameters:

```bash
ros2 run uwb_serial_driver uwb_serial_node --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=115200 \
  -p update_rate_hz:=50.0 \
  -p reconnect_interval_sec:=2.0 \
  -p data_timeout_sec:=3.0
```

The serial implementation uses POSIX `termios`, so build and run it on Linux
(for example, the Ubuntu environment where ROS 2 is installed). Add the user to
the system's serial-port access group if opening `/dev/ttyUSB0` is denied.
Close Arduino Serial Monitor before starting the node because a serial port
cannot normally be opened by both programs at once. The existing ESP32
`UART_init()` uses 115200 baud.

`update_rate_hz` controls how often the ROS node checks and drains the serial
buffer; it does not change the UWB measurement rate on ESP32.
Both Tag and Anchor sketches must be updated and use the same float-in-meters
REPORT format over the UWB air interface.

Run one node instance per connected ESP32 Serial port. Give each process a
distinct ROS node name; firmware identity fields determine the Tag/Anchor
topics:

```bash
ros2 run uwb_serial_driver uwb_serial_node --ros-args \
  -r __node:=uwb_anchor_3 \
  -p port:=/dev/ttyUSB1 \
  -p baudrate:=115200
```

## Tests

```bash
colcon test --packages-select uwb_serial_driver
colcon test-result --verbose
```
