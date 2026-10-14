# Nano ESP32 Wi-Fi/UDP RPLIDAR and tilt bridge

The `tilt_lidar_3d.launch.py` flow uses Wi-Fi UDP between the Nano ESP32 and the
host. The board joins configured home Wi-Fi while also hosting a fallback
access point for operation without a shared network. A host-side UDP bridge
creates separate PTYs for the RPLIDAR UART byte stream and tilt-stage text
protocol, keeping binary LiDAR data separate from tilt telemetry.

## Network configuration

Copy `firmware/tilt_stage/wifi_config.example.h` to
`firmware/tilt_stage/wifi_config.h`, then set the home Wi-Fi SSID/password and
a private fallback AP password. The real `wifi_config.h` is gitignored; do not
commit Wi-Fi credentials. If the local file is absent, firmware starts the
fallback AP only, using SSID `TiltLidar-Bridge` and password `tilt-3d-scan`.

On boot, the board starts both its fallback AP and a station connection to the
configured home Wi-Fi. The host can discover the board over the shared WLAN.
When offline, connect the host computer to `TiltLidar-Bridge` (or your
configured fallback SSID) and launch with `board_ip:=192.168.4.1`.
The mobile host can also provision home Wi-Fi over the fallback connection.
It sends a control-channel `WIFI` payload containing one-byte SSID and
password lengths followed by those UTF-8 values. The board stores them in
ESP32 NVS, acknowledges with `WIFI_OK`, and joins the configured network on
this boot and subsequent boots. The compile-time values in `wifi_config.h`
remain the initial defaults when no credentials have been provisioned.
A control-channel `STATUS` request returns `STATUS,<0|1>,<ip>` so the host can
verify that the board actually joined the home network and learn its address.

## Wiring

Use the RPLIDAR A1's UART interface, not its USB adapter:

| RPLIDAR A1 signal | Nano ESP32 connection |
| --- | --- |
| UART TX | D0 / `Serial1` RX |
| UART RX | D1 / `Serial1` TX |
| DTR/control input | D6 |
| Ground | GND |
| Sensor power | Separate regulated 5 V supply |

Join the sensor supply ground to the Nano ESP32 ground. Do not power the
RPLIDAR motor from the Nano ESP32 3.3 V pin. Verify the sensor connector pinout
and UART logic voltage for the specific A1 revision; use a level shifter if a
signal exceeds the ESP32's 3.3 V GPIO limit. Firmware starts D6 high, asserts it
for RPLIDAR commands, and deasserts it after a forwarded STOP command. The ROS
launch keeps the driver's DTR setting asserted; PTY DTR does not control the
physical D6 output. The USB bridge also asserts host CDC DTR when opening the
Nano connection so the firmware's CDC transport can respond.

Keep the existing stepper driver and Hall sensor connections unchanged. Ensure
the selected Nano ESP32 pin labels for D0, D1, and D2 are not reassigned in the
Arduino IDE board configuration.

## Software setup

1. Install the ESP32 Arduino core and `AccelStepper`.
   Select **Arduino Nano ESP32**, then make a local Wi-Fi config:

   ```bash
   cp firmware/tilt_stage/wifi_config.example.h \
     firmware/tilt_stage/wifi_config.h
   ```

   Edit `wifi_config.h` with the home Wi-Fi and a private fallback AP password.
   Compile/upload `firmware/tilt_stage/tilt_stage.ino`:
2. Build and upload the combined firmware:

   ```bash
   arduino-cli compile --fqbn arduino:esp32:nano_nora firmware/tilt_stage
   arduino-cli upload --fqbn arduino:esp32:nano_nora \
     --port /dev/ttyACM0 firmware/tilt_stage
   ```

3. Build the ROS workspace and launch the scan:

   ```bash
   colcon build --packages-up-to tilt_coordinator
   source install/setup.bash
   ros2 launch tilt_coordinator tilt_lidar_3d.launch.py
   ```

The launch starts `udp_serial_bridge`, which creates
`/tmp/tilt_lidar_udp/lidar` and `/tmp/tilt_lidar_udp/tilt`, then waits for the
UDP PING/PONG discovery exchange before starting the RPLIDAR driver and capture
nodes. The default target is UDP broadcast (`255.255.255.255`) plus the
fallback AP address. When directly connected to the AP, set
`board_ip:=192.168.4.1`. The board and host default to UDP ports 4210 and 4211;
override them with `board_udp_port:=...` and `host_udp_port:=...`. PTY paths
can be changed with `lidar_serial_port:=...` and `tilt_serial_port:=...`.

Each UDP datagram carries one framed channel with a CRC-16 checksum. LiDAR
bytes and tilt text retain their existing serial protocols inside separate
channels. UDP does not retransmit lost packets; keep the board and host on a
reliable local Wi-Fi link, and avoid routing this unauthenticated control/data
protocol onto untrusted networks. The USB-only `rplidar_uart_test` firmware is
not compatible with this UDP 3D launch.
