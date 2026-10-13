# Nano ESP32 USB RPLIDAR and tilt bridge

The `tilt_lidar_3d.launch.py` flow uses the Nano ESP32 as a USB serial
multiplexer. The host bridge owns the board's USB serial device and creates
separate PTYs for the RPLIDAR UART byte stream and the tilt-stage text
protocol. The RPLIDAR driver and tilt coordinator use those PTYs independently,
so binary scan packets are never mixed with tilt telemetry.

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
   Select **Arduino Nano ESP32** and compile/upload
   `firmware/tilt_stage/tilt_stage.ino`.
2. Build the ROS workspace and launch the scan:

   ```bash
   colcon build --packages-up-to tilt_coordinator
   source install/setup.bash
   ros2 launch tilt_coordinator tilt_lidar_3d.launch.py device_port:=/dev/ttyACM0
   ```

The launch starts `usb_serial_bridge`, which creates
`/tmp/tilt_lidar_usb/lidar` and `/tmp/tilt_lidar_usb/tilt`, then waits for a
USB mux PING/PONG handshake before starting the RPLIDAR driver and capture
nodes. Set `device_port:=...` if the board is not `/dev/ttyACM0`. The PTY
locations can be changed with `lidar_serial_port:=...` and
`tilt_serial_port:=...`.

The USB connection carries framed channels with CRC-16 checksums. LiDAR bytes
and tilt text retain their existing serial protocols inside separate channels.
The `rplidar_uart_test` firmware is a LiDAR-only passthrough and cannot run the
combined 3D launch; upload `tilt_stage.ino` for this setup.
