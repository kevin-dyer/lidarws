# RPLIDAR A1 UART passthrough test

This test sketch presents the Nano ESP32 as a transparent USB-serial bridge to
an RPLIDAR A1. It forwards raw bytes in both directions at 115200 baud:

- Host USB CDC TX to the sensor UART RX
- Sensor UART TX to host USB CDC RX

The sketch intentionally sends no status text on USB; even a startup message
would corrupt the RPLIDAR command/data stream. It does not run the tilt-stage
ASCII command protocol. Do not run the tilt coordinator against this sketch:
both protocols would share the same USB stream, and text tilt replies would be
indistinguishable from binary LiDAR output.

## Wiring

| RPLIDAR A1 signal | Nano ESP32 pin |
| --- | --- |
| UART TX | D0 / `Serial1` RX |
| UART RX | D1 / `Serial1` TX |
| DTR / motor control | D6 |
| Ground | GND |
| Sensor power | Separate regulated supply per the A1 documentation |

Connect grounds together. Confirm the UART and DTR logic levels for the exact
sensor revision before powering it; protect the ESP32's 3.3 V GPIOs from any
higher voltage. Do not power the LiDAR motor from a Nano ESP32 GPIO.

D6 is held HIGH for this test firmware. A direct test with the connected
A1M8 returned a valid 27-byte device-info response while D6 was HIGH and no
response while D6 was LOW. D6 is deliberately not tied to USB CDC DTR: the
Slamtec ROS driver changes the host DTR line as it opens the serial port, and
that transition interfered with sensor responses during testing. This means
the RPLIDAR motor/control line is not automatically switched off when the
host closes the serial port.

## Build and run

Select **Arduino Nano ESP32** with its default **Normal mode (TinyUSB)** USB
setting, then compile and upload `rplidar_uart_test.ino`. From this workspace,
the sketch can be compiled with:

```bash
arduino-cli compile --fqbn arduino:esp32:nano_nora \
  firmware/rplidar_uart_test
```

Connect the Arduino by USB, identify its serial device (commonly
`/dev/ttyACM0`), and launch only the RPLIDAR driver:

```bash
source install/setup.bash
ros2 launch rplidar_driver rplidar.launch.py \
  serial_port:=/dev/ttyACM0 serial_baudrate:=115200 \
  serial_dtr_asserted:=true
```

Close the Arduino IDE Serial Monitor and any other program using the serial
port before launching the ROS driver; only one process can own the port.

## UART loopback check

To isolate the bridge from the sensor, disconnect the LiDAR and temporarily
jumper Nano ESP32 D1 (TX) to D0 (RX), then send any bytes through the USB
serial port. The same bytes should return to the host; no firmware change is
needed for loopback. Remove the jumper and reconnect the LiDAR afterward. This
check verifies the USB CDC and UART directions, but does not verify sensor
power, sensor UART levels, or D6 motor-control behavior.
