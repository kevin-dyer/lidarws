#include <Arduino.h>

// RPLIDAR A1 UART bridge test for Arduino Nano ESP32.
//
// USB CDC is a transparent byte stream to/from the host. Serial1 carries the
// RPLIDAR A1 UART protocol. No text is written to USB because it would corrupt
// the sensor protocol stream.

const uint32_t LIDAR_BAUD = 115200;
const uint8_t LIDAR_RX_PIN = D0;
const uint8_t LIDAR_TX_PIN = D1;
const uint8_t LIDAR_DTR_PIN = D6;
const uint8_t LIDAR_DTR_ACTIVE_LEVEL = HIGH;
const size_t BRIDGE_BUFFER_SIZE = 256;

uint8_t hostToLidarBuffer[BRIDGE_BUFFER_SIZE];
uint8_t lidarToHostBuffer[BRIDGE_BUFFER_SIZE];

void relayHostToLidar() {
  size_t count = 0;
  while (count < sizeof(hostToLidarBuffer) && Serial.available() > 0) {
    const int value = Serial.read();
    if (value < 0) {
      break;
    }
    hostToLidarBuffer[count++] = static_cast<uint8_t>(value);
  }
  if (count > 0) {
    Serial1.write(hostToLidarBuffer, count);
  }
}

void relayLidarToHost() {
  size_t count = 0;
  while (count < sizeof(lidarToHostBuffer) && Serial1.available() > 0) {
    const int value = Serial1.read();
    if (value < 0) {
      break;
    }
    lidarToHostBuffer[count++] = static_cast<uint8_t>(value);
  }
  if (count > 0) {
    Serial.write(lidarToHostBuffer, count);
  }
}

void setup() {
  pinMode(LIDAR_DTR_PIN, OUTPUT);
  digitalWrite(LIDAR_DTR_PIN, LIDAR_DTR_ACTIVE_LEVEL);

  Serial.setRxBufferSize(4096);
  Serial.begin(LIDAR_BAUD);
  Serial1.setRxBufferSize(4096);
  Serial1.begin(
      LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
}

void loop() {
  relayHostToLidar();
  relayLidarToHost();
}
