void setup() {
  Serial.begin(115200);
  delay(1000);

  // Enable the LiDAR motor
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);

  // UART connected to D0 (RX) and D1 (TX)
  Serial0.begin(115200);

  Serial.println("RPLidar diagnostic ready");
  Serial.println("D6 motor enable = HIGH");
  Serial.println("Commands: i=Info, h=Health, s=Scan, x=Stop");
}

void loop() {
  // Send commands from USB Serial Monitor
  while (Serial.available()) {
    char c = Serial.read();

    if (c == 'i') {
      Serial.println("\nTX: A5 50 (GET_INFO)");
      Serial0.write((uint8_t)0xA5);
      Serial0.write((uint8_t)0x50);
    }
    else if (c == 'h') {
      Serial.println("\nTX: A5 52 (GET_HEALTH)");
      Serial0.write((uint8_t)0xA5);
      Serial0.write((uint8_t)0x52);
    }
    else if (c == 's') {
      Serial.println("\nTX: A5 20 (SCAN)");
      Serial0.write((uint8_t)0xA5);
      Serial0.write((uint8_t)0x20);
    }
    else if (c == 'x') {
      Serial.println("\nTX: A5 25 (STOP)");
      Serial0.write((uint8_t)0xA5);
      Serial0.write((uint8_t)0x25);
    }
  }

  // Read LiDAR responses from the SAME UART
  while (Serial0.available()) {
    uint8_t b = Serial0.read();

    Serial.print("RX: ");
    if (b < 0x10) Serial.print('0');
    Serial.println(b, HEX);
  }
}