#include <AccelStepper.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <Arduino.h>

const uint8_t STEP_PIN = 8;
const uint8_t DIR_PIN = 7;
const uint8_t ENABLE_PIN = 12;
const uint8_t MS1_PIN = 11;
const uint8_t MS2_PIN = 10;
const uint8_t HALL_PIN = A4;
const uint8_t LIDAR_RX_PIN = D0;
const uint8_t LIDAR_TX_PIN = D1;
const uint8_t LIDAR_DTR_PIN = D6;
const uint8_t LIDAR_DTR_ACTIVE_LEVEL = HIGH;
const uint8_t MUX_MAGIC_0 = 0xA5;
const uint8_t MUX_MAGIC_1 = 0x5A;
const uint8_t MUX_CHANNEL_LIDAR = 1;
const uint8_t MUX_CHANNEL_TILT = 2;
const uint8_t MUX_CHANNEL_CONTROL = 0x7F;
const size_t MUX_MAX_PAYLOAD = 180;
const size_t TILT_RX_BUFFER_SIZE = 512;

const int HALF_STEP = 2;
const int QUARTER_STEP = 4;
const int EIGHTH_STEP = 8;
const int SIXTEENTH_STEP = 16;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

const float MOTOR_STEPS_PER_REV = 200.0;
const float GEAR_RATIO = 7.6875;  // 123 / 16
const float MAX_TILT_SPEED_DEG_S = 10.0;
const float HOMING_SPEED_DEG_S = 40.0;
const float MAX_ANGLE_DEG = 45.0;
const float DEFAULT_SWEEP_START_DEG = -15.0;
const float DEFAULT_SWEEP_STOP_DEG = 15.0;
const unsigned long SWEEP_SAMPLE_INTERVAL_US = 10000UL;

uint8_t tiltRxBuffer[TILT_RX_BUFFER_SIZE];
size_t tiltRxHead = 0;
size_t tiltRxTail = 0;
size_t tiltRxCount = 0;
bool tiltRxOverflow = false;
uint8_t lidarTxBuffer[MUX_MAX_PAYLOAD];
size_t lidarTxLength = 0;
uint8_t previousLidarCommandByte = 0;
uint8_t muxChannel = 0;
uint16_t muxLength = 0;
uint16_t muxReceivedCrc = 0;
uint8_t muxPayload[MUX_MAX_PAYLOAD];
size_t muxPayloadIndex = 0;
uint8_t muxState = 0;

uint16_t crc16Update(uint16_t crc, uint8_t value) {
  crc ^= (uint16_t)value << 8;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

uint16_t frameCrc(uint8_t channel, uint16_t length, const uint8_t *payload) {
  uint16_t crc = 0xFFFF;
  crc = crc16Update(crc, channel);
  crc = crc16Update(crc, length & 0xFF);
  crc = crc16Update(crc, length >> 8);
  for (uint16_t i = 0; i < length; ++i) {
    crc = crc16Update(crc, payload[i]);
  }
  return crc;
}

void sendMuxFrame(uint8_t channel, const uint8_t *payload, uint16_t length) {
  if (length > MUX_MAX_PAYLOAD) {
    return;
  }
  const uint16_t crc = frameCrc(channel, length, payload);
  Serial.write(MUX_MAGIC_0);
  Serial.write(MUX_MAGIC_1);
  Serial.write(channel);
  Serial.write(length & 0xFF);
  Serial.write(length >> 8);
  if (length > 0) {
    Serial.write(payload, length);
  }
  Serial.write(crc & 0xFF);
  Serial.write(crc >> 8);
}

class MuxTiltStream : public Stream {
 public:
  int available() override { return tiltRxCount; }

  int read() override {
    if (tiltRxCount == 0) {
      return -1;
    }
    const uint8_t value = tiltRxBuffer[tiltRxTail];
    tiltRxTail = (tiltRxTail + 1) % TILT_RX_BUFFER_SIZE;
    --tiltRxCount;
    return value;
  }

  int peek() override {
    return tiltRxCount == 0 ? -1 : tiltRxBuffer[tiltRxTail];
  }

  void flush() override { sendPending(); }

  size_t write(uint8_t value) override {
    pending_[pendingLength_++] = value;
    if (value == '\n' || pendingLength_ == sizeof(pending_)) {
      sendPending();
    }
    return 1;
  }

 private:
  void sendPending() {
    if (pendingLength_ > 0) {
      sendMuxFrame(MUX_CHANNEL_TILT, pending_, pendingLength_);
      pendingLength_ = 0;
    }
  }

  uint8_t pending_[MUX_MAX_PAYLOAD];
  size_t pendingLength_ = 0;
};

MuxTiltStream TiltSerial;

void enqueueTiltBytes(const uint8_t *payload, uint16_t length) {
  for (uint16_t i = 0; i < length; ++i) {
    if (tiltRxCount == TILT_RX_BUFFER_SIZE) {
      tiltRxOverflow = true;
      return;
    }
    tiltRxBuffer[tiltRxHead] = payload[i];
    tiltRxHead = (tiltRxHead + 1) % TILT_RX_BUFFER_SIZE;
    ++tiltRxCount;
  }
}

void forwardLidarCommand(uint8_t value) {
  if (previousLidarCommandByte == 0xA5 &&
      (value == 0x20 || value == 0x21 || value == 0x40 ||
       value == 0x50 || value == 0x52)) {
    digitalWrite(LIDAR_DTR_PIN, LIDAR_DTR_ACTIVE_LEVEL);
  }
  Serial1.write(value);
  if (previousLidarCommandByte == 0xA5 && value == 0x25) {
    Serial1.flush();
    digitalWrite(LIDAR_DTR_PIN, LOW);
  }
  previousLidarCommandByte = value;
}

void handleMuxFrame() {
  if (frameCrc(muxChannel, muxLength, muxPayload) != muxReceivedCrc) {
    return;
  }
  if (muxChannel == MUX_CHANNEL_LIDAR) {
    for (uint16_t i = 0; i < muxLength; ++i) {
      forwardLidarCommand(muxPayload[i]);
    }
    return;
  }
  if (muxChannel == MUX_CHANNEL_TILT) {
    enqueueTiltBytes(muxPayload, muxLength);
    return;
  }
  if (muxChannel == MUX_CHANNEL_CONTROL && muxLength == 4 &&
      memcmp(muxPayload, "PING", 4) == 0) {
    sendMuxFrame(MUX_CHANNEL_CONTROL,
                 reinterpret_cast<const uint8_t *>("PONG"), 4);
  }
}

void processMuxByte(uint8_t value) {
  switch (muxState) {
    case 0:
      muxState = value == MUX_MAGIC_0 ? 1 : 0;
      break;
    case 1:
      muxState = value == MUX_MAGIC_1 ? 2 : (value == MUX_MAGIC_0 ? 1 : 0);
      break;
    case 2:
      muxChannel = value;
      muxState = 3;
      break;
    case 3:
      muxLength = value;
      muxState = 4;
      break;
    case 4:
      muxLength |= (uint16_t)value << 8;
      if (muxLength > MUX_MAX_PAYLOAD) {
        muxState = value == MUX_MAGIC_0 ? 1 : 0;
      } else {
        muxPayloadIndex = 0;
        muxState = muxLength == 0 ? 6 : 5;
      }
      break;
    case 5:
      muxPayload[muxPayloadIndex++] = value;
      if (muxPayloadIndex == muxLength) {
        muxState = 6;
      }
      break;
    case 6:
      muxReceivedCrc = value;
      muxState = 7;
      break;
    case 7:
      muxReceivedCrc |= (uint16_t)value << 8;
      handleMuxFrame();
      muxState = 0;
      break;
    default:
      muxState = 0;
      break;
  }
}

void processUsbMux() {
  while (Serial.available() > 0) {
    const int value = Serial.read();
    if (value >= 0) {
      processMuxByte((uint8_t)value);
    }
  }
}

void serviceLidarBridge() {
  processUsbMux();
  size_t count = 0;
  while (count < sizeof(lidarTxBuffer) && Serial1.available() > 0) {
    const int value = Serial1.read();
    if (value < 0) {
      break;
    }
    lidarTxBuffer[count++] = (uint8_t)value;
  }
  if (count > 0) {
    sendMuxFrame(MUX_CHANNEL_LIDAR, lidarTxBuffer, count);
  }
}

void runToPositionWithBridge(long target) {
  stepper.moveTo(target);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    serviceLidarBridge();
  }
}

enum TiltMotionState {
  TILT_IDLE,
  TILT_MOVING_TO_SWEEP_START,
  TILT_SWEEPING,
  TILT_RETURNING_HOME
};

int stepMode = SIXTEENTH_STEP;
float stepsPerDegree = 0.0;
float maxTiltSpeedDegS = MAX_TILT_SPEED_DEG_S;
// float hallOffsetDeg = 37.0;
float hallOffsetDeg = 2.0;
bool invertTilt = false;
bool enabled = true;
TiltMotionState tiltMotionState = TILT_IDLE;
long sweepStopSteps = 0;
unsigned long lastSweepSampleMicros = 0;

float calcStepsPerDegree() {
  return (MOTOR_STEPS_PER_REV * stepMode * GEAR_RATIO) / 360.0;
}

float angleToSteps(float angle) {
  return angle * stepsPerDegree;
}

float stepsToAngle(long steps) {
  return (float)steps / stepsPerDegree;
}

bool hallTriggered() {
  return digitalRead(HALL_PIN) == LOW;
}

int hallStateCode() {
  return hallTriggered() ? 0 : 1;
}

void applyMotorSpeed() {
  stepper.setMaxSpeed(angleToSteps(maxTiltSpeedDegS));
}

void setStepMode(int newMode) {
  if (newMode == HALF_STEP) {
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, LOW);
  } else if (newMode == QUARTER_STEP) {
    digitalWrite(MS1_PIN, LOW);
    digitalWrite(MS2_PIN, HIGH);
  } else if (newMode == EIGHTH_STEP) {
    digitalWrite(MS1_PIN, LOW);
    digitalWrite(MS2_PIN, LOW);
  } else if (newMode == SIXTEENTH_STEP) {
    digitalWrite(MS1_PIN, HIGH);
    digitalWrite(MS2_PIN, HIGH);
  } else {
    TiltSerial.println(F("Invalid mode. Enter 2, 4, 8 or 16"));
    return;
  }

  const float previousStepsPerDegree = stepsPerDegree;
  if (previousStepsPerDegree > 0.0) {
    stepper.setCurrentPosition((long)lround(stepper.currentPosition() * ((float)newMode / (float)stepMode)));
  }

  stepMode = newMode;
  stepsPerDegree = calcStepsPerDegree();
  applyMotorSpeed();

  TiltSerial.print(F("set to "));
  TiltSerial.print(stepMode);
  TiltSerial.println(F(" step mode."));
}

void setSpeed(float speedDegPerSec) {
  maxTiltSpeedDegS = speedDegPerSec;
  applyMotorSpeed();
  TiltSerial.print(F("Max tilt speed: "));
  TiltSerial.print(maxTiltSpeedDegS, 1);
  TiltSerial.println(F(" deg/s"));
}

void enableSteppers() {
  if (enabled) {
    digitalWrite(ENABLE_PIN, HIGH);
    enabled = false;
    TiltSerial.println(F("Disabled"));
  } else {
    digitalWrite(ENABLE_PIN, LOW);
    enabled = true;
    TiltSerial.println(F("Enabled"));
  }
}

void queryTiltAngle() {
  TiltSerial.write('T');
  TiltSerial.print(stepsToAngle(stepper.currentPosition()), 4);
  TiltSerial.write('\n');
}

void queryTiltSteps() {
  TiltSerial.write('Z');
  TiltSerial.print(stepper.currentPosition());
  TiltSerial.write('\n');
}

void testHallSensor() {
  TiltSerial.write('L');
  TiltSerial.print(hallTriggered() ? 0 : 1);
  TiltSerial.write('\n');
}

void moveToAbsoluteAngle(float angle) {
  if (fabs(angle) > MAX_ANGLE_DEG) {
    TiltSerial.println(F("LIMIT"));
    return;
  }
  const long target = lround(angleToSteps(angle));
  runToPositionWithBridge(target);
  queryTiltAngle();
}

bool parseSweepAngles(char *payload, float &startAngle, float &stopAngle) {
  if (payload[0] == '\0') {
    startAngle = DEFAULT_SWEEP_START_DEG;
    stopAngle = DEFAULT_SWEEP_STOP_DEG;
    return true;
  }

  char *separator = strchr(payload, ',');
  if (separator == NULL) {
    return false;
  }
  *separator = '\0';

  char *startEnd = NULL;
  char *stopEnd = NULL;
  startAngle = strtod(payload, &startEnd);
  stopAngle = strtod(separator + 1, &stopEnd);
  if (startEnd == payload || stopEnd == separator + 1) {
    return false;
  }

  while (isspace((unsigned char)*startEnd)) {
    ++startEnd;
  }
  while (isspace((unsigned char)*stopEnd)) {
    ++stopEnd;
  }
  return *startEnd == '\0' && *stopEnd == '\0';
}

void startTiltSweep(float startAngle, float stopAngle) {
  if (tiltMotionState != TILT_IDLE) {
    TiltSerial.println(F("ERR BUSY"));
    return;
  }
  if (!isfinite(startAngle) || !isfinite(stopAngle) || fabs(startAngle) > MAX_ANGLE_DEG || fabs(stopAngle) > MAX_ANGLE_DEG || startAngle == stopAngle) {
    TiltSerial.println(F("ERR SWEEP LIMIT"));
    return;
  }

  sweepStopSteps = lround(angleToSteps(stopAngle));
  stepper.setMaxSpeed(angleToSteps(MAX_TILT_SPEED_DEG_S));
  tiltMotionState = TILT_MOVING_TO_SWEEP_START;
  stepper.moveTo(lround(angleToSteps(startAngle)));
}

void stopTiltSweepAndReturnHome() {
  tiltMotionState = TILT_RETURNING_HOME;
  stepper.setMaxSpeed(angleToSteps(MAX_TILT_SPEED_DEG_S));
  stepper.moveTo(0);
}

void emitSweepSample() {
  TiltSerial.print(micros());
  TiltSerial.write(',');
  TiltSerial.print(stepper.currentPosition());
  TiltSerial.write('\n');
}

void updateTiltMotion() {
  if (tiltMotionState == TILT_MOVING_TO_SWEEP_START && stepper.distanceToGo() == 0) {
    applyMotorSpeed();
    stepper.moveTo(sweepStopSteps);
    tiltMotionState = TILT_SWEEPING;
    lastSweepSampleMicros = micros();
    emitSweepSample();
    return;
  }

  if (tiltMotionState == TILT_SWEEPING) {
    if (stepper.distanceToGo() == 0) {
      emitSweepSample();
      tiltMotionState = TILT_RETURNING_HOME;
      stepper.setMaxSpeed(angleToSteps(MAX_TILT_SPEED_DEG_S));
      stepper.moveTo(0);
      return;
    }

    const unsigned long now = micros();
    if (now - lastSweepSampleMicros >= SWEEP_SAMPLE_INTERVAL_US) {
      lastSweepSampleMicros = now;
      emitSweepSample();
    }
    return;
  }

  if (tiltMotionState == TILT_RETURNING_HOME && stepper.distanceToGo() == 0) {
    tiltMotionState = TILT_IDLE;
    applyMotorSpeed();
    TiltSerial.println(F("X"));
  }
}

void printStatus() {
  TiltSerial.println(F("\n=== TILT MOUNT STATUS ==="));
  TiltSerial.print(F("Enable state: "));
  TiltSerial.println(enabled ? F("Enabled") : F("Disabled"));
  TiltSerial.print(F("Step mode: "));
  TiltSerial.print(stepMode);
  TiltSerial.println(F("x"));
  TiltSerial.print(F("Current tilt angle: "));
  TiltSerial.print(stepsToAngle(stepper.currentPosition()), 3);
  TiltSerial.println(F(" deg"));
  TiltSerial.print(F("Current tilt steps: "));
  TiltSerial.println(stepper.currentPosition());
  TiltSerial.print(F("Max tilt speed: "));
  TiltSerial.print(maxTiltSpeedDegS, 2);
  TiltSerial.println(F(" deg/s"));
  TiltSerial.print(F("Hall sensor state: "));
  TiltSerial.println(hallTriggered() ? 0 : 1);
  TiltSerial.print(F("Hall offset: "));
  TiltSerial.print(hallOffsetDeg, 3);
  TiltSerial.println(F(" deg"));
}

void invertTiltDirection(bool invert) {
  invertTilt = invert;
  stepper.setPinsInverted(invert, false, false);
  TiltSerial.print(F("tilt inversion: "));
  TiltSerial.println(invert ? 1 : 0);
}

bool findHome() {
  bool startedOnSensor = hallTriggered();
  float originalMaxSpeed = maxTiltSpeedDegS;
  maxTiltSpeedDegS = HOMING_SPEED_DEG_S;
  applyMotorSpeed();

  float firstTransition = 0.0;
  float secondTransition = 0.0;

  if (startedOnSensor) {
    TiltSerial.println(F("Phase 1: Searching for Low->High transition..."));
    stepper.moveTo(angleToSteps(-70.0));
    int prevHall = hallStateCode();
    bool found = false;
    while (stepper.run()) {
      serviceLidarBridge();
      const int currentHall = hallStateCode();
      if (currentHall != prevHall && prevHall == 0) {
        firstTransition = stepsToAngle(stepper.currentPosition());
        found = true;
        break;
      }
      prevHall = currentHall;
    }
    if (!found) {
      maxTiltSpeedDegS = originalMaxSpeed;
      applyMotorSpeed();
      TiltSerial.println(F("Error: Could not find Low->High transition"));
      return false;
    }

    TiltSerial.println(F("Phase 2: Reversing for second transition..."));
    const float phase2Target = firstTransition + 70.0;
    stepper.moveTo(angleToSteps(phase2Target));
    prevHall = hallStateCode();
    const float phase2Start = firstTransition;
    bool secondFound = false;
    while (stepper.run()) {
      serviceLidarBridge();
      const int currentHall = hallStateCode();
      const float currentPosition = stepsToAngle(stepper.currentPosition());
      const float distance = fabs(currentPosition - phase2Start);
      if (distance >= 10.0 && currentHall != prevHall && prevHall == 0) {
        secondTransition = currentPosition;
        secondFound = true;
        break;
      }
      prevHall = currentHall;
    }
    if (!secondFound) {
      maxTiltSpeedDegS = originalMaxSpeed;
      applyMotorSpeed();
      TiltSerial.println(F("Error: Could not find second Low->High transition"));
      return false;
    }
  } else {
    TiltSerial.println(F("Phase 1: Searching for High->Low transition..."));
    stepper.moveTo(angleToSteps(-70.0));
    int prevHall = hallStateCode();
    bool found = false;
    while (stepper.run()) {
      serviceLidarBridge();
      const int currentHall = hallStateCode();
      if (currentHall != prevHall && prevHall == 1) {
        firstTransition = stepsToAngle(stepper.currentPosition());
        found = true;
        break;
      }
      prevHall = currentHall;
    }
    if (!found) {
      maxTiltSpeedDegS = originalMaxSpeed;
      applyMotorSpeed();
      TiltSerial.println(F("Error: Could not find High->Low transition"));
      return false;
    }

    TiltSerial.println(F("Phase 2: Continuing for second transition..."));
    const float phase2Target = firstTransition - 70.0;
    stepper.moveTo(angleToSteps(phase2Target));
    prevHall = hallStateCode();
    const float phase2Start = firstTransition;
    bool secondFound = false;
    while (stepper.run()) {
      serviceLidarBridge();
      const int currentHall = hallStateCode();
      const float currentPosition = stepsToAngle(stepper.currentPosition());
      const float distance = fabs(currentPosition - phase2Start);
      if (distance >= 10.0 && currentHall != prevHall && prevHall == 0) {
        secondTransition = currentPosition;
        secondFound = true;
        break;
      }
      prevHall = currentHall;
    }
    if (!secondFound) {
      maxTiltSpeedDegS = originalMaxSpeed;
      applyMotorSpeed();
      TiltSerial.println(F("Error: Could not find second High->Low transition"));
      return false;
    }
  }

  const float homeAngle = (firstTransition + secondTransition) / 2.0;
  TiltSerial.print(F("Center position: "));
  TiltSerial.print(homeAngle, 2);
  TiltSerial.println(F(" deg"));

  runToPositionWithBridge(lround(angleToSteps(homeAngle)));

  if (hallOffsetDeg != 0.0) {
    const float target = homeAngle + hallOffsetDeg;
    runToPositionWithBridge(lround(angleToSteps(target)));
  }

  stepper.setCurrentPosition(0);
  stepper.moveTo(0);

  maxTiltSpeedDegS = originalMaxSpeed;
  applyMotorSpeed();
  TiltSerial.println(F("Homing complete"));
  return true;
}

void serialParse() {
  if (TiltSerial.available() == 0) {
    return;
  }

  char instruction = TiltSerial.read();
  if (instruction == '\n' || instruction == '\r') {
    return;
  }

  if (instruction == '+') {
    while (TiltSerial.available()) {
      TiltSerial.read();
    }
    return;
  }

  char payload[32];
  memset(payload, 0, sizeof(payload));
  size_t index = 0;

  delay(2);
  while (TiltSerial.available() > 0 && index < sizeof(payload) - 1) {
    char c = TiltSerial.read();
    if (c == '\n' || c == '\r') {
      break;
    }
    payload[index++] = c;
  }

  const float value = atof(payload);
  const long integerValue = atoi(payload);

  if (tiltMotionState != TILT_IDLE && instruction != 'X') {
    TiltSerial.println(F("ERR BUSY"));
    return;
  }

  switch (instruction) {
    case 'T':
      queryTiltAngle();
      break;
    case 'Z':
      queryTiltSteps();
      break;
    case 'L':
      testHallSensor();
      break;
    case 'M':
      moveToAbsoluteAngle(value);
      break;
    case 'C':
      {
        float startAngle = 0.0;
        float stopAngle = 0.0;
        if (!parseSweepAngles(payload, startAngle, stopAngle)) {
          TiltSerial.println(F("ERR C expected startAngle,stopAngle"));
          break;
        }
        startTiltSweep(startAngle, stopAngle);
        break;
      }
    case 'X':
      stopTiltSweepAndReturnHome();
      break;
    case 't':
      stepper.moveTo(lround(angleToSteps(value)));
      break;
    case 'S':
      setSpeed(value);
      break;
    case 'A':
      if (findHome()) {
        TiltSerial.println(F("Homing complete"));
        queryTiltAngle();
      } else {
        stepper.setCurrentPosition(0);
        stepper.moveTo(0);
        TiltSerial.println(F("Error homing"));
      }
      break;
    case 'e':
      // Toggle stepper enable state
      enableSteppers();
      break;
    case 'm':
      setStepMode(integerValue);
      break;
    case 'R':
      printStatus();
      break;
    case 'I':
      invertTiltDirection(integerValue != 0);
      break;
    case 'O':
      hallOffsetDeg = value;
      TiltSerial.print(F("Tilt offset: "));
      TiltSerial.print(hallOffsetDeg, 3);
      TiltSerial.println(F(" deg"));
      break;
    case 'H':
      TiltSerial.print(F("Homing mode: "));
      TiltSerial.println(integerValue);
      break;
    case 'U':
      TiltSerial.println(F("Settings saved"));
      break;
    default:
      TiltSerial.print(F("ERR UNKNOWN COMMAND: "));
      TiltSerial.println(instruction);
      break;
  }
}

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(LIDAR_DTR_PIN, OUTPUT);
  digitalWrite(LIDAR_DTR_PIN, LIDAR_DTR_ACTIVE_LEVEL);

  digitalWrite(ENABLE_PIN, LOW);
  enabled = true;

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  stepper.setAcceleration(5000.0);
  stepMode = SIXTEENTH_STEP;
  setStepMode(stepMode);
  stepper.setCurrentPosition(0);
  applyMotorSpeed();

  invertTiltDirection(false);
  TiltSerial.println(F("READY"));
}

void loop() {
  if (tiltRxOverflow) {
    tiltRxOverflow = false;
    TiltSerial.println(F("ERR TILT RX OVERFLOW"));
  }

  processUsbMux();
  serialParse();
  stepper.run();
  updateTiltMotion();
  serviceLidarBridge();
  TiltSerial.flush();
}