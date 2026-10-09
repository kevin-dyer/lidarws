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

const int HALF_STEP = 2;
const int QUARTER_STEP = 4;
const int EIGHTH_STEP = 8;
const int SIXTEENTH_STEP = 16;

const float MOTOR_STEPS_PER_REV = 200.0;
const float GEAR_RATIO = 7.6875;  // 123 / 16
const float MAX_TILT_SPEED_DEG_S = 10.0;
const float HOMING_SPEED_DEG_S = 40.0;
const float MAX_ANGLE_DEG = 45.0;
const float DEFAULT_SWEEP_START_DEG = -15.0;
const float DEFAULT_SWEEP_STOP_DEG = 15.0;
const unsigned long SWEEP_SAMPLE_INTERVAL_US = 10000UL;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

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
    Serial.println(F("Invalid mode. Enter 2, 4, 8 or 16"));
    return;
  }

  const float previousStepsPerDegree = stepsPerDegree;
  if (previousStepsPerDegree > 0.0) {
    stepper.setCurrentPosition((long)lround(stepper.currentPosition() * ((float)newMode / (float)stepMode)));
  }

  stepMode = newMode;
  stepsPerDegree = calcStepsPerDegree();
  applyMotorSpeed();

  Serial.print(F("set to "));
  Serial.print(stepMode);
  Serial.println(F(" step mode."));
}

void setSpeed(float speedDegPerSec) {
  maxTiltSpeedDegS = speedDegPerSec;
  applyMotorSpeed();
  Serial.print(F("Max tilt speed: "));
  Serial.print(maxTiltSpeedDegS, 1);
  Serial.println(F(" deg/s"));
}

void enableSteppers() {
  if (enabled) {
    digitalWrite(ENABLE_PIN, HIGH);
    enabled = false;
    Serial.println(F("Disabled"));
  } else {
    digitalWrite(ENABLE_PIN, LOW);
    enabled = true;
    Serial.println(F("Enabled"));
  }
}

void queryTiltAngle() {
  Serial.write('T');
  Serial.print(stepsToAngle(stepper.currentPosition()), 4);
  Serial.write('\n');
}

void queryTiltSteps() {
  Serial.write('Z');
  Serial.print(stepper.currentPosition());
  Serial.write('\n');
}

void testHallSensor() {
  Serial.write('L');
  Serial.print(hallTriggered() ? 0 : 1);
  Serial.write('\n');
}

void moveToAbsoluteAngle(float angle) {
  if (fabs(angle) > MAX_ANGLE_DEG) {
    Serial.println(F("LIMIT"));
    return;
  }
  const long target = lround(angleToSteps(angle));
  stepper.moveTo(target);
  stepper.runToNewPosition(target);
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
    Serial.println(F("ERR BUSY"));
    return;
  }
  if (!isfinite(startAngle) || !isfinite(stopAngle) ||
      fabs(startAngle) > MAX_ANGLE_DEG || fabs(stopAngle) > MAX_ANGLE_DEG ||
      startAngle == stopAngle) {
    Serial.println(F("ERR SWEEP LIMIT"));
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
  Serial.print(micros());
  Serial.write(',');
  Serial.print(stepper.currentPosition());
  Serial.write('\n');
}

void updateTiltMotion() {
  if (tiltMotionState == TILT_MOVING_TO_SWEEP_START &&
      stepper.distanceToGo() == 0) {
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

  if (tiltMotionState == TILT_RETURNING_HOME &&
      stepper.distanceToGo() == 0) {
    tiltMotionState = TILT_IDLE;
    applyMotorSpeed();
    Serial.println(F("X"));
  }
}

void printStatus() {
  Serial.println(F("\n=== TILT MOUNT STATUS ==="));
  Serial.print(F("Enable state: ")); Serial.println(enabled ? F("Enabled") : F("Disabled"));
  Serial.print(F("Step mode: ")); Serial.print(stepMode); Serial.println(F("x"));
  Serial.print(F("Current tilt angle: ")); Serial.print(stepsToAngle(stepper.currentPosition()), 3); Serial.println(F(" deg"));
  Serial.print(F("Current tilt steps: ")); Serial.println(stepper.currentPosition());
  Serial.print(F("Max tilt speed: ")); Serial.print(maxTiltSpeedDegS, 2); Serial.println(F(" deg/s"));
  Serial.print(F("Hall sensor state: ")); Serial.println(hallTriggered() ? 0 : 1);
  Serial.print(F("Hall offset: ")); Serial.print(hallOffsetDeg, 3); Serial.println(F(" deg"));
}

void invertTiltDirection(bool invert) {
  invertTilt = invert;
  stepper.setPinsInverted(invert, false, false);
  Serial.print(F("tilt inversion: "));
  Serial.println(invert ? 1 : 0);
}

bool findHome() {
  bool startedOnSensor = hallTriggered();
  float originalMaxSpeed = maxTiltSpeedDegS;
  maxTiltSpeedDegS = HOMING_SPEED_DEG_S;
  applyMotorSpeed();

  float firstTransition = 0.0;
  float secondTransition = 0.0;

  if (startedOnSensor) {
    Serial.println(F("Phase 1: Searching for Low->High transition..."));
    stepper.moveTo(angleToSteps(-70.0));
    int prevHall = hallStateCode();
    bool found = false;
    while (stepper.run()) {
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
      Serial.println(F("Error: Could not find Low->High transition"));
      return false;
    }

    Serial.println(F("Phase 2: Reversing for second transition..."));
    const float phase2Target = firstTransition + 70.0;
    stepper.moveTo(angleToSteps(phase2Target));
    prevHall = hallStateCode();
    const float phase2Start = firstTransition;
    bool secondFound = false;
    while (stepper.run()) {
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
      Serial.println(F("Error: Could not find second Low->High transition"));
      return false;
    }
  } else {
    Serial.println(F("Phase 1: Searching for High->Low transition..."));
    stepper.moveTo(angleToSteps(-70.0));
    int prevHall = hallStateCode();
    bool found = false;
    while (stepper.run()) {
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
      Serial.println(F("Error: Could not find High->Low transition"));
      return false;
    }

    Serial.println(F("Phase 2: Continuing for second transition..."));
    const float phase2Target = firstTransition - 70.0;
    stepper.moveTo(angleToSteps(phase2Target));
    prevHall = hallStateCode();
    const float phase2Start = firstTransition;
    bool secondFound = false;
    while (stepper.run()) {
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
      Serial.println(F("Error: Could not find second High->Low transition"));
      return false;
    }
  }

  const float homeAngle = (firstTransition + secondTransition) / 2.0;
  Serial.print(F("Center position: ")); Serial.print(homeAngle, 2); Serial.println(F(" deg"));

  stepper.moveTo(angleToSteps(homeAngle));
  stepper.runToNewPosition(angleToSteps(homeAngle));

  if (hallOffsetDeg != 0.0) {
    const float target = homeAngle + hallOffsetDeg;
    stepper.moveTo(angleToSteps(target));
    stepper.runToNewPosition(angleToSteps(target));
  }

  stepper.setCurrentPosition(0);
  stepper.moveTo(0);

  maxTiltSpeedDegS = originalMaxSpeed;
  applyMotorSpeed();
  Serial.println(F("Homing complete"));
  return true;
}

void serialParse() {
  if (Serial.available() == 0) {
    return;
  }

  char instruction = Serial.read();
  if (instruction == '\n' || instruction == '\r') {
    return;
  }

  if (instruction == '+') {
    while (Serial.available()) {
      Serial.read();
    }
    return;
  }

  char payload[32];
  memset(payload, 0, sizeof(payload));
  size_t index = 0;

  delay(2);
  while (Serial.available() > 0 && index < sizeof(payload) - 1) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      break;
    }
    payload[index++] = c;
  }

  const float value = atof(payload);
  const long integerValue = atoi(payload);

  if (tiltMotionState != TILT_IDLE && instruction != 'X') {
    Serial.println(F("ERR BUSY"));
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
    case 'C': {
      float startAngle = 0.0;
      float stopAngle = 0.0;
      if (!parseSweepAngles(payload, startAngle, stopAngle)) {
        Serial.println(F("ERR C expected startAngle,stopAngle"));
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
        Serial.println(F("Homing complete"));
        queryTiltAngle();
      } else {
        stepper.setCurrentPosition(0);
        stepper.moveTo(0);
        Serial.println(F("Error homing"));
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
      Serial.print(F("Tilt offset: ")); Serial.print(hallOffsetDeg, 3); Serial.println(F(" deg"));
      break;
    case 'H':
      Serial.print(F("Homing mode: ")); Serial.println(integerValue);
      break;
    case 'U':
      Serial.println(F("Settings saved"));
      break;
    default:
      Serial.print(F("ERR UNKNOWN COMMAND: "));
      Serial.println(instruction);
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

  digitalWrite(ENABLE_PIN, LOW);
  enabled = true;

  Serial.begin(115200);

  stepper.setAcceleration(5000.0);
  stepMode = SIXTEENTH_STEP;
  setStepMode(stepMode);
  stepper.setCurrentPosition(0);
  applyMotorSpeed();

  invertTiltDirection(false);
  Serial.println(F("READY"));
}

void loop() {
  serialParse();
  stepper.run();
  updateTiltMotion();
}