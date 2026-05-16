#include <Wire.h>
#include <QMC5883LCompass.h>

/*
  RoboScan Serial Monitor Controller

  Target board: Arduino Mega
  Serial Monitor: 115200 baud, Newline recommended

  Quick commands:
    W              forward
    X              backward
    A              turn left 90 degrees
    D              turn right 90 degrees
    S              stop
*/

QMC5883LCompass compass;

// Left motor driver
const int M1_RPWM = 5;
const int M1_LPWM = 6;
const int M1_R_EN = 7;
const int M1_L_EN = 8;

// Right motor driver
const int M2_RPWM = 44;
const int M2_LPWM = 45;
const int M2_R_EN = 46;
const int M2_L_EN = 47;

// Encoder pins
const int ENC_LEFT_A = 2;
const int ENC_LEFT_B = 18;
const int ENC_RIGHT_A = 3;
const int ENC_RIGHT_B = 19;

const float WHEEL_DIAMETER_M = 0.32;
const float WHEEL_CIRC_M = PI * WHEEL_DIAMETER_M;
const float TICKS_PER_REV = 2400.0;

// Measure center-to-center distance between the left and right wheels and tune this.
const float ROBOT_TRACK_WIDTH_M = 0.50;

const int DRIVE_SPEED = 160;
const int TURN_SPEED = 120;
const int TURN_SLOW_SPEED = 80;
const float TURN_ANGLE_DEG = 90.0;
const float HEADING_TOLERANCE_DEG = 3.0;
const unsigned long TURN_TIMEOUT_MS = 8000;
const unsigned long TELEMETRY_INTERVAL_MS = 500;

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;
unsigned long lastTelemetryMs = 0;

void onLeftA() {
  if (digitalRead(ENC_LEFT_A) != digitalRead(ENC_LEFT_B)) leftEncoderTicks++;
  else leftEncoderTicks--;
}

void onLeftB() {
  if (digitalRead(ENC_LEFT_A) == digitalRead(ENC_LEFT_B)) leftEncoderTicks++;
  else leftEncoderTicks--;
}

void onRightA() {
  if (digitalRead(ENC_RIGHT_A) != digitalRead(ENC_RIGHT_B)) rightEncoderTicks++;
  else rightEncoderTicks--;
}

void onRightB() {
  if (digitalRead(ENC_RIGHT_A) == digitalRead(ENC_RIGHT_B)) rightEncoderTicks++;
  else rightEncoderTicks--;
}

void readEncoderTicks(long &leftTicks, long &rightTicks) {
  noInterrupts();
  leftTicks = leftEncoderTicks;
  rightTicks = rightEncoderTicks;
  interrupts();
}

float normalizeHeading(float heading) {
  while (heading < 0.0) heading += 360.0;
  while (heading >= 360.0) heading -= 360.0;
  return heading;
}

float readHeading() {
  compass.read();
  return normalizeHeading(compass.getAzimuth());
}

float headingError(float target, float current) {
  float error = normalizeHeading(target) - normalizeHeading(current);
  if (error > 180.0) error -= 360.0;
  if (error < -180.0) error += 360.0;
  return error;
}

float ticksToMeters(long ticks) {
  return (ticks / TICKS_PER_REV) * WHEEL_CIRC_M;
}

void writeDrivePwm(int leftForward, int leftBackward, int rightForward, int rightBackward) {
  analogWrite(M1_RPWM, constrain(leftForward, 0, 255));
  analogWrite(M1_LPWM, constrain(leftBackward, 0, 255));
  analogWrite(M2_RPWM, constrain(rightForward, 0, 255));
  analogWrite(M2_LPWM, constrain(rightBackward, 0, 255));
}

void stopRobot() {
  writeDrivePwm(0, 0, 0, 0);
  Serial.println("ACK:STOP");
}

void driveForward(int speed) {
  speed = constrain(speed, 0, 255);
  writeDrivePwm(speed, 0, speed, 0);
  Serial.print("ACK:FORWARD|speed=");
  Serial.println(speed);
}

void driveBackward(int speed) {
  speed = constrain(speed, 0, 255);
  writeDrivePwm(0, speed, 0, speed);
  Serial.print("ACK:BACKWARD|speed=");
  Serial.println(speed);
}

void startSpinLeft(int speed) {
  speed = constrain(speed, 0, 255);
  writeDrivePwm(0, speed, speed, 0);
}

void startSpinRight(int speed) {
  speed = constrain(speed, 0, 255);
  writeDrivePwm(speed, 0, 0, speed);
}

void printTelemetry() {
  long leftTicks;
  long rightTicks;
  readEncoderTicks(leftTicks, rightTicks);

  Serial.print("DATA");
  Serial.print("|heading=");
  Serial.print(readHeading(), 2);
  Serial.print("|left_ticks=");
  Serial.print(leftTicks);
  Serial.print("|right_ticks=");
  Serial.print(rightTicks);
  Serial.print("|left_m=");
  Serial.print(ticksToMeters(leftTicks), 3);
  Serial.print("|right_m=");
  Serial.println(ticksToMeters(rightTicks), 3);
}

void printTelemetryIfDue() {
  if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = millis();
    printTelemetry();
  }
}

bool stopCommandReceived() {
  if (Serial.available() == 0) return false;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();
  return cmd == "S";
}

void turnNinety(int direction) {
  long startLeftTicks;
  long startRightTicks;
  readEncoderTicks(startLeftTicks, startRightTicks);

  float startHeading = readHeading();
  float targetHeading = normalizeHeading(startHeading + (direction * TURN_ANGLE_DEG));
  float expectedWheelTravelM = (PI * ROBOT_TRACK_WIDTH_M) * (TURN_ANGLE_DEG / 360.0);
  long expectedTicks = expectedWheelTravelM / WHEEL_CIRC_M * TICKS_PER_REV;
  unsigned long startMs = millis();

  Serial.print(direction < 0 ? "ACK:TURN_LEFT_90" : "ACK:TURN_RIGHT_90");
  Serial.print("|start=");
  Serial.print(startHeading, 2);
  Serial.print("|target=");
  Serial.print(targetHeading, 2);
  Serial.print("|expected_ticks=");
  Serial.println(expectedTicks);

  while (millis() - startMs < TURN_TIMEOUT_MS) {
    float currentHeading = readHeading();
    float error = headingError(targetHeading, currentHeading);
    if (abs(error) <= HEADING_TOLERANCE_DEG) {
      stopRobot();
      Serial.print("ACK:TURN_DONE|heading=");
      Serial.println(currentHeading, 2);
      printTelemetry();
      return;
    }

    if (stopCommandReceived()) {
      stopRobot();
      Serial.println("ACK:TURN_ABORTED");
      return;
    }

    long leftTicks;
    long rightTicks;
    readEncoderTicks(leftTicks, rightTicks);
    long leftDelta = abs(leftTicks - startLeftTicks);
    long rightDelta = abs(rightTicks - startRightTicks);
    long averageDelta = (leftDelta + rightDelta) / 2;
    if (averageDelta > expectedTicks * 1.4) {
      stopRobot();
      Serial.println("WARN:TURN_STOPPED_BY_ENCODER_LIMIT");
      printTelemetry();
      return;
    }

    int turnSpeed = TURN_SPEED;
    if (averageDelta > expectedTicks * 0.7 || abs(error) < 15.0) {
      turnSpeed = TURN_SLOW_SPEED;
    }

    if (direction < 0) startSpinLeft(turnSpeed);
    else startSpinRight(turnSpeed);

    printTelemetryIfDue();
  }

  stopRobot();
  Serial.println("WARN:TURN_TIMEOUT");
  printTelemetry();
}

void printHelp() {
  Serial.println();
  Serial.println("RoboScan Serial Monitor Commands");
  Serial.println("  W / X / A / D / S       forward / backward / left 90 / right 90 / stop");
  Serial.println();
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  cmd.toUpperCase();

  if (cmd == "W") {
    driveForward(DRIVE_SPEED);
  } else if (cmd == "X") {
    driveBackward(DRIVE_SPEED);
  } else if (cmd == "A") {
    turnNinety(-1);
  } else if (cmd == "D") {
    turnNinety(1);
  } else if (cmd == "S") {
    stopRobot();
  } else {
    Serial.print("ERROR:Unknown_command|cmd=");
    Serial.println(cmd);
    printHelp();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  pinMode(M1_RPWM, OUTPUT);
  pinMode(M1_LPWM, OUTPUT);
  pinMode(M1_R_EN, OUTPUT);
  pinMode(M1_L_EN, OUTPUT);
  pinMode(M2_RPWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);
  pinMode(M2_R_EN, OUTPUT);
  pinMode(M2_L_EN, OUTPUT);
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  digitalWrite(M1_R_EN, HIGH);
  digitalWrite(M1_L_EN, HIGH);
  digitalWrite(M2_R_EN, HIGH);
  digitalWrite(M2_L_EN, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), onLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B), onLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), onRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), onRightB, CHANGE);

  Wire.begin();
  compass.init();

  stopRobot();

  Serial.println("READY:RoboScan_serial_monitor_controller");
  printHelp();
}

void loop() {
  printTelemetryIfDue();

  if (Serial.available() > 0) {
    handleCommand(Serial.readStringUntil('\n'));
  }
}
