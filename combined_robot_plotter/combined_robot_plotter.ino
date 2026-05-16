#include <Wire.h>
#include <QMC5883LCompass.h>

/*
  Combined RoboScan Controller

  Target board: Arduino Mega
  Serial Monitor: 115200 baud, Newline recommended

  Drive commands:
    W                  drive forward
    X                  drive backward
    A                  turn left 90 degrees from current heading
    D                  turn right 90 degrees from current heading
    S                  stop drive and plotter
    SPEED <0-255>      set drive speed
    TURN SPEED <0-255> set 90-degree turn speed

  Plotter commands:
    PLOT CONT          arm continuous plotting while moving
    PLOT DASH          arm dashed plotting while moving
    PLOT DASH DIST <dash_m> <gap_m>
                       dashed plotting by measured travel distance
    PLOT OFF           plotter off
    PLOT SPEED <0-255> set plotter motor speed
    PLOT DIST <meters> plot for this travel distance, 0 = unlimited
    PLOT TICKS <ticks> plot for this encoder tick distance, 0 = unlimited
    WHEEL RADIUS <m>   set wheel radius used for distance math

  Info:
    STATUS             print current heading, encoders, and plotter mode
    HELP               show commands
*/

QMC5883LCompass compass;

// Left drive motor driver
const int M1_RPWM = 5;
const int M1_LPWM = 6;
const int M1_R_EN = 7;
const int M1_L_EN = 8;

// Right drive motor driver
const int M2_RPWM = 44;
const int M2_LPWM = 45;
const int M2_R_EN = 46;
const int M2_L_EN = 47;

// Plotter motor driver
const int PLOTTER_RPWM = 38;
const int PLOTTER_LPWM = 39;
const int PLOTTER_R_EN = 40;
const int PLOTTER_L_EN = 41;

// Encoder pins
const int ENC_LEFT_A = 2;
const int ENC_LEFT_B = 18;
const int ENC_RIGHT_A = 3;
const int ENC_RIGHT_B = 19;

const float TICKS_PER_REV = 2400.0;
const float ENCODER_PPR = 600.0;
const float QUADRATURE_EDGES_PER_PULSE = 4.0;

// Measure center-to-center distance between left and right wheels, then tune this.
const float ROBOT_TRACK_WIDTH_M = 0.50;

const float TURN_ANGLE_DEG = 90.0;
const float HEADING_TOLERANCE_DEG = 2.5;
const unsigned long TURN_TIMEOUT_MS = 10000;
const unsigned long TELEMETRY_INTERVAL_MS = 100;

const int MIN_TURN_PWM = 45;
const int DEFAULT_DRIVE_SPEED = 160;
const int DEFAULT_TURN_SPEED = 90;
const int DEFAULT_PLOTTER_SPEED = 180;
const long SERIAL_BAUD_RATE = 115200;

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

int driveSpeed = DEFAULT_DRIVE_SPEED;
int turnSpeed = DEFAULT_TURN_SPEED;
int plotterSpeed = DEFAULT_PLOTTER_SPEED;
bool driveIsMoving = false;
float wheelRadiusM = 0.16;
float plotDistanceTargetM = 0.0;
bool plotDistanceStartSet = false;
bool plotDistanceReached = false;
long plotStartLeftTicks = 0;
long plotStartRightTicks = 0;
float dashPaintDistanceM = 0.50;
float dashGapDistanceM = 0.30;
long dashSegmentStartLeftTicks = 0;
long dashSegmentStartRightTicks = 0;

enum PlotterMode {
  PLOTTER_OFF,
  PLOTTER_CONT,
  PLOTTER_DASH
};

PlotterMode plotterMode = PLOTTER_OFF;
bool plotterDashMotorOn = false;
unsigned long lastTelemetryMs = 0;
unsigned long lastSpeedSampleMs = 0;
long lastSpeedLeftTicks = 0;
long lastSpeedRightTicks = 0;
long lastSpeedLeftDelta = 0;
long lastSpeedRightDelta = 0;
float measuredSpeedMps = 0.0;

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

float wheelCircumferenceM() {
  return 2.0 * PI * wheelRadiusM;
}

float ticksToMeters(long ticks) {
  return (ticks / TICKS_PER_REV) * wheelCircumferenceM();
}

float ticksToDistanceM(long ticks) {
  long absoluteTicks = ticks < 0 ? -ticks : ticks;
  return (absoluteTicks / TICKS_PER_REV) * wheelCircumferenceM();
}

void resetPlotDistanceStart() {
  readEncoderTicks(plotStartLeftTicks, plotStartRightTicks);
  plotDistanceStartSet = true;
  plotDistanceReached = false;
}

void resetDashSegmentStart() {
  readEncoderTicks(dashSegmentStartLeftTicks, dashSegmentStartRightTicks);
}

float plotTravelDistanceM() {
  long leftTicks;
  long rightTicks;
  readEncoderTicks(leftTicks, rightTicks);

  float leftDistance = ticksToDistanceM(leftTicks - plotStartLeftTicks);
  float rightDistance = ticksToDistanceM(rightTicks - plotStartRightTicks);
  return (leftDistance + rightDistance) / 2.0;
}

float dashSegmentTravelDistanceM() {
  long leftTicks;
  long rightTicks;
  readEncoderTicks(leftTicks, rightTicks);

  float leftDistance = ticksToDistanceM(leftTicks - dashSegmentStartLeftTicks);
  float rightDistance = ticksToDistanceM(rightTicks - dashSegmentStartRightTicks);
  return (leftDistance + rightDistance) / 2.0;
}

void updateMeasuredSpeed() {
  unsigned long now = millis();
  if (now - lastSpeedSampleMs < TELEMETRY_INTERVAL_MS) return;

  long leftTicks;
  long rightTicks;
  readEncoderTicks(leftTicks, rightTicks);

  unsigned long dtMs = now - lastSpeedSampleMs;
  if (dtMs > 0) {
    lastSpeedLeftDelta = leftTicks - lastSpeedLeftTicks;
    lastSpeedRightDelta = rightTicks - lastSpeedRightTicks;
    float leftDistance = ticksToDistanceM(lastSpeedLeftDelta);
    float rightDistance = ticksToDistanceM(lastSpeedRightDelta);
    measuredSpeedMps = ((leftDistance + rightDistance) / 2.0) / (dtMs / 1000.0);
  }

  lastSpeedLeftTicks = leftTicks;
  lastSpeedRightTicks = rightTicks;
  lastSpeedSampleMs = now;
}

const char *plotterModeName() {
  if (plotterMode == PLOTTER_CONT) return "CONT";
  if (plotterMode == PLOTTER_DASH) return "DASH";
  return "OFF";
}

void writeDrivePwm(int leftForward, int leftBackward, int rightForward, int rightBackward) {
  analogWrite(M1_RPWM, constrain(leftForward, 0, 255));
  analogWrite(M1_LPWM, constrain(leftBackward, 0, 255));
  analogWrite(M2_RPWM, constrain(rightForward, 0, 255));
  analogWrite(M2_LPWM, constrain(rightBackward, 0, 255));
}

void stopDrive() {
  driveIsMoving = false;
  writeDrivePwm(0, 0, 0, 0);
  stopPlotterMotor();
  plotDistanceStartSet = false;
  Serial.println("ACK:DRIVE_STOP");
}

void driveForward(int speed) {
  speed = constrain(speed, 0, 255);
  driveIsMoving = speed > 0;
  writeDrivePwm(speed, 0, speed, 0);
  Serial.print("ACK:FORWARD|speed=");
  Serial.println(speed);
}

void driveBackward(int speed) {
  speed = constrain(speed, 0, 255);
  driveIsMoving = speed > 0;
  writeDrivePwm(0, speed, 0, speed);
  Serial.print("ACK:BACKWARD|speed=");
  Serial.println(speed);
}

void startSpinLeft(int speed) {
  speed = constrain(speed, 0, 255);
  driveIsMoving = speed > 0;
  writeDrivePwm(0, speed, speed, 0);
}

void startSpinRight(int speed) {
  speed = constrain(speed, 0, 255);
  driveIsMoving = speed > 0;
  writeDrivePwm(speed, 0, 0, speed);
}

void runPlotterForward(int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(PLOTTER_RPWM, speed);
  analogWrite(PLOTTER_LPWM, 0);
}

void stopPlotterMotor() {
  analogWrite(PLOTTER_RPWM, 0);
  analogWrite(PLOTTER_LPWM, 0);
}

void setPlotterMode(PlotterMode mode) {
  plotterMode = mode;
  plotterDashMotorOn = false;

  if (plotterMode == PLOTTER_CONT) {
    if (driveIsMoving) runPlotterForward(plotterSpeed);
    else stopPlotterMotor();
  } else if (plotterMode == PLOTTER_DASH) {
    plotterDashMotorOn = true;
    resetDashSegmentStart();
    if (driveIsMoving) runPlotterForward(plotterSpeed);
    else stopPlotterMotor();
  } else {
    stopPlotterMotor();
  }

  Serial.print("ACK:PLOT_MODE|mode=");
  Serial.println(plotterModeName());
}

void updatePlotter() {
  if (!driveIsMoving) {
    stopPlotterMotor();
    plotterDashMotorOn = true;
    plotDistanceStartSet = false;
    resetDashSegmentStart();
    return;
  }

  if (plotterMode == PLOTTER_OFF) {
    stopPlotterMotor();
    return;
  }

  if (plotDistanceTargetM > 0.0) {
    if (!plotDistanceStartSet) resetPlotDistanceStart();

    if (plotTravelDistanceM() >= plotDistanceTargetM) {
      stopPlotterMotor();
      plotDistanceReached = true;
      return;
    }
  } else {
    plotDistanceReached = false;
  }

  if (plotDistanceReached) {
    stopPlotterMotor();
    return;
  }

  if (plotterMode == PLOTTER_CONT) {
    runPlotterForward(plotterSpeed);
    return;
  }

  if (plotterMode != PLOTTER_DASH) return;

  float segmentTargetM = plotterDashMotorOn ? dashPaintDistanceM : dashGapDistanceM;
  if (segmentTargetM <= 0.0) segmentTargetM = 0.01;
  if (dashSegmentTravelDistanceM() < segmentTargetM) return;

  plotterDashMotorOn = !plotterDashMotorOn;
  resetDashSegmentStart();

  if (plotterDashMotorOn) runPlotterForward(plotterSpeed);
  else stopPlotterMotor();
}

void printStatus() {
  long leftTicks;
  long rightTicks;
  readEncoderTicks(leftTicks, rightTicks);
  float heading = readHeading();

  Serial.print("STATUS");
  Serial.print("|heading=");
  Serial.print(heading, 2);
  Serial.print("|yaw=");
  Serial.print(heading, 2);
  Serial.print("|e1=");
  Serial.print(leftTicks);
  Serial.print("|e2=");
  Serial.print(rightTicks);
  Serial.print("|de1=");
  Serial.print(lastSpeedLeftDelta);
  Serial.print("|de2=");
  Serial.print(lastSpeedRightDelta);
  Serial.print("|left_m=");
  Serial.print(ticksToMeters(leftTicks), 3);
  Serial.print("|right_m=");
  Serial.print(ticksToMeters(rightTicks), 3);
  Serial.print("|speed=");
  Serial.print(measuredSpeedMps, 3);
  Serial.print("|lspeed=");
  Serial.print(driveIsMoving ? driveSpeed : 0);
  Serial.print("|rspeed=");
  Serial.print(driveIsMoving ? driveSpeed : 0);
  Serial.print("|battery=0");
  Serial.print("|drive_speed=");
  Serial.print(driveSpeed);
  Serial.print("|turn_speed=");
  Serial.print(turnSpeed);
  Serial.print("|drive_moving=");
  Serial.print(driveIsMoving ? 1 : 0);
  Serial.print("|wheel_radius_m=");
  Serial.print(wheelRadiusM, 3);
  Serial.print("|plot_mode=");
  Serial.print(plotterModeName());
  Serial.print("|plot_speed=");
  Serial.print(plotterSpeed);
  Serial.print("|spraying=");
  Serial.print((driveIsMoving && plotterMode != PLOTTER_OFF && !plotDistanceReached) ? 1 : 0);
  Serial.print("|dash_m=");
  Serial.print(dashPaintDistanceM, 3);
  Serial.print("|gap_m=");
  Serial.print(dashGapDistanceM, 3);
  Serial.print("|plot_target_m=");
  Serial.print(plotDistanceTargetM, 3);
  Serial.print("|plot_done=");
  Serial.println(plotDistanceReached ? 1 : 0);
}

void printStatusIfDue() {
  if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = millis();
    printStatus();
  }
}

bool abortTurnCommandReceived() {
  if (Serial.available() == 0) return false;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "S" || cmd == "STOP") {
    stopDrive();
    setPlotterMode(PLOTTER_OFF);
    return true;
  }

  return false;
}

int turnPwmForError(float errorDeg, long encoderDelta, long expectedTicks) {
  int maxTurnPwm = constrain(turnSpeed, MIN_TURN_PWM, 255);
  float absError = abs(errorDeg);

  int headingPwm = map(constrain((int)absError, 0, 90), 0, 90, MIN_TURN_PWM, maxTurnPwm);
  int encoderPwm = maxTurnPwm;

  if (expectedTicks > 0) {
    float progress = constrain((float)encoderDelta / expectedTicks, 0.0, 1.0);
    encoderPwm = map((int)(progress * 100.0), 0, 100, maxTurnPwm, MIN_TURN_PWM);
  }

  return constrain(min(headingPwm, encoderPwm), MIN_TURN_PWM, maxTurnPwm);
}

void turnNinety(int direction) {
  long startLeftTicks;
  long startRightTicks;
  readEncoderTicks(startLeftTicks, startRightTicks);

  float startHeading = readHeading();
  float targetHeading = normalizeHeading(startHeading + (direction * TURN_ANGLE_DEG));
  float expectedWheelTravelM = (PI * ROBOT_TRACK_WIDTH_M) * (TURN_ANGLE_DEG / 360.0);
  long expectedTicks = expectedWheelTravelM / wheelCircumferenceM() * TICKS_PER_REV;
  unsigned long startMs = millis();

  Serial.print(direction < 0 ? "ACK:TURN_LEFT_90" : "ACK:TURN_RIGHT_90");
  Serial.print("|start=");
  Serial.print(startHeading, 2);
  Serial.print("|target=");
  Serial.print(targetHeading, 2);
  Serial.print("|turn_speed=");
  Serial.print(turnSpeed);
  Serial.print("|expected_ticks=");
  Serial.println(expectedTicks);

  while (millis() - startMs < TURN_TIMEOUT_MS) {
    updatePlotter();
    printStatusIfDue();

    float currentHeading = readHeading();
    float error = headingError(targetHeading, currentHeading);
    if (abs(error) <= HEADING_TOLERANCE_DEG) {
      stopDrive();
      Serial.print("ACK:TURN_DONE|heading=");
      Serial.println(currentHeading, 2);
      printStatus();
      return;
    }

    if (abortTurnCommandReceived()) {
      Serial.println("ACK:TURN_ABORTED");
      return;
    }

    long leftTicks;
    long rightTicks;
    readEncoderTicks(leftTicks, rightTicks);
    long leftDelta = abs(leftTicks - startLeftTicks);
    long rightDelta = abs(rightTicks - startRightTicks);
    long averageDelta = (leftDelta + rightDelta) / 2;

    if (averageDelta > expectedTicks * 1.35) {
      stopDrive();
      Serial.println("WARN:TURN_STOPPED_BY_ENCODER_LIMIT");
      printStatus();
      return;
    }

    int turnPwm = turnPwmForError(error, averageDelta, expectedTicks);
    if (direction < 0) startSpinLeft(turnPwm);
    else startSpinRight(turnPwm);
  }

  stopDrive();
  Serial.println("WARN:TURN_TIMEOUT");
  printStatus();
}

void stopAll() {
  stopDrive();
  setPlotterMode(PLOTTER_OFF);
}

void printHelp() {
  Serial.println();
  Serial.println("Combined RoboScan Commands");
  Serial.println("  W / X / A / D / S       forward / backward / left 90 / right 90 / stop all");
  Serial.println("  SPEED <0-255>           set drive speed");
  Serial.println("  TURN SPEED <0-255>      set slower/faster turn speed");
  Serial.println("  PLOT CONT               plotter continuous while moving");
  Serial.println("  PLOT DASH               plotter dashed while moving");
  Serial.println("  PLOT DASH DIST <d> <g>  dashed plot/gap distances in meters");
  Serial.println("  PLOT OFF                plotter off");
  Serial.println("  PLOT SPEED <0-255>      set plotter speed");
  Serial.println("  PLOT DIST <meters>      plot for distance, 0 = unlimited");
  Serial.println("  PLOT TICKS <ticks>      plot for encoder ticks, 0 = unlimited");
  Serial.println("  WHEEL RADIUS <meters>   set wheel radius for distance math");
  Serial.println("  STATUS                  print sensor and mode data");
  Serial.println("  HELP                    show this menu");
  Serial.println();
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  cmd.toUpperCase();

  if (cmd == "W") {
    driveForward(driveSpeed);
  } else if (cmd == "X") {
    driveBackward(driveSpeed);
  } else if (cmd == "A") {
    turnNinety(-1);
  } else if (cmd == "D") {
    turnNinety(1);
  } else if (cmd == "S" || cmd == "STOP") {
    stopAll();
  } else if (cmd.startsWith("SPEED ")) {
    driveSpeed = constrain(cmd.substring(6).toInt(), 0, 255);
    Serial.print("ACK:SPEED|drive_speed=");
    Serial.println(driveSpeed);
  } else if (cmd.startsWith("TURN SPEED ")) {
    turnSpeed = constrain(cmd.substring(11).toInt(), 0, 255);
    Serial.print("ACK:TURN_SPEED|speed=");
    Serial.println(turnSpeed);
  } else if (cmd == "PLOT CONT" || cmd == "CONT") {
    setPlotterMode(PLOTTER_CONT);
  } else if (cmd == "PLOT DASH" || cmd == "DASH") {
    setPlotterMode(PLOTTER_DASH);
  } else if (cmd.startsWith("PLOT DASH DIST ")) {
    int valuesStart = 15;
    int separator = cmd.indexOf(' ', valuesStart);
    if (separator < 0) {
      Serial.println("ERROR:Use_PLOT_DASH_DIST_dash_m_gap_m");
      return;
    }
    float nextDashM = cmd.substring(valuesStart, separator).toFloat();
    float nextGapM = cmd.substring(separator + 1).toFloat();
    if (nextDashM <= 0.0 || nextGapM <= 0.0) {
      Serial.println("ERROR:Dash_and_gap_must_be_positive");
      return;
    }
    dashPaintDistanceM = nextDashM;
    dashGapDistanceM = nextGapM;
    resetDashSegmentStart();
    Serial.print("ACK:PLOT_DASH_DIST|dash_m=");
    Serial.print(dashPaintDistanceM, 3);
    Serial.print("|gap_m=");
    Serial.println(dashGapDistanceM, 3);
  } else if (cmd == "PLOT OFF") {
    setPlotterMode(PLOTTER_OFF);
  } else if (cmd.startsWith("PLOT SPEED ")) {
    plotterSpeed = constrain(cmd.substring(11).toInt(), 0, 255);
    Serial.print("ACK:PLOT_SPEED|speed=");
    Serial.println(plotterSpeed);
  } else if (cmd.startsWith("PLOT DIST ")) {
    plotDistanceTargetM = cmd.substring(10).toFloat();
    if (plotDistanceTargetM < 0.0) plotDistanceTargetM = 0.0;
    plotDistanceStartSet = false;
    plotDistanceReached = false;
    Serial.print("ACK:PLOT_DIST|meters=");
    Serial.println(plotDistanceTargetM, 3);
  } else if (cmd.startsWith("PLOT TICKS ")) {
    long targetTicks = cmd.substring(11).toInt();
    if (targetTicks < 0) targetTicks = 0;
    plotDistanceTargetM = ticksToDistanceM(targetTicks);
    plotDistanceStartSet = false;
    plotDistanceReached = false;
    Serial.print("ACK:PLOT_TICKS|ticks=");
    Serial.print(targetTicks);
    Serial.print("|meters=");
    Serial.println(plotDistanceTargetM, 3);
  } else if (cmd.startsWith("WHEEL RADIUS ")) {
    float newRadius = cmd.substring(13).toFloat();
    if (newRadius > 0.0) {
      wheelRadiusM = newRadius;
      plotDistanceStartSet = false;
      plotDistanceReached = false;
      Serial.print("ACK:WHEEL_RADIUS|meters=");
      Serial.println(wheelRadiusM, 3);
    } else {
      Serial.println("ERROR:Wheel_radius_must_be_positive");
    }
  } else if (cmd == "STATUS") {
    printStatus();
  } else if (cmd == "HELP") {
    printHelp();
  } else {
    Serial.print("ERROR:Unknown_command|cmd=");
    Serial.println(cmd);
    printHelp();
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setTimeout(100);

  pinMode(M1_RPWM, OUTPUT);
  pinMode(M1_LPWM, OUTPUT);
  pinMode(M1_R_EN, OUTPUT);
  pinMode(M1_L_EN, OUTPUT);
  pinMode(M2_RPWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);
  pinMode(M2_R_EN, OUTPUT);
  pinMode(M2_L_EN, OUTPUT);
  pinMode(PLOTTER_RPWM, OUTPUT);
  pinMode(PLOTTER_LPWM, OUTPUT);
  pinMode(PLOTTER_R_EN, OUTPUT);
  pinMode(PLOTTER_L_EN, OUTPUT);
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  digitalWrite(M1_R_EN, HIGH);
  digitalWrite(M1_L_EN, HIGH);
  digitalWrite(M2_R_EN, HIGH);
  digitalWrite(M2_L_EN, HIGH);
  digitalWrite(PLOTTER_R_EN, HIGH);
  digitalWrite(PLOTTER_L_EN, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), onLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B), onLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), onRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), onRightB, CHANGE);

  Wire.begin();
  compass.init();

  stopAll();
  readEncoderTicks(lastSpeedLeftTicks, lastSpeedRightTicks);
  lastSpeedSampleMs = millis();

  Serial.print("READY:Combined_RoboScan_controller|baud=");
  Serial.println(SERIAL_BAUD_RATE);
  printHelp();
}

void loop() {
  updateMeasuredSpeed();
  updatePlotter();
  printStatusIfDue();

  if (Serial.available() > 0) {
    handleCommand(Serial.readStringUntil('\n'));
  }
}
