#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>

// ── LEFT Motor (M1)
const int LEFT_RPWM = 5,  LEFT_LPWM = 6,  LEFT_R_EN = 7,  LEFT_L_EN = 8;

// ── RIGHT Motor (M2)
const int RIGHT_RPWM = 9, RIGHT_LPWM = 10, RIGHT_R_EN = 11, RIGHT_L_EN = 12;

const int RAMP_STEPS = 50;
const unsigned long RAMP_TIME = 800;

int currentSpeed = 0;
int currentDir   = 0; // 1=fwd, -1=back, 0=stop

// ── IMU streaming state
bool imuStreaming = false;
unsigned long imuInterval = 100;   // ms between IMU packets (default 10 Hz)
unsigned long lastImuTime  = 0;

FaBo9Axis fabo_9axis;

// ── Non-blocking turn state machine
enum TurnState { TURN_IDLE, TURN_RAMP_DOWN, TURN_SETTLE, TURN_SPINNING, TURN_DONE };
TurnState turnState = TURN_IDLE;

// Turn parameters (set when a turn command starts)
int    turnDir       = 0;      // +1=cw, -1=ccw
int    turnSpeed     = 0;
float  turnTargetDeg = 0.0;
int    turnRampStep  = 0;
unsigned long turnStepStart    = 0;   // for ramp-down settle timing
unsigned long turnSpinStart    = 0;   // absolute start of spinning (for timeout)
unsigned long turnLastCheck    = 0;   // last heading check time (for 10ms interval)
unsigned long turnTimeout      = 10000;
const float TURN_HEADING_TOL = 5.0;

// ── Non-blocking ramp state machine (for FORWARD/BACKWARD)
enum RampState { RAMP_IDLE, RAMP_RAMPING };
RampState rampState = RAMP_IDLE;
int       rampDir   = 0;
int       rampTargetSpeed = 0;
int       rampStep  = 0;
unsigned long rampStepStart = 0;

// ---------- Motor primitives ----------
void emergencyStop() {
  analogWrite(LEFT_RPWM, 0);  analogWrite(LEFT_LPWM, 0);
  analogWrite(RIGHT_RPWM, 0); analogWrite(RIGHT_LPWM, 0);
  currentSpeed = 0;
  currentDir = 0;
  // Reset all state machines
  turnState = TURN_IDLE;
  rampState = RAMP_IDLE;
  Serial.println("!! EMERGENCY STOP !!");
}

void stopMotors() {
  analogWrite(LEFT_RPWM, 0);  analogWrite(LEFT_LPWM, 0);
  analogWrite(RIGHT_RPWM, 0); analogWrite(RIGHT_LPWM, 0);
  currentSpeed = 0;
  currentDir = 0;
  turnState = TURN_IDLE;
  rampState = RAMP_IDLE;
  Serial.println("Motors stopped.");
}

void setForward(int speed) {
  analogWrite(LEFT_LPWM, 0);      analogWrite(RIGHT_LPWM, 0);
  analogWrite(LEFT_RPWM, speed);  analogWrite(RIGHT_RPWM, speed);
}

void setBackward(int speed) {
  analogWrite(LEFT_RPWM, 0);      analogWrite(RIGHT_RPWM, 0);
  analogWrite(LEFT_LPWM, speed);  analogWrite(RIGHT_LPWM, speed);
}

void spinRight(int speed) {
  analogWrite(LEFT_LPWM, 0);     analogWrite(LEFT_RPWM, speed);
  analogWrite(RIGHT_RPWM, 0);    analogWrite(RIGHT_LPWM, speed);
}

void spinLeft(int speed) {
  analogWrite(LEFT_RPWM, 0);     analogWrite(LEFT_LPWM, speed);
  analogWrite(RIGHT_LPWM, 0);    analogWrite(RIGHT_RPWM, speed);
}

// ---------- IMU helpers ----------
float readHeadingDeg() {
  float mx, my, mz;
  fabo_9axis.readMagnetXYZ(&mx, &my, &mz);
  float heading = atan2(my, mx) * 180.0 / PI;
  if (heading < 0) heading += 360.0;
  return heading;
}

float angleDiffDeg(float fromDeg, float toDeg) {
  float d = toDeg - fromDeg;
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

// Send one IMU packet over Serial as a single CSV line:
// IMU,ax,ay,az,gx,gy,gz,heading
void sendImuPacket() {
  float ax, ay, az, gx, gy, gz;
  fabo_9axis.readAccelXYZ(&ax, &ay, &az);
  fabo_9axis.readGyroXYZ(&gx, &gy, &gz);
  float heading = readHeadingDeg();

  Serial.print("IMU,");
  Serial.print(ax, 3); Serial.print(",");
  Serial.print(ay, 3); Serial.print(",");
  Serial.print(az, 3); Serial.print(",");
  Serial.print(gx, 3); Serial.print(",");
  Serial.print(gy, 3); Serial.print(",");
  Serial.print(gz, 3); Serial.print(",");
  Serial.println(heading, 2);
}

// ── Non-blocking turn state machine tick — call every loop()
// Returns true while a turn is in progress, false when idle.
bool turnTick() {
  unsigned long now = millis();
  unsigned long stepDelay = RAMP_TIME / RAMP_STEPS;

  switch (turnState) {

    case TURN_IDLE:
      return false;

    case TURN_RAMP_DOWN: {
      if (now - turnStepStart < stepDelay) return true;
      int speed = (turnRampStep * currentSpeed) / RAMP_STEPS;
      if (currentDir == 1) setForward(speed);
      else setBackward(speed);
      turnRampStep--;
      turnStepStart = now;
      if (turnRampStep < 0) {
        stopMotors();
        turnState = TURN_SETTLE;
        turnStepStart = now;
      }
      return true;
    }

    case TURN_SETTLE:
      if (now - turnStepStart >= 150) {
        float startH = readHeadingDeg();
        Serial.print("Turn start heading: "); Serial.println(startH);
        Serial.print("Turn target heading: "); Serial.println(turnTargetDeg);
        turnState      = TURN_SPINNING;
        turnSpinStart  = millis();
        turnLastCheck  = millis();
        if (turnDir == 1) spinRight(turnSpeed);
        else spinLeft(turnSpeed);
      }
      return true;

    case TURN_SPINNING: {
      // Check heading every ~10ms
      if (now - turnLastCheck < 10) return true;
      turnLastCheck = now;

      float nowH = readHeadingDeg();
      if (abs(angleDiffDeg(nowH, turnTargetDeg)) <= TURN_HEADING_TOL) {
        stopMotors();
        Serial.print("Turn complete. End heading: "); Serial.println(readHeadingDeg());
        turnState = TURN_DONE;
        return false;
      }

      // Timeout check against the original spin start time
      if (now - turnSpinStart > turnTimeout) {
        stopMotors();
        Serial.println("Turn timeout.");
        turnState = TURN_DONE;
        return false;
      }

      return true;
    }

    case TURN_DONE:
      turnState = TURN_IDLE;
      return false;
  }
  return false;
}

// ── Non-blocking ramp state machine tick — call every loop()
bool rampTick() {
  if (rampState == RAMP_IDLE) return false;

  unsigned long now = millis();
  unsigned long stepDelay = RAMP_TIME / RAMP_STEPS;

  if (now - rampStepStart < stepDelay) return true;

  int speed = (rampStep * rampTargetSpeed) / RAMP_STEPS;
  if (rampDir == 1) setForward(speed);
  else setBackward(speed);

  rampStep++;
  rampStepStart = now;

  if (rampStep > RAMP_STEPS) {
    currentSpeed = rampTargetSpeed;
    currentDir = rampDir;
    rampState = RAMP_IDLE;
    return false;
  }
  return true;
}

// ── Start a non-blocking turn — returns immediately
// dir: +1 right(cw), -1 left(ccw)
void startTurn(int dir, int speed) {
  if (speed < 1) speed = 120;

  turnDir   = dir;
  turnSpeed = speed;

  // Calculate relative target heading: ±90° from current
  float currentH = readHeadingDeg();
  turnTargetDeg = currentH + (dir * 90.0);
  if (turnTargetDeg >= 360.0) turnTargetDeg -= 360.0;
  if (turnTargetDeg < 0.0)    turnTargetDeg += 360.0;

  Serial.print("Turn from heading: "); Serial.println(currentH);
  Serial.print("Turn target heading: "); Serial.println(turnTargetDeg);

  if (currentDir != 0) {
    // Begin ramp-down phase
    turnRampStep  = RAMP_STEPS;
    turnStepStart = millis();
    turnState     = TURN_RAMP_DOWN;
  } else {
    // Skip ramp-down, go straight to settle then spin
    turnState     = TURN_SETTLE;
    turnStepStart = millis();
  }
}

// ---------- Command handling ----------
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  // Emergency stop — always works, even mid-turn
  if (cmd == "S") {
    emergencyStop();
    return;
  }

  if (cmd == "STOP") {
    if (currentDir == 0 && turnState == TURN_IDLE) Serial.println("Already stopped.");
    else stopMotors();
    return;
  }

  // IMU_STREAM [interval_ms]  — start streaming IMU data
  if (cmd == "IMU_STREAM" || cmd.startsWith("IMU_STREAM ")) {
    if (cmd.length() > 11) {
      int ms = cmd.substring(11).toInt();
      if (ms >= 20 && ms <= 2000) imuInterval = ms;
    }
    imuStreaming = true;
    Serial.print("IMU streaming started @ ");
    Serial.print(imuInterval);
    Serial.println(" ms");
    return;
  }

  // IMU_STOP — stop streaming
  if (cmd == "IMU_STOP") {
    imuStreaming = false;
    Serial.println("IMU streaming stopped.");
    return;
  }

  // IMU_READ — single one-shot IMU read
  if (cmd == "IMU_READ") {
    sendImuPacket();
    return;
  }

  // SET_SPEED <0-255> — update currentSpeed without changing direction
  if (cmd.startsWith("SET_SPEED ")) {
    int speed = cmd.substring(10).toInt();
    if (speed < 0 || speed > 255) {
      Serial.println("ERROR: Speed must be 0-255.");
      return;
    }
    if (currentDir == 1) setForward(speed);
    else if (currentDir == -1) setBackward(speed);
    currentSpeed = speed;
    Serial.print("Speed set to "); Serial.println(speed);
    return;
  }

  // FORWARD / BACKWARD — start non-blocking ramp
  if (cmd.startsWith("FORWARD ") || cmd.startsWith("BACKWARD ")) {
    int spaceIdx = cmd.indexOf(' ');
    String dirStr = cmd.substring(0, spaceIdx);
    int speed = cmd.substring(spaceIdx + 1).toInt();

    if (speed < 0 || speed > 255) {
      Serial.println("ERROR: Speed must be 0-255.");
      return;
    }

    // Cancel any active turn
    if (turnState != TURN_IDLE) {
      turnState = TURN_IDLE;
      stopMotors();
    }

    rampDir   = dir;
    rampTargetSpeed = speed;
    rampStep  = 0;
    rampState = RAMP_RAMPING;
    return;
  }

  // TURN_LEFT_90 — turn left 90° relative to current heading
  if (cmd.startsWith("TURN_LEFT_90 ")) {
    int speed = cmd.substring(13).toInt();
    if (speed < 1 || speed > 255) {
      Serial.println("ERROR: Speed must be 1-255.");
      return;
    }
    startTurn(-1, speed);
    return;
  }

  // TURN_RIGHT_90 — turn right 90° relative to current heading
  if (cmd.startsWith("TURN_RIGHT_90 ")) {
    int speed = cmd.substring(14).toInt();
    if (speed < 1 || speed > 255) {
      Serial.println("ERROR: Speed must be 1-255.");
      return;
    }
    startTurn(1, speed);
    return;
  }

  Serial.println("Unknown command.");
  Serial.println("Commands:");
  Serial.println("  FORWARD <0-255>");
  Serial.println("  BACKWARD <0-255>");
  Serial.println("  SET_SPEED <0-255>");
  Serial.println("  TURN_LEFT_90 <1-255>");
  Serial.println("  TURN_RIGHT_90 <1-255>");
  Serial.println("  STOP | S");
  Serial.println("  IMU_STREAM [interval_ms]");
  Serial.println("  IMU_STOP");
  Serial.println("  IMU_READ");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  pinMode(LEFT_RPWM, OUTPUT);  pinMode(LEFT_LPWM, OUTPUT);
  pinMode(LEFT_R_EN, OUTPUT);  pinMode(LEFT_L_EN, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT); pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(RIGHT_R_EN, OUTPUT); pinMode(RIGHT_L_EN, OUTPUT);

  stopMotors();

  digitalWrite(LEFT_R_EN, HIGH);  digitalWrite(LEFT_L_EN, HIGH);
  digitalWrite(RIGHT_R_EN, HIGH); digitalWrite(RIGHT_L_EN, HIGH);

  Wire.begin();
  Serial.println("configuring 9axis...");
  if (fabo_9axis.begin()) {
    Serial.println("configured FaBo 9Axis I2C Brick");
  } else {
    Serial.println("device error");
    while (1);
  }

  Serial.println("Ready.");
}

void loop() {
  // ── Non-blocking IMU streaming
  if (imuStreaming) {
    unsigned long now = millis();
    if (now - lastImuTime >= imuInterval) {
      lastImuTime = now;
      sendImuPacket();
    }
  }

  // ── Non-blocking ramp state machine
  rampTick();

  // ── Non-blocking turn state machine
  turnTick();

  // ── Serial command handling (always runs — even during turns)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }
}
