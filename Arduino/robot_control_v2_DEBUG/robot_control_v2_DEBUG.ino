#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// ── LEFT Motor (M1)
const int LEFT_RPWM = 5,  LEFT_LPWM = 6,  LEFT_R_EN = 7,  LEFT_L_EN = 8;

// ── RIGHT Motor (M2)
const int RIGHT_RPWM = 9, RIGHT_LPWM = 10, RIGHT_R_EN = 11, RIGHT_L_EN = 12;

// ── Encoder pins
const int ENC1_A = 2, ENC1_B = 4;
const int ENC2_A = 3, ENC2_B = 13;

// Compass sensor
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Timing
unsigned long lastImuTime = 0;
unsigned long lastEncoderTime = 0;
const unsigned long IMU_INTERVAL = 100;      // 10Hz compass updates
const unsigned long ENCODER_INTERVAL = 50;   // 20Hz encoder updates

// Motor state tracking
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
int leftMotorDir = 0;  // -1: backward, 0: stop, 1: forward
int rightMotorDir = 0;

// Encoder counters
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;

void setup() {
  Serial.begin(115200);
  delay(500);  // Give serial monitor time to connect
  
  Serial.println(F("\n========================================"));
  Serial.println(F("ROBOT CONTROL V2 - DEBUG MODE"));
  Serial.println(F("========================================"));
  Serial.println(F("[SETUP] Initializing..."));
  
  // Motor pin setup
  Serial.println(F("[SETUP] Configuring motor pins..."));
  pinMode(LEFT_RPWM, OUTPUT);  pinMode(LEFT_LPWM, OUTPUT);
  pinMode(LEFT_R_EN, OUTPUT);  pinMode(LEFT_L_EN, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT); pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(RIGHT_R_EN, OUTPUT); pinMode(RIGHT_L_EN, OUTPUT);
  Serial.println(F("       ✓ Motor pins configured"));

  // Enable motors
  Serial.println(F("[SETUP] Enabling motor drivers..."));
  digitalWrite(LEFT_R_EN, HIGH);  digitalWrite(LEFT_L_EN, HIGH);
  digitalWrite(RIGHT_R_EN, HIGH); digitalWrite(RIGHT_L_EN, HIGH);
  Serial.println(F("       ✓ Motor drivers enabled"));

  // Encoder pin setup
  Serial.println(F("[SETUP] Configuring encoder pins..."));
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);
  Serial.println(F("       ✓ Encoder pins configured"));

  // Attach interrupts for encoders
  Serial.println(F("[SETUP] Attaching encoder interrupts..."));
  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoderISR2, CHANGE);
  Serial.println(F("       ✓ Encoder interrupts attached"));

  stopMotors();
  Serial.println(F("[SETUP] Motors stopped (safe state)"));

  // Compass setup
  Serial.println(F("[SETUP] Initializing compass sensor..."));
  if(!mag.begin()) {
    Serial.println(F("       ✗ ERROR: Compass not found!"));
    Serial.println(F("       Check I2C connections and sensor wiring"));
    while(1);
  }
  Serial.println(F("       ✓ Compass initialized successfully"));
  
  Serial.println(F("\n========================================"));
  Serial.println(F("READY: Arduino initialized successfully"));
  Serial.println(F("Baud Rate: 115200"));
  Serial.println(F("========================================\n"));
}

void loop() {
  // Read Serial Commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    Serial.print(F("[COMMAND] Received: '"));
    Serial.print(cmd);
    Serial.println(F("'"));
    handleCommand(cmd);
  }

  // Periodic sensor data streaming
  unsigned long now = millis();
  
  if (now - lastImuTime >= IMU_INTERVAL) {
    lastImuTime = now;
    sendCompassData();
  }

  if (now - lastEncoderTime >= ENCODER_INTERVAL) {
    lastEncoderTime = now;
    sendEncoderData();
  }
}

void sendCompassData() {
  sensors_event_t event; 
  mag.getEvent(&event);
  
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  if(heading < 0) heading += 2*PI;
  if(heading > 2*PI) heading -= 2*PI;
  
  float headingDegrees = heading * 180/M_PI;

  Serial.print(F("[COMPASS] "));
  Serial.print(F("X=")); Serial.print(event.magnetic.x, 2); Serial.print(F(" | "));
  Serial.print(F("Y=")); Serial.print(event.magnetic.y, 2); Serial.print(F(" | "));
  Serial.print(F("Z=")); Serial.print(event.magnetic.z, 2); Serial.print(F(" | "));
  Serial.print(F("Heading=")); Serial.print(headingDegrees, 2); Serial.println(F("°"));
  
  // Also send in original format for compatibility
  Serial.print(F("DATA:COMPASS|"));
  Serial.print(F("x=")); Serial.print(event.magnetic.x, 2); Serial.print(F("|"));
  Serial.print(F("y=")); Serial.print(event.magnetic.y, 2); Serial.print(F("|"));
  Serial.print(F("z=")); Serial.print(event.magnetic.z, 2); Serial.print(F("|"));
  Serial.print(F("heading=")); Serial.println(headingDegrees, 2);
}

void sendEncoderData() {
  Serial.print(F("[ENCODER] "));
  Serial.print(F("E1=")); Serial.print(encoder1Count); Serial.print(F(" | "));
  Serial.print(F("E2=")); Serial.print(encoder2Count); Serial.print(F(" | "));
  Serial.print(F("L_Speed=")); Serial.print(leftMotorSpeed); Serial.print(F(" | "));
  Serial.print(F("R_Speed=")); Serial.print(rightMotorSpeed); Serial.print(F(" | "));
  Serial.print(F("L_Dir=")); Serial.print(leftMotorDir); Serial.print(F(" | "));
  Serial.print(F("R_Dir=")); Serial.println(rightMotorDir);
  
  // Also send in original format for compatibility
  Serial.print(F("DATA:ENCODER|"));
  Serial.print(F("e1=")); Serial.print(encoder1Count); Serial.print(F("|"));
  Serial.print(F("e2=")); Serial.print(encoder2Count); Serial.print(F("|"));
  Serial.print(F("lspeed=")); Serial.print(leftMotorSpeed); Serial.print(F("|"));
  Serial.print(F("rspeed=")); Serial.print(rightMotorSpeed); Serial.print(F("|"));
  Serial.print(F("ldir=")); Serial.print(leftMotorDir); Serial.print(F("|"));
  Serial.print(F("rdir=")); Serial.println(rightMotorDir);
}

void handleCommand(String cmd) {
  cmd.trim();
  
  if (cmd == "STOP" || cmd == "S") {
    Serial.println(F("[ACTION] Stopping motors..."));
    stopMotors();
    Serial.println(F("   ✓ Motors stopped"));
    Serial.println(F("ACK:STOP"));
  } 
  else if (cmd.startsWith("FORWARD ")) {
    int speed = cmd.substring(8).toInt();
    Serial.print(F("[ACTION] Moving forward with speed: "));
    Serial.println(speed);
    setForward(speed);
    Serial.print(F("ACK:FORWARD|speed=")); Serial.println(speed);
  }
  else if (cmd.startsWith("BACKWARD ")) {
    int speed = cmd.substring(9).toInt();
    Serial.print(F("[ACTION] Moving backward with speed: "));
    Serial.println(speed);
    setBackward(speed);
    Serial.print(F("ACK:BACKWARD|speed=")); Serial.println(speed);
  }
  else if (cmd.startsWith("LEFT ")) {
    int speed = cmd.substring(5).toInt();
    Serial.print(F("[ACTION] Spinning left with speed: "));
    Serial.println(speed);
    spinLeft(speed);
    Serial.print(F("ACK:LEFT|speed=")); Serial.println(speed);
  }
  else if (cmd.startsWith("RIGHT ")) {
    int speed = cmd.substring(6).toInt();
    Serial.print(F("[ACTION] Spinning right with speed: "));
    Serial.println(speed);
    spinRight(speed);
    Serial.print(F("ACK:RIGHT|speed=")); Serial.println(speed);
  }
  else if (cmd.startsWith("MOTOR ")) {
    Serial.print(F("[ACTION] Setting individual motor speeds: "));
    Serial.println(cmd.substring(6));
    handleMotorCommand(cmd.substring(6));
  }
  else if (cmd == "STATUS") {
    Serial.println(F("[REQUEST] Status requested"));
    sendStatus();
  }
  else {
    Serial.print(F("ERROR:Unknown_command|")); Serial.println(cmd);
  }
  
  Serial.println();
}

void handleMotorCommand(String params) {
  // Parse "L <speed> R <speed>"
  int lIndex = params.indexOf('L');
  int rIndex = params.indexOf('R');
  
  if (lIndex != -1 && rIndex != -1) {
    String lSpeedStr = params.substring(lIndex + 1, rIndex);
    lSpeedStr.trim();
    String rSpeedStr = params.substring(rIndex + 1);
    rSpeedStr.trim();
    
    int lSpeed = lSpeedStr.toInt();
    int rSpeed = rSpeedStr.toInt();
    
    Serial.print(F("   Left Motor: ")); Serial.print(lSpeed);
    Serial.print(F(" | Right Motor: ")); Serial.println(rSpeed);
    
    // Set motors independently
    setMotor(0, lSpeed);
    setMotor(1, rSpeed);
    
    Serial.print(F("ACK:MOTOR|L=")); Serial.print(lSpeed); Serial.print(F("|R=")); Serial.println(rSpeed);
  } else {
    Serial.println(F("[ERROR] Invalid motor command format. Use: MOTOR L <speed> R <speed>"));
  }
}

void setMotor(int motor, int speed) {
  speed = constrain(speed, -255, 255);
  
  String motorName = (motor == 0) ? "LEFT" : "RIGHT";
  String direction = (speed > 0) ? "FORWARD" : (speed < 0) ? "BACKWARD" : "STOP";
  
  if (motor == 0) {  // Left motor
    leftMotorSpeed = abs(speed);
    leftMotorDir = (speed == 0) ? 0 : (speed > 0 ? 1 : -1);
    
    Serial.print(F("   [MOTOR_SET] "));
    Serial.print(motorName);
    Serial.print(F(" - Direction: "));
    Serial.print(direction);
    Serial.print(F(" | PWM Value: "));
    Serial.println(speed);
    
    if (speed > 0) {
      analogWrite(LEFT_LPWM, 0);
      analogWrite(LEFT_RPWM, speed);
    } else if (speed < 0) {
      analogWrite(LEFT_RPWM, 0);
      analogWrite(LEFT_LPWM, -speed);
    } else {
      analogWrite(LEFT_RPWM, 0);
      analogWrite(LEFT_LPWM, 0);
    }
  }
  else if (motor == 1) {  // Right motor
    rightMotorSpeed = abs(speed);
    rightMotorDir = (speed == 0) ? 0 : (speed > 0 ? 1 : -1);
    
    Serial.print(F("   [MOTOR_SET] "));
    Serial.print(motorName);
    Serial.print(F(" - Direction: "));
    Serial.print(direction);
    Serial.print(F(" | PWM Value: "));
    Serial.println(speed);
    
    if (speed > 0) {
      analogWrite(RIGHT_LPWM, 0);
      analogWrite(RIGHT_RPWM, speed);
    } else if (speed < 0) {
      analogWrite(RIGHT_RPWM, 0);
      analogWrite(RIGHT_LPWM, -speed);
    } else {
      analogWrite(RIGHT_RPWM, 0);
      analogWrite(RIGHT_LPWM, 0);
    }
  }
}

void sendStatus() {
  Serial.println(F("\n--- ROBOT STATUS ---"));
  Serial.print(F("Left Motor Speed: ")); Serial.print(leftMotorSpeed);
  Serial.print(F(" | Direction: ")); Serial.println((leftMotorDir > 0) ? "FWD" : (leftMotorDir < 0) ? "BWD" : "STOP");
  
  Serial.print(F("Right Motor Speed: ")); Serial.print(rightMotorSpeed);
  Serial.print(F(" | Direction: ")); Serial.println((rightMotorDir > 0) ? "FWD" : (rightMotorDir < 0) ? "BWD" : "STOP");
  
  Serial.print(F("Encoder 1 Count: ")); Serial.println(encoder1Count);
  Serial.print(F("Encoder 2 Count: ")); Serial.println(encoder2Count);
  Serial.println(F("-------------------\n"));
  
  // Also send in original format for compatibility
  Serial.print(F("STATUS:|lspeed=")); Serial.print(leftMotorSpeed); Serial.print(F("|"));
  Serial.print(F("rspeed=")); Serial.print(rightMotorSpeed); Serial.print(F("|"));
  Serial.print(F("e1=")); Serial.print(encoder1Count); Serial.print(F("|"));
  Serial.print(F("e2=")); Serial.println(encoder2Count);
}

void stopMotors() {
  setMotor(0, 0);
  setMotor(1, 0);
}

void setForward(int speed) {
  speed = constrain(speed, 0, 255);
  setMotor(0, speed);
  setMotor(1, speed);
}

void setBackward(int speed) {
  speed = constrain(speed, 0, 255);
  setMotor(0, -speed);
  setMotor(1, -speed);
}

void spinLeft(int speed) {
  speed = constrain(speed, 0, 255);
  setMotor(0, -speed);
  setMotor(1, speed);
}

void spinRight(int speed) {
  speed = constrain(speed, 0, 255);
  setMotor(0, speed);
  setMotor(1, -speed);
}

// Encoder interrupt handlers
void encoderISR1() {
  encoder1Count++;
}

void encoderISR2() {
  encoder2Count++;
}

