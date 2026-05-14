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
  
  // Motor pin setup
  pinMode(LEFT_RPWM, OUTPUT);  pinMode(LEFT_LPWM, OUTPUT);
  pinMode(LEFT_R_EN, OUTPUT);  pinMode(LEFT_L_EN, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT); pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(RIGHT_R_EN, OUTPUT); pinMode(RIGHT_L_EN, OUTPUT);

  // Enable motors
  digitalWrite(LEFT_R_EN, HIGH);  digitalWrite(LEFT_L_EN, HIGH);
  digitalWrite(RIGHT_R_EN, HIGH); digitalWrite(RIGHT_L_EN, HIGH);

  // Encoder pin setup
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoderISR2, CHANGE);

  stopMotors();

  // Compass setup
  if(!mag.begin()) {
    Serial.println("ERROR:Compass_not_found");
    while(1);
  }
  
  Serial.println("READY:Arduino_initialized");
}

void loop() {
  // Read Serial Commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
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

  Serial.print("DATA:COMPASS|");
  Serial.print("x="); Serial.print(event.magnetic.x, 2); Serial.print("|");
  Serial.print("y="); Serial.print(event.magnetic.y, 2); Serial.print("|");
  Serial.print("z="); Serial.print(event.magnetic.z, 2); Serial.print("|");
  Serial.print("heading="); Serial.println(headingDegrees, 2);
}

void sendEncoderData() {
  Serial.print("DATA:ENCODER|");
  Serial.print("e1="); Serial.print(encoder1Count); Serial.print("|");
  Serial.print("e2="); Serial.print(encoder2Count); Serial.print("|");
  Serial.print("lspeed="); Serial.print(leftMotorSpeed); Serial.print("|");
  Serial.print("rspeed="); Serial.print(rightMotorSpeed); Serial.print("|");
  Serial.print("ldir="); Serial.print(leftMotorDir); Serial.print("|");
  Serial.print("rdir="); Serial.println(rightMotorDir);
}

void handleCommand(String cmd) {
  cmd.trim();
  
  if (cmd == "STOP" || cmd == "S") {
    stopMotors();
    Serial.println("ACK:STOP");
  } 
  else if (cmd.startsWith("FORWARD ")) {
    int speed = cmd.substring(8).toInt();
    setForward(speed);
    Serial.print("ACK:FORWARD|speed="); Serial.println(speed);
  }
  else if (cmd.startsWith("BACKWARD ")) {
    int speed = cmd.substring(9).toInt();
    setBackward(speed);
    Serial.print("ACK:BACKWARD|speed="); Serial.println(speed);
  }
  else if (cmd.startsWith("LEFT ")) {
    int speed = cmd.substring(5).toInt();
    spinLeft(speed);
    Serial.print("ACK:LEFT|speed="); Serial.println(speed);
  }
  else if (cmd.startsWith("RIGHT ")) {
    int speed = cmd.substring(6).toInt();
    spinRight(speed);
    Serial.print("ACK:RIGHT|speed="); Serial.println(speed);
  }
  else if (cmd.startsWith("MOTOR ")) {
    // Format: MOTOR L <speed> R <speed>
    // Example: MOTOR L 200 R 150
    handleMotorCommand(cmd.substring(6));
  }
  else if (cmd == "STATUS") {
    sendStatus();
  }
  else {
    Serial.print("ERROR:Unknown_command|"); Serial.println(cmd);
  }
}

void handleMotorCommand(String params) {
  // Parse "L <speed> R <speed>"
  int lIndex = params.indexOf('L');
  int rIndex = params.indexOf('R');
  
  if (lIndex != -1 && rIndex != -1) {
    String lSpeedStr = params.substring(lIndex + 1, rIndex).trim();
    String rSpeedStr = params.substring(rIndex + 1).trim();
    
    int lSpeed = lSpeedStr.toInt();
    int rSpeed = rSpeedStr.toInt();
    
    // Set motors independently
    setMotor(0, lSpeed);
    setMotor(1, rSpeed);
    
    Serial.print("ACK:MOTOR|L="); Serial.print(lSpeed); Serial.print("|R="); Serial.println(rSpeed);
  }
}

void setMotor(int motor, int speed) {
  speed = constrain(speed, -255, 255);
  
  if (motor == 0) {  // Left motor
    leftMotorSpeed = abs(speed);
    leftMotorDir = (speed == 0) ? 0 : (speed > 0 ? 1 : -1);
    
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
  Serial.print("STATUS:|lspeed="); Serial.print(leftMotorSpeed); Serial.print("|");
  Serial.print("rspeed="); Serial.print(rightMotorSpeed); Serial.print("|");
  Serial.print("e1="); Serial.print(encoder1Count); Serial.print("|");
  Serial.print("e2="); Serial.println(encoder2Count);
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
