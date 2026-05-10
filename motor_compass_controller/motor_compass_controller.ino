#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// ── LEFT Motor (M1)
const int LEFT_RPWM = 5,  LEFT_LPWM = 6,  LEFT_R_EN = 7,  LEFT_L_EN = 8;

// ── RIGHT Motor (M2)
const int RIGHT_RPWM = 9, RIGHT_LPWM = 10, RIGHT_R_EN = 11, RIGHT_L_EN = 12;

// ── Encoder pins (Optional for simple control, but kept for layout consistency)
const int ENC1_A = 2, ENC1_B = 4;
const int ENC2_A = 3, ENC2_B = 13;

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

unsigned long lastImuTime = 0;
const unsigned long IMU_INTERVAL = 100; // 10Hz

void setup() {
  Serial.begin(115200);
  
  // Motor setup
  pinMode(LEFT_RPWM, OUTPUT);  pinMode(LEFT_LPWM, OUTPUT);
  pinMode(LEFT_R_EN, OUTPUT);  pinMode(LEFT_L_EN, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT); pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(RIGHT_R_EN, OUTPUT); pinMode(RIGHT_L_EN, OUTPUT);

  // Enable motors
  digitalWrite(LEFT_R_EN, HIGH);  digitalWrite(LEFT_L_EN, HIGH);
  digitalWrite(RIGHT_R_EN, HIGH); digitalWrite(RIGHT_L_EN, HIGH);

  stopMotors();

  // Compass setup
  if(!mag.begin()) {
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  Serial.println("Ready.");
}

void loop() {
  // Read Serial Commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

  // Periodic Compass Data Streaming
  unsigned long now = millis();
  if (now - lastImuTime >= IMU_INTERVAL) {
    lastImuTime = now;
    sendCompassData();
  }
}

void sendCompassData() {
  sensors_event_t event; 
  mag.getEvent(&event);
  
  // Calculate heading
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Correct for when signs are reversed.
  if(heading < 0) heading += 2*PI;
  // Check for wrap due to addition of declination.
  if(heading > 2*PI) heading -= 2*PI;
  
  // Convert radians to degrees
  float headingDegrees = heading * 180/M_PI; 

  Serial.print("MAG,");
  Serial.print(event.magnetic.x); Serial.print(",");
  Serial.print(event.magnetic.y); Serial.print(",");
  Serial.print(event.magnetic.z); Serial.print(",");
  Serial.println(headingDegrees);
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "STOP" || cmd == "S") {
    stopMotors();
    Serial.println("Stopped");
  } 
  else if (cmd.startsWith("FORWARD ")) {
    int speed = cmd.substring(8).toInt();
    setForward(speed);
  }
  else if (cmd.startsWith("BACKWARD ")) {
    int speed = cmd.substring(9).toInt();
    setBackward(speed);
  }
  else if (cmd.startsWith("LEFT ")) {
    int speed = cmd.substring(5).toInt();
    spinLeft(speed);
  }
  else if (cmd.startsWith("RIGHT ")) {
    int speed = cmd.substring(6).toInt();
    spinRight(speed);
  }
}

void stopMotors() {
  analogWrite(LEFT_RPWM, 0);  analogWrite(LEFT_LPWM, 0);
  analogWrite(RIGHT_RPWM, 0); analogWrite(RIGHT_LPWM, 0);
}

void setForward(int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(LEFT_LPWM, 0);     analogWrite(RIGHT_LPWM, 0);
  analogWrite(LEFT_RPWM, speed); analogWrite(RIGHT_RPWM, speed);
}

void setBackward(int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(LEFT_RPWM, 0);     analogWrite(RIGHT_RPWM, 0);
  analogWrite(LEFT_LPWM, speed); analogWrite(RIGHT_LPWM, speed);
}

void spinLeft(int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(LEFT_RPWM, 0);     analogWrite(LEFT_LPWM, speed);
  analogWrite(RIGHT_LPWM, 0);    analogWrite(RIGHT_RPWM, speed);
}

void spinRight(int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(LEFT_LPWM, 0);     analogWrite(LEFT_RPWM, speed);
  analogWrite(RIGHT_RPWM, 0);    analogWrite(RIGHT_LPWM, speed);
}
