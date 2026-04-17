#include <MPU9250.h> // Library by Hideaki Tai

/*
 * NOTE: This sketch uses the "MPU9250" library by Hideaki Tai.
 * It is much better suited for the 8-bit Arduino Mega 2560 
 * and includes built-in Mahony sensor fusion.
 */

MPU9250 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  
  // Initialize the MPU9250 at address 0x68
  if (!mpu.setup(0x68)) {
    Serial.println("{\"error\": \"MPU9250 initialization failed. Check connections.\"}");
    while (1) { delay(1000); }
  }

  // Optional: Uncomment these to calibrate for better accuracy
  // Serial.println("Calibrating... keep sensor level and still.");
  // mpu.calibrateAccelGyro();
  // mpu.calibrateMag();
}

void loop() {
  // mpu.update() performs the sensor fusion calculation internally
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    
    // Output JSON every 20ms (~50Hz) for smooth HUD responsiveness
    if (millis() - prev_ms > 20) {
      Serial.print("{\"yaw\":");
      Serial.print(mpu.getYaw(), 2);
      Serial.print(",\"pitch\":");
      Serial.print(mpu.getPitch(), 2);
      Serial.print(",\"roll\":");
      Serial.print(mpu.getRoll(), 2);
      Serial.println("}");
      
      prev_ms = millis();
    }
  }
}


