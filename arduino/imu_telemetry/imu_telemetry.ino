#include <MPU9250.h> // Library by Hideaki Tai

/*
 * NOTE: This sketch uses the "MPU9250" library by Hideaki Tai.
 * It is much better suited for the 8-bit Arduino Mega 2560 
 * and includes built-in Mahony sensor fusion.
 *
 * IMPORTANT: Yaw requires a calibrated magnetometer.
 * At every power-on the sketch runs a two-phase calibration:
 *   1) Accel + Gyro  → keep sensor FLAT and STILL
 *   2) Magnetometer  → slowly rotate sensor in all directions (figure-8)
 *
 * After calibration, it prints the bias/scale values so you can
 * hardcode them later to skip the interactive calibration step.
 */

MPU9250 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  delay(2000);  // Let the sensor stabilize after power-on
  
  // Initialize the MPU9250 at address 0x68
  if (!mpu.setup(0x68)) {
    Serial.println("{\"error\": \"MPU9250 initialization failed. Check connections.\"}");
    while (1) { delay(1000); }
  }

  // Use Madgwick filter with 15 iterations for stable yaw
  mpu.selectFilter(QuatFilterSel::MADGWICK);
  mpu.setFilterIterations(15);

  // Set magnetic declination for your location (degrees)
  // Find yours at: http://www.magnetic-declination.com/
  // Example: Cairo, Egypt ≈ 4.5°  — change this to your city's value
  mpu.setMagneticDeclination(4.5);

  // ── Phase 1: Accel & Gyro calibration ──────────────────────────
  Serial.println("{\"status\": \"Calibrating Accel/Gyro... keep sensor FLAT and STILL for 5 seconds.\"}");
  delay(5000);
  mpu.calibrateAccelGyro();
  Serial.println("{\"status\": \"Accel/Gyro calibration done.\"}");

  // ── Phase 2: Magnetometer calibration ──────────────────────────
  Serial.println("{\"status\": \"Calibrating Mag... slowly rotate sensor in ALL directions (figure-8) for 15 seconds.\"}");
  delay(1000);
  mpu.calibrateMag();
  Serial.println("{\"status\": \"Mag calibration done.\"}");

  // Print calibration values so you can hardcode them later
  Serial.print("{\"cal_accel_bias\": [");
  Serial.print(mpu.getAccBiasX()); Serial.print(",");
  Serial.print(mpu.getAccBiasY()); Serial.print(",");
  Serial.print(mpu.getAccBiasZ());
  Serial.println("]}");

  Serial.print("{\"cal_gyro_bias\": [");
  Serial.print(mpu.getGyroBiasX()); Serial.print(",");
  Serial.print(mpu.getGyroBiasY()); Serial.print(",");
  Serial.print(mpu.getGyroBiasZ());
  Serial.println("]}");

  Serial.print("{\"cal_mag_bias\": [");
  Serial.print(mpu.getMagBiasX()); Serial.print(",");
  Serial.print(mpu.getMagBiasY()); Serial.print(",");
  Serial.print(mpu.getMagBiasZ());
  Serial.println("]}");

  Serial.print("{\"cal_mag_scale\": [");
  Serial.print(mpu.getMagScaleX()); Serial.print(",");
  Serial.print(mpu.getMagScaleY()); Serial.print(",");
  Serial.print(mpu.getMagScaleZ());
  Serial.println("]}");

  Serial.println("{\"status\": \"Calibration complete. Streaming orientation data.\"}");
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
