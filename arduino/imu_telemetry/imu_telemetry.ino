#include <MPU9250.h> // Library by Hideaki Tai

/*
 * NOTE: This sketch uses the "MPU9250" library by Hideaki Tai.
 * It is much better suited for the 8-bit Arduino Mega 2560 
 * and includes built-in Mahony sensor fusion.
 *
 * YAW / HEADING:
 *   The Madgwick/Mahony filter is used for smooth pitch & roll only.
 *   For absolute compass heading (N/E/S/W), we compute a 
 *   tilt-compensated magnetic heading directly from the calibrated
 *   magnetometer + accelerometer data. This gives a true compass 
 *   bearing that updates even when the sensor is stationary.
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

// Magnetic declination for your location (degrees)
// Find yours at: http://www.magnetic-declination.com/
// Cairo, Egypt ≈ 4.5° — change this to your city's value
const float MAGNETIC_DECLINATION = 4.5;

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

  // Use Madgwick filter for pitch/roll stabilization
  mpu.selectFilter(QuatFilterSel::MADGWICK);
  mpu.setFilterIterations(15);

  // Set magnetic declination
  mpu.setMagneticDeclination(MAGNETIC_DECLINATION);

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

/*
 * Compute tilt-compensated magnetic heading.
 * This uses raw accelerometer data to determine pitch & roll,
 * then rotates the magnetometer vector into the horizontal plane
 * to compute a true compass bearing (0-360°, clockwise from North).
 */
float computeCompassHeading() {
  // Get raw accelerometer values (in g)
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();

  // Get calibrated magnetometer values (in µT)
  float mx = mpu.getMagX();
  float my = mpu.getMagY();
  float mz = mpu.getMagZ();

  // Compute pitch and roll from accelerometer (radians)
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  float roll  = atan2(ay, az);

  // Tilt-compensate the magnetometer readings
  float cosP = cos(pitch);
  float sinP = sin(pitch);
  float cosR = cos(roll);
  float sinR = sin(roll);

  // Rotate mag vector to horizontal plane
  float magX_h = mx * cosP + my * sinR * sinP + mz * cosR * sinP;
  float magY_h = my * cosR - mz * sinR;

  // Compute heading in degrees (0-360, clockwise from North)
  float heading = atan2(-magY_h, magX_h) * 180.0 / PI;

  // Apply magnetic declination
  heading += MAGNETIC_DECLINATION;

  // Normalize to 0-360
  if (heading < 0)    heading += 360.0;
  if (heading >= 360) heading -= 360.0;

  return heading;
}

void loop() {
  // mpu.update() performs the sensor fusion calculation internally
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    
    // Output JSON every 20ms (~50Hz) for smooth HUD responsiveness
    if (millis() - prev_ms > 20) {
      // Use tilt-compensated compass heading for yaw (absolute direction)
      float heading = computeCompassHeading();
      // Use Madgwick filter for smooth pitch & roll
      float pitch = mpu.getPitch();
      float roll  = mpu.getRoll();

      Serial.print("{\"yaw\":");
      Serial.print(heading, 2);
      Serial.print(",\"pitch\":");
      Serial.print(pitch, 2);
      Serial.print(",\"roll\":");
      Serial.print(roll, 2);
      Serial.println("}");
      
      prev_ms = millis();
    }
  }
}
