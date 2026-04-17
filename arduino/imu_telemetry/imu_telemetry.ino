#include <MPU9250.h> // Library by Hideaki Tai

/*
 * Road Inspector Bot — IMU Telemetry
 *
 * Library: "MPU9250" by Hideaki Tai (optimized for 8-bit AVR / Mega 2560).
 *
 * PITCH & ROLL: Madgwick filter (from the library) — smooth, responsive.
 *
 * YAW (HEADING): Complementary filter blending:
 *   - Gyroscope Z integration (fast, smooth, short-term)
 *   - Tilt-compensated magnetometer heading (absolute, long-term correction)
 *   Formula: yaw = alpha * (yaw + gz*dt) + (1-alpha) * mag_yaw
 *   alpha = 0.98  →  98% gyro, 2% magnetometer per step.
 *
 * CALIBRATION MODE:
 *   Send 'c' over Serial to enter calibration mode.
 *   Arduino will output {"mx":..., "my":..., "mz":...} for 30 seconds
 *   so the RPi script (calibrate_imu.py) can compute hard-iron offsets.
 *   Send 'n' (or wait 30s) to return to normal telemetry mode.
 *
 * IMPORTANT: After running calibration, update MAG_OFFSET_X/Y/Z below.
 */

MPU9250 mpu;

// ── Magnetic declination for your location (degrees) ─────────────
// Find yours at: http://www.magnetic-declination.com/
// Cairo, Egypt ≈ 4.5° — change this to your city's value
const float MAGNETIC_DECLINATION = 4.5;

// ── Hard-iron magnetometer offsets ───────────────────────────────
// Fill these in after running calibrate_imu.py on the RPi.
// They compensate for permanent magnetic biases near the sensor.
const float MAG_OFFSET_X = 0.0;
const float MAG_OFFSET_Y = 0.0;
const float MAG_OFFSET_Z = 0.0;

// ── Complementary filter tuning ──────────────────────────────────
// 0.98 = 98% gyro (smooth, fast) + 2% magnetometer (absolute correction)
const float ALPHA = 0.98;

// ── State variables ──────────────────────────────────────────────
float fusedYaw     = 0.0;   // Complementary-filtered heading (0-360°)
bool  yawInitialized = false;
uint32_t prevMicros = 0;     // For precise dt calculation

// ── Calibration mode state ───────────────────────────────────────
bool     calMode       = false;
uint32_t calStartMs    = 0;
const uint32_t CAL_DURATION_MS = 30000; // 30 seconds

// ── Output timing ────────────────────────────────────────────────
const uint32_t OUTPUT_INTERVAL_MS = 20; // ~50 Hz

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  delay(2000); // Let the sensor stabilize after power-on

  // Initialize the MPU9250 at address 0x68
  if (!mpu.setup(0x68)) {
    Serial.println("{\"error\": \"MPU9250 initialization failed. Check connections.\"}");
    while (1) { delay(1000); }
  }

  // Use Madgwick filter for pitch/roll stabilization
  mpu.selectFilter(QuatFilterSel::MADGWICK);
  mpu.setFilterIterations(15);

  // Set magnetic declination (used internally by the library, but we also
  // apply it ourselves in computeMagHeading for the complementary filter)
  mpu.setMagneticDeclination(MAGNETIC_DECLINATION);

  // ── Phase 1: Accel & Gyro calibration ──────────────────────────
  Serial.println("{\"status\": \"Calibrating Accel/Gyro... keep sensor FLAT and STILL for 5 seconds.\"}");
  delay(5000);
  mpu.calibrateAccelGyro();
  Serial.println("{\"status\": \"Accel/Gyro calibration done.\"}");

  // ── Phase 2: Magnetometer calibration (library built-in) ───────
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

  prevMicros = micros();
}

// ═════════════════════════════════════════════════════════════════
// Normalize an angle to [0, 360)
// ═════════════════════════════════════════════════════════════════
float normalize360(float angle) {
  while (angle < 0.0)    angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  return angle;
}

// ═════════════════════════════════════════════════════════════════
// Compute the shortest signed angular difference from 'from' to 'to'.
// Result is in [-180, +180].  Positive = clockwise.
// ═════════════════════════════════════════════════════════════════
float angleDiff(float to, float from) {
  float d = to - from;
  while (d > 180.0)  d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

// ═════════════════════════════════════════════════════════════════
// Compute tilt-compensated magnetic heading (0-360°, CW from North).
// Hard-iron offsets are applied before the tilt-compensation math.
// ═════════════════════════════════════════════════════════════════
float computeMagHeading() {
  // Accelerometer for tilt (in g)
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();

  // Magnetometer (in µT) — apply hard-iron offsets
  float mx = mpu.getMagX() - MAG_OFFSET_X;
  float my = mpu.getMagY() - MAG_OFFSET_Y;
  float mz = mpu.getMagZ() - MAG_OFFSET_Z;

  // Pitch and roll from accelerometer (radians)
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  float roll  = atan2(ay, az);

  float cosP = cos(pitch);
  float sinP = sin(pitch);
  float cosR = cos(roll);
  float sinR = sin(roll);

  // Rotate mag vector into the horizontal plane
  float magX_h = mx * cosP + my * sinR * sinP + mz * cosR * sinP;
  float magY_h = my * cosR - mz * sinR;

  // Heading (degrees), clockwise from North
  float heading = atan2(-magY_h, magX_h) * 180.0 / PI;
  heading += MAGNETIC_DECLINATION;

  return normalize360(heading);
}

// ═════════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════════
void loop() {

  // ── Check for Serial commands ──────────────────────────────────
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'c' || cmd == 'C') {
      calMode    = true;
      calStartMs = millis();
      Serial.println("{\"status\": \"Entering calibration mode for 30 seconds.\"}");
    } else if (cmd == 'n' || cmd == 'N') {
      calMode = false;
      Serial.println("{\"status\": \"Returning to normal telemetry mode.\"}");
    }
  }

  // ── Auto-exit calibration mode after 30 seconds ────────────────
  if (calMode && (millis() - calStartMs >= CAL_DURATION_MS)) {
    calMode = false;
    Serial.println("{\"status\": \"Calibration collection complete. Returning to normal mode.\"}");
  }

  // ── Update sensor fusion (must always run) ─────────────────────
  if (mpu.update()) {
    static uint32_t prevOutputMs = millis();
    uint32_t now = millis();

    if (now - prevOutputMs >= OUTPUT_INTERVAL_MS) {
      prevOutputMs = now;

      if (calMode) {
        // ── CALIBRATION MODE: output raw mag values ──────────────
        // These are the library's readings (after its internal cal)
        // but BEFORE our hard-iron offsets, so the calibrate_imu.py
        // script can compute the offsets from scratch.
        Serial.print("{\"mx\":");
        Serial.print(mpu.getMagX(), 4);
        Serial.print(",\"my\":");
        Serial.print(mpu.getMagY(), 4);
        Serial.print(",\"mz\":");
        Serial.print(mpu.getMagZ(), 4);
        Serial.println("}");

      } else {
        // ── NORMAL TELEMETRY MODE ────────────────────────────────

        // --- Compute dt in seconds (microsecond precision) ---
        uint32_t nowMicros = micros();
        float dt = (nowMicros - prevMicros) / 1000000.0;
        prevMicros = nowMicros;

        // Clamp dt to avoid spikes after pauses (e.g. calibration mode)
        if (dt <= 0.0 || dt > 0.5) dt = 0.02;

        // --- Gyroscope Z rate (degrees/sec) ---
        float gz = mpu.getGyroZ();

        // --- Tilt-compensated magnetometer heading ---
        float magHeading = computeMagHeading();

        // --- Complementary filter ---
        if (!yawInitialized) {
          // Seed the filter with the first magnetometer reading
          fusedYaw = magHeading;
          yawInitialized = true;
        } else {
          // Gyro prediction: integrate gyro rate
          float gyroPrediction = normalize360(fusedYaw + gz * dt);

          // Blend: use angleDiff to handle the 0°/360° wraparound
          // diff = shortest path from gyroPrediction to magHeading
          float diff = angleDiff(magHeading, gyroPrediction);

          // fusedYaw = alpha * gyroPrediction + (1-alpha) * magHeading
          // Equivalent to: gyroPrediction + (1-alpha) * diff
          fusedYaw = normalize360(gyroPrediction + (1.0 - ALPHA) * diff);
        }

        // --- Pitch & Roll from Madgwick ---
        float pitch = mpu.getPitch();
        float roll  = mpu.getRoll();

        // --- Output JSON (same format as before) ---
        Serial.print("{\"yaw\":");
        Serial.print(fusedYaw, 2);
        Serial.print(",\"pitch\":");
        Serial.print(pitch, 2);
        Serial.print(",\"roll\":");
        Serial.print(roll, 2);
        Serial.println("}");
      }
    }
  }
}
