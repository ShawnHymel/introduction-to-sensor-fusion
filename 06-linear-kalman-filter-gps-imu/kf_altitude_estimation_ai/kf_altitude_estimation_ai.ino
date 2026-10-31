/**
 * kf_altitude_estimation.ino
 *
 * Real-time linear Kalman filter for altitude (p) and vertical velocity (v),
 * fusing the BNO085 accelerometer (z-axis) and the BMP581 barometer. Same
 * filter as the kf_altitude_estimation notebook:
 *   - Predict on every accelerometer sample (input u = vertical acceleration)
 *   - Correct on every barometer sample (measurement z = relative altitude)
 *
 * Assumption: the board stays flat and face up, so sensor z is vertical.
 *
 * At startup, hold the board still for REF_TIME_US. The sketch averages the
 * readings to get the references (zero points): the at-rest accelerometer
 * offset (gravity + bias) and the starting barometer altitude.
 *
 * Open the Serial Plotter to see altitude and velocity.
 *
 * Written for the earlephilhower Pico core (required for time_us_64()).
 * Libraries: Adafruit BNO08x, Adafruit BMP5xx
 */

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP5xx.h>
#include <math.h>

/******************************************************************************
 * Settings
 */

#define UART_CLOCK_HZ           115200
#define I2C_CLOCK_HZ            400000
#define BNO_RESET_PIN           -1        // -1 for "no reset pin"
#define BNO_REPORT_INTERVAL_US  10000     // 100 Hz (BNO085 runs ~125 Hz)
#define BMP_POLL_US             1000      // How often to check for baro data
#define REF_TIME_US             2000000   // Hold still this long at startup
#define SEA_LEVEL_HPA           1013.25f  // Standard atmosphere

// Kalman filter tuning (from the notebooks)
#define SIGMA_A                 0.5f      // Acceleration (input) noise, m/s^2
#define SIGMA_Z                 0.0255f   // Barometer (measurement) noise, m
#define SIGMA_V0                0.05f     // Initial velocity uncertainty, m/s

/******************************************************************************
 * Globals
 */

Adafruit_BNO08x bno(BNO_RESET_PIN);
Adafruit_BMP5xx bmp;

// Kalman filter state: x = [p, v] and its error covariance P (2x2)
float x[2] = {0.0f, 0.0f};
float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
float u = 0.0f;               // Held acceleration (input for the next step)
const float R = SIGMA_Z * SIGMA_Z;  // Measurement noise covariance (1x1)
int64_t t_prev = 0;           // Timestamp of the previous predict step (us)
bool have_t_prev = false;     // Records if we have a previous timestep

// Startup references (zero points)
bool referencing = true;      // True while measuring the references
uint64_t ref_start_us = 0;    // When we started measuring
float acc_sum = 0.0f;         // Sum of az readings during startup
uint32_t acc_count = 0;
float baro_sum = 0.0f;        // Sum of baro altitudes during startup
uint32_t baro_count = 0;
float g_ref = 0.0f;           // At-rest accelerometer offset (gravity + bias)
float baro_ref = 0.0f;        // Starting altitude

// Most recent relative barometer altitude (for plotting)
float baro_rel = 0.0f;
uint64_t next_bmp_poll_us = 0;

/******************************************************************************
 * Kalman filter
 */

// Set the initial state and covariance: at rest, at p = 0
void kfInit() {
  x[0] = 0.0f;
  x[1] = 0.0f;
  P[0][0] = SIGMA_Z * SIGMA_Z;  // Initial position uncertainty
  P[0][1] = 0.0f;
  P[1][0] = 0.0f;
  P[1][1] = SIGMA_V0 * SIGMA_V0;  // Initial velocity uncertainty
  u = 0.0f;
  have_t_prev = false;
}

// Predict the state at time t (us), using the acceleration held from the
// previous step. Then hold the new acceleration for the next step.
void predict(int64_t t, float a) {
  // Skip the prediction on the first timestep (no dt)
  if (have_t_prev) {
    float dt = (t - t_prev) * 1e-6f;

    // System matrices: F = [[1, dt], [0, 1]], B = [dt^2/2, dt]
    float b0 = 0.5f * dt * dt;
    float b1 = dt;

    // Process noise: Q = B B^T sigma_a^2
    float sa2 = SIGMA_A * SIGMA_A;
    float q00 = b0 * b0 * sa2;
    float q01 = b0 * b1 * sa2;
    float q11 = b1 * b1 * sa2;

    // 1a) Predict state: x = F x + B u
    x[0] = x[0] + dt * x[1] + b0 * u;
    x[1] = x[1] + b1 * u;

    // 1b) Predict covariance: P = F P F^T + Q (written out for 2x2)
    float p00 = P[0][0] + dt * (P[0][1] + P[1][0]) + dt * dt * P[1][1] + q00;
    float p01 = P[0][1] + dt * P[1][1] + q01;
    float p10 = P[1][0] + dt * P[1][1] + q01;
    float p11 = P[1][1] + q11;
    P[0][0] = p00;
    P[0][1] = p01;
    P[1][0] = p10;
    P[1][1] = p11;
  }

  // Hold the acceleration as the input for the next timestep
  u = a;

  // Save the previous timestamp
  t_prev = t;
  have_t_prev = true;
}

// Correct the prior estimate using the barometer's altitude measurement (m).
// H = [1, 0] and D = 0, so the matrix math reduces to a few scalar lines.
void correct(float z) {
  // 1c) Predict measurement: z_hat = H x + D u = p
  float z_hat = x[0];

  // Innovation (how far off the prediction was)
  float y = z - z_hat;

  // Innovation covariance: S = H P H^T + R = P[0][0] + R
  float S = P[0][0] + R;

  // 2a) Kalman gain: K = P H^T S^-1 = [P[0][0], P[1][0]] / S
  float k0 = P[0][0] / S;
  float k1 = P[1][0] / S;

  // 2b) Update state: x = x + K y
  x[0] += k0 * y;
  x[1] += k1 * y;

  // 2c) Update covariance: P = (I - K H) P (written out for 2x2)
  float p00 = (1.0f - k0) * P[0][0];
  float p01 = (1.0f - k0) * P[0][1];
  float p10 = P[1][0] - k1 * P[0][0];
  float p11 = P[1][1] - k1 * P[0][1];

  // Keep P symmetric (float rounding can make p01 and p10 drift apart)
  P[0][0] = p00;
  P[0][1] = 0.5f * (p01 + p10);
  P[1][0] = P[0][1];
  P[1][1] = p11;
}

/******************************************************************************
 * Sensor functions
 */

// Enable the accelerometer report
bool bnoEnableReports() {
  return bno.enableReport(SH2_ACCELEROMETER, BNO_REPORT_INTERVAL_US);
}

// Configure the BMP581: x16 pressure oversampling, no IIR filter, 50 Hz
bool bmpInit() {
  if (!bmp.begin(0x47, &Wire) && !bmp.begin(0x46, &Wire)) {
    return false;
  }
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_1X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_BYPASS);
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
  bmp.enablePressure(true);
  bmp.configureInterrupt(BMP5XX_INTERRUPT_LATCHED,
                         BMP5XX_INTERRUPT_ACTIVE_HIGH,
                         BMP5XX_INTERRUPT_PUSH_PULL,
                         BMP5XX_INTERRUPT_DATA_READY, true);
  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);
  return true;
}

// Read a new barometer altitude (m) if one is ready. Returns false if not.
bool readBaroAltitude(float &alt_m) {
  // Only poll every BMP_POLL_US to leave the I2C bus free for the BNO085
  uint64_t now = time_us_64();
  if (now < next_bmp_poll_us) {
    return false;
  }
  next_bmp_poll_us = now + BMP_POLL_US;

  // Check for and read a new sample (library reports pressure in hPa)
  if (!bmp.dataReady() || !bmp.performReading()) {
    return false;
  }
  alt_m = 44330.0f * (1.0f - powf(bmp.pressure / SEA_LEVEL_HPA, 0.190295f));
  return true;
}

/******************************************************************************
 * Main
 */

void setup() {
  // Initialize serial
  Serial.begin(UART_CLOCK_HZ);

  // Wait for serial to connect
  while (!Serial) {
    delay(10);
  }

  // Initialize I2C, let lines settle
  Wire.begin();
  delay(100);

  // Initialize BNO085. Keep trying if it fails.
  if (!bno.begin_I2C()) {
    while (1) {
      Serial.println("ERROR: could not initialize BNO085");
      if (bno.begin_I2C()) {
        break;
      }
      delay(1000);
    }
  }

  // Initialize BMP581. Keep trying if it fails.
  if (!bmpInit()) {
    while (1) {
      Serial.println("ERROR: could not initialize BMP581");
      if (bmpInit()) {
        break;
      }
      delay(1000);
    }
  }

  // Set I2C speed (after both begin() calls)
  Wire.setClock(I2C_CLOCK_HZ);

  // Disable dynamic calibration (keeps the accelerometer bias steady)
  sh2_setCalConfig(0);

  // Enable accelerometer reports from BNO085
  if (!bnoEnableReports()) {
    while (1) {
      Serial.println("ERROR: could not enable BNO085 reports");
      delay(1000);
    }
  }

  // Start measuring the references
  Serial.println("Hold still: measuring references...");
  ref_start_us = time_us_64();
}

void loop() {
  sh2_SensorValue_t bno_val;
  float alt_m;

  // If BNO085 was reset, re-enable reports
  if (bno.wasReset()) {
    bnoEnableReports();
  }

  // Startup: average both sensors to get the references (zero points)
  if (referencing) {
    if (bno.getSensorEvent(&bno_val) &&
        bno_val.sensorId == SH2_ACCELEROMETER) {
      acc_sum += bno_val.un.accelerometer.z;
      acc_count++;
    }
    if (readBaroAltitude(alt_m)) {
      baro_sum += alt_m;
      baro_count++;
    }

    // Once enough time has passed, compute the references and start the KF
    if (time_us_64() - ref_start_us >= REF_TIME_US &&
        acc_count > 0 && baro_count > 0) {
      g_ref = acc_sum / acc_count;
      baro_ref = baro_sum / baro_count;
      kfInit();
      referencing = false;
      Serial.printf("References: g_ref = %.4f m/s^2, baro_ref = %.2f m\n",
                    g_ref, baro_ref);
    }
    return;
  }

  // Barometer: correct with each new altitude sample (relative to start)
  if (readBaroAltitude(alt_m)) {
    baro_rel = alt_m - baro_ref;
    correct(baro_rel);
  }

  // Accelerometer: get a report, return if none are available
  if (!bno.getSensorEvent(&bno_val)) {
    return;
  }
  if (bno_val.sensorId != SH2_ACCELEROMETER) {
    return;
  }

  // Get the timestamp for reading the report. Correct with the BNO timestamp
  // (often negative) to get the actual timestamp when the sensor was read.
  int64_t timestamp_us = (int64_t)time_us_64();
  timestamp_us += (int32_t)bno_val.timestamp;

  // Predict with the vertical acceleration (gravity + bias removed)
  float a = bno_val.un.accelerometer.z - g_ref;
  predict(timestamp_us, a);

  // Plot the estimated altitude and velocity, plus the raw barometer altitude
  Serial.printf("Baro(m):%.3f,Altitude(m):%.3f,Velocity(m/s):%.3f\n",
                baro_rel, x[0], x[1]);
}
