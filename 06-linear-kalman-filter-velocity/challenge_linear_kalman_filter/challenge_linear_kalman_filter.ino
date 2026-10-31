/**
 * Linear Kalman filter challenge
 *
 * Look for the TODO comments and implement your code where you see YOUR CODE
 * HERE
 *
 * On reset, keep the board flat and still for 2 seconds to get the reference
 * points for the barometer (current starting altitude) and accelerometer
 * (gravity), which are subtracted out of the later readings for KF processing.
 */

#include <math.h>

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP5xx.h>

/******************************************************************************
 * Settings
 */

#define UART_CLOCK_HZ           115200
#define I2C_CLOCK_HZ            400000
#define BNO_RESET_PIN           -1        // -1 for "no reset pin"
#define BNO_REPORT_INTERVAL_US  10000     // Request 100 Hz (BNO gives ~125 Hz)
#define BMP_POLL_US             1000      // How often to check for baro data
#define REF_TIME_US             2000000   // Hold still this long at startup
#define SEA_LEVEL_HPA           1013.25f  // Standard atmosphere

// Kalman filter tuning (from the Jupyter Notebook)
#define SIGMA_A                 0.5f      // Acceleration (input) noise (m/s^2)
#define SIGMA_Z                 0.0255f   // Barometer (measurement) noise (m)
#define SIGMA_V0                0.05f     // Initial velocity uncertainty (m/s)

/******************************************************************************
 * Globals
 */

Adafruit_BNO08x bno(BNO_RESET_PIN);
Adafruit_BMP5xx bmp;

// Kalman filter state: x = [p, v] and the error covariance P (2x2)
float x[2] = {0.0f, 0.0f};
float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
float u = 0.0f;                     // Held acceleration (input for next step)
const float R = SIGMA_Z * SIGMA_Z;  // Measurement noise covariance (1x1)
int64_t t_prev = 0;                 // Timestamp of previous predict step (us)
bool have_t_prev = false;           // Records if we have a previous timestep

// Startup references (zero points)
float g_ref = 0.0f;                 // Accelerometer offset (gravity + bias)
float alt_ref = 0.0f;               // Starting altitude (m)

// Barometer readings
float alt_rel = 0.0f;               // Altitude (m) relative to start position
uint64_t prev_bmp_time_us = 0;      // Previous BMP581 poll timestamp (us)

/******************************************************************************
 * Sensor functions
 */

// Enable accelerometer reports on the BNO085
bool bnoEnableReports() {
  bool ok = true;
  ok &= bno.enableReport(SH2_ACCELEROMETER, BNO_REPORT_INTERVAL_US);

  return ok;
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
bool readBmpAltitude(float &alt_m) {
  // Only poll every BMP_POLL_US to leave the I2C bus free for the BNO085
  uint64_t now = time_us_64();
  if (now - prev_bmp_time_us < BMP_POLL_US) {
    return false;
  }
  prev_bmp_time_us = now;

  // Check for and read a new sample (library reports pressure in hPa)
  if (!bmp.dataReady() || !bmp.performReading()) {
    return false;
  }

  // Convert barometric pressure to altitude
  alt_m = 44330.0f * (1.0f - powf(bmp.pressure / SEA_LEVEL_HPA, 0.190295f));

  return true;
}

// Average both sensors to get the references (zero points): accelerometer
// offset (gravity + bias) and the starting altitude. Note: this is blocking.
void measureReferences() {
  sh2_SensorValue_t bno_val;
  float alt_m;
  double acc_sum = 0.0;
  double alt_sum = 0.0;
  uint32_t acc_count = 0;
  uint32_t alt_count = 0;

  // Collect readings for REF_TIME_US
  uint64_t start_us = time_us_64();
  Serial.println("Hold still, measuring references...");
  while (time_us_64() - start_us < REF_TIME_US ||
         acc_count == 0 || alt_count == 0) {

    // If BNO085 was reset, re-enable reports
    if (bno.wasReset()) {
      bnoEnableReports();
    }

    // Accelerometer
    if (bno.getSensorEvent(&bno_val) &&
        bno_val.sensorId == SH2_ACCELEROMETER) {
      acc_sum += bno_val.un.accelerometer.z;
      acc_count++;
    }

    // Barometer
    if (readBmpAltitude(alt_m)) {
      alt_sum += alt_m;
      alt_count++;
    }
  }

  // References are the averages
  g_ref = acc_sum / acc_count;
  alt_ref = alt_sum / alt_count;

  Serial.printf("References: g_ref = %.4f m/s^2, alt_ref = %.2f m\n",
                g_ref, alt_ref);
}

/******************************************************************************
 * Kalman filter functions
 */

// Predict the state at time t (us) using the accel value from previous step
void predict(int64_t t, float a) {
  // Store u to use as our previous input/control u_{k-1}
  float u_prev = u;

  // Acceleration becomes our new input/control u_{k}
  u = a;

  // Skip the prediction on the first timestep (no dt)
  if (have_t_prev) {
    // Calculate the difference between this and the previous timestep
    float dt = (t - t_prev) * 1e-6f;

    // Store x to use as our previous estimate x_{k-1}
    float x_prev[2];
    for (int i = 0; i < 2; i++) {
      x_prev[i] = x[i];
    }

    // Store P to use as our previous error covariance P_{k-1}
    float P_prev[2][2];
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        P_prev[i][j] = P[i][j];
      }
    }

    // TODO: Compute the state transition matrix: F = [[1, dt], [0, 1]]
    float F[2][2] = {
      // YOUR CODE HERE
    };

    // TODO: Compute the input/control matrix: B = [dt^2/2, dt]^T
    float B[2] = {
      // YOUR CODE HERE
    };

    // TODO: Compute the process noise covariance: Q = B B^T sigma_a^2
    float Q[2][2];
    // YOUR CODE HERE

    // TODO: 1a) Predict state: x_{k} = F x_{k-1} + B u_{k-1}
    // YOUR CODE HERE

    // TODO: 1b) Predict covariance: P_{k}^{-} = F P_{k-1} F^T + Q
    // Hint: do this in 2 stages
    // First stage: FP = F * P
    // YOUR CODE HERE

    // Second stage: P = FP * F^T + Q
    // YOUR CODE HERE
  }

  // Save the previous timestamp
  t_prev = t;
  have_t_prev = true;
}

// Correct the predicted estimate using the barometer's altitude measurement
void correct(float z) {
  // Store x to use as our prior estimate x_{k}^{-}
  float x_prior[2];
  for (int i = 0; i < 2; i++) {
    x_prior[i] = x[i];
  }

  // Store P to use as our prior error covariance P_{k}^{-}
  float P_prior[2][2];
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      P_prior[i][j] = P[i][j];
    }
  }

  // Compute the measurement matrix: H = [1, 0]
  float H[2] = {
    1.0f, 0.0f
  };

  // Compute the feedthrough matrix: D = 0
  float D = 0.0f;

  // Identity matrix (2x2)
  float I[2][2] = {
    {1.0f, 0.0f},
    {0.0f, 1.0f}
  };

  // TODO: 1c) Predict measurement: z_hat_{k} = H x_{k}^{-} + D u_{k}
  float z_hat;
  // YOUR CODE HERE

  // TODO: Compute the innovation: y_{k} = z_{k} - z_hat_{k}
  float y;
  // YOUR CODE HERE

  // TODO: Compute the innovation covariance: S_{k} = H P_{k}^{-} H^T + R
  // Hint: do this in 2 stages
  // First stage: PHt = P_prior * H^T (2x1)
  float PHt[2];
  // YOUR CODE HERE

  // Second stage: S = H * PHt + R (1x1)
  float S;
  // YOUR CODE HERE

  // TODO: 2a) Compute the Kalman gain: K_{k} = P_{k}^{-} H^T S_{k}^{-1}
  // Note: S is 1x1, so its inverse is just 1/S
  float K[2];
  // YOUR CODE HERE

  // TODO: 2b) Update state: x_{k} = x_{k}^{-} + K_{k} y_{k}
  // YOUR CODE HERE

  // TODO: 2c) Update covariance: P_{k} = (I - K_{k} H) P_{k}^{-}
  // Hint: do this in 2 stages
  // First stage: IKH = I - K * H (2x2; K * H is an outer product)
  float IKH[2][2];
  // YOUR CODE HERE

  // Second stage: P = IKH * P_prior
  // YOUR CODE HERE

  // P is a covariance matrix, so it must be symmetric (by definition). Float
  // rounding can make the off-diagonal terms drift apart, so we take their
  // average and re-assign that one average to both terms.
  float p_off = 0.5f * (P[0][1] + P[1][0]);
  P[0][1] = p_off;
  P[1][0] = p_off;
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

  // Disable dynamic calibration so the accelerometer bias can't shift
  // mid-recording (we remove the at-rest mean manually in this sketch)
  sh2_setCalConfig(0);

  // Enable accelerometer reports from BNO085
  if (!bnoEnableReports()) {
    while (1) {
      Serial.println("ERROR: could not enable BNO085 reports");
      delay(1000);
    }
  }

  // Initialize error covariance P with initial uncertainties on the diagonal
  float P[2][2] = {
    {SIGMA_Z * SIGMA_Z, 0.0f},
    {0.0f, SIGMA_V0 * SIGMA_V0}
  };

  // Wait while the references (zero-points) are measured
  measureReferences();
}

void loop() {
  sh2_SensorValue_t bno_val;
  float alt_m;

  // If BNO085 was reset, re-enable reports
  if (bno.wasReset()) {
    bnoEnableReports();
  }

  // If there is a new accelerometer sample: predict the new state estimate
  if (bno.getSensorEvent(&bno_val) &&
      bno_val.sensorId == SH2_ACCELEROMETER) {
    // Get the timestamp for reading the report. Correct with the BNO
    // timestamp (often negative) to get when the sensor was actually read.
    int64_t timestamp_us = (int64_t)time_us_64();
    timestamp_us += (int32_t)bno_val.timestamp;

    // Remove gravity and bias
    float a = bno_val.un.accelerometer.z - g_ref;

    // Predict the new state estimate
    predict(timestamp_us, a);

    // Plot the raw barometer, estimated altitude, and estimated velocity
    Serial.printf("baro_alt(m):%.3f,est_alt(m):%.3f,est_vel(m/s):%.3f\n",
                  alt_rel, x[0], x[1]);
  }

  // If there is a new barometer sample: correct the estimate with the altitude
  if (readBmpAltitude(alt_m)) {
    // Remove the reference altitude to get the relative altitude
    alt_rel = alt_m - alt_ref;

    // Correct the state estimate using the altitude reading
    correct(alt_rel);
  }
}
