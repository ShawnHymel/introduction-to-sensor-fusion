/**
 * Solution to the complementary filter challenge
 */

#include <Adafruit_BNO08x.h>

/******************************************************************************
 * Settings
 */

#define UART_CLOCK_HZ           115200
#define I2C_CLOCK_HZ            400000
#define BNO_RESET_PIN           -1        // -1 for "no reset pin"
#define BNO_REPORT_INTERVAL_US  10000     // 100 Hz
#define ALPHA                   0.99f     // How much to blend the gyro

/******************************************************************************
 * Globals
 */

Adafruit_BNO08x bno(BNO_RESET_PIN);
float pitch_deg = 0.0f;     // Current pitch estimate (degrees)
uint64_t t_prev = 0;        // Previous gyro integration time
bool have_t_prev = false;   // Records if we have a previous timestep
bool initialized = false;   // Records if we have an initial acc reading

/******************************************************************************
 * Functions
 */

// Enable accelerometer and gyroscope reports
bool bnoEnableReports() {
  bool ok = true;
  ok &= bno.enableReport(SH2_ACCELEROMETER, BNO_REPORT_INTERVAL_US);
  ok &= bno.enableReport(SH2_GYROSCOPE_UNCALIBRATED, BNO_REPORT_INTERVAL_US);

  return ok;
}

// TODO: write the predict() function
void predict(int64_t t, float gyro_rate) {
  // If previous timestamp exists, integrate the gyro reading
  if (have_t_prev) {
    float dt = (t - t_prev) * 1e-6f;
    pitch_deg += gyro_rate * dt;
  }

  // Save the previous timestamp
  t_prev = t;
  have_t_prev = true;
}

// TODO: write the correct() function
void correct(float measured_pitch_deg) {
  if (!initialized) {
    // The first accelerometer reading should set the initial pitch
    pitch_deg = measured_pitch_deg;
    initialized = true;
  } else {
    // Complementary filter: blend acc and integrated gyro readings
    pitch_deg = ALPHA * pitch_deg + (1.0 - ALPHA) * measured_pitch_deg;
  }
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

  // Set I2C speed
  Wire.setClock(I2C_CLOCK_HZ);

  // Disable dynamic calibration
  sh2_setCalConfig(0);

  // Enable accelerometer and gyroscope reports from BNO085
  if (!bnoEnableReports()) {
    while (1) {
      Serial.println("ERROR: could not enable BNO085 reports");
      delay(1000);
    }
  }
}

void loop() {
  sh2_SensorValue_t bno_val;

  // If BNO085 was reset, re-enable reports
  if (bno.wasReset()) {
    bnoEnableReports();
  }

  // Get sensor reading, return if no reports are available
  if (!bno.getSensorEvent(&bno_val)) {
    return;
  }

  // Get the timestamp for reading the report. Correct with the BNO timestamp
  // (often negative) to get the actual timestamp when the sensor was read.
  int64_t timestamp_us = (int64_t)time_us_64();
  timestamp_us += (int32_t)bno_val.timestamp;

  // Predict or correct, depending on the type of event
  switch (bno_val.sensorId) {
    case SH2_GYROSCOPE_UNCALIBRATED: {
      // Get y-axis reading from gyroscope
      auto &g = bno_val.un.gyroscopeUncal;
      float gy = degrees(g.y);

      // TODO: call predict or correct with the gyro reading
      predict(timestamp_us, gy);

      break;
    }

    // Calculate pitch from x and z axes from the accelerometer
    case SH2_ACCELEROMETER: {
      // Get x- and z-axis readings from the accelerometer
      auto &a = bno_val.un.accelerometer;
      float ax = a.x;
      float az = a.z;

      // Calculate the pitch (assume no roll, i.e. no y-axis component)
      float pitch_acc_deg = degrees(atan2f(-ax, az));

      // TODO: call predict or correct with the accelerometer pitch calculation
      correct(pitch_acc_deg);
    }

    // All other reports: do nothing
    default:
      break;
  }

  // Plot the estimated pitch
  Serial.printf("Pitch(deg):%2f\n", pitch_deg);
}