/**
 * Simply read and output quaternions from the BNO085. Intended for the
 * Adafruit 3D Model Viewer:
 * https://adafruit.github.io/Adafruit_WebSerial_3DModelViewer/
 *
 * Note that we use game rotational vector (GRV) for this, so the heading is
 * not absolute.
 */

#include <Adafruit_BNO08x.h>

/******************************************************************************
 * Settings
 */

#define UART_CLOCK_HZ       115200
#define I2C_CLOCK_HZ        400000
#define BNO_RESET_PIN       -1
#define REPORT_INTERVAL_US  10000    // 100 Hz

/******************************************************************************
 * Globals
 */

Adafruit_BNO08x bno(BNO_RESET_PIN);

/******************************************************************************
 * Functions
 */

// Enable GRV reports from the BNO085
bool bnoEnableReports() {
  bool ok = bno.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_INTERVAL_US);

  return ok;
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

  // Enable GRV reports from BNO085
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
  // Note: unused. Uncomment this section for debugging purposes
  // int64_t timestamp_us = (int64_t)time_us_64();
  // timestamp_us += (int32_t)bno_val.timestamp;
  // Serial.printf("%lli - ", timestamp_us);

  // Print quaternions from GRV
  if (bno_val.sensorId == SH2_GAME_ROTATION_VECTOR) {
    auto &q = bno_val.un.gameRotationVector;
    Serial.printf("Quaternion: %.6f, %.6f, %.6f, %.6f\n", q.real, q.i, q.j, q.k);
  }
}