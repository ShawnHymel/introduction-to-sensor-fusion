/**
 * sensor_log_test.ino
 *
 * Reads and prints out the readings from the Adafruit LSM6DSOX+LIS3MDL board
 * and the Adafruit BNO085 board. Note that this sketch was specifically
 * written for the RP2350, as it relies on some function calls from the 
 * earlephilhower Pico core library.
 */

#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_BNO08x.h>

/******************************************************************************
 * Settings
 */

// Set UART speed
#define UART_CLOCK_HZ 500000

// Use fast I2C clock
#define I2C_CLOCK_HZ 400000

// BNO085 reset pin. Use -1 for unconnected reset pin.
#define BNO_RESET_PIN -1

// BNO085 reporting interval (microseconds)
#define BNO_REPORT_INTERVAL_US 10000  // 100 Hz

/******************************************************************************
 * Globals
 */

// Adafruit_LSM6DSOX sox;
// Adafruit_LIS3MDL lis;
Adafruit_BNO08x bno(BNO_RESET_PIN);
sh2_SensorValue_t bno_val;

/******************************************************************************
 * Functions
 */

// Define the sensor reports to receive from the BNO085
bool bnoSetReports() {
  bool ok = true;
  ok &= bno.enableReport(SH2_ACCELEROMETER, BNO_REPORT_INTERVAL_US);
  // ok &= bno.enableReport(SH2_RAW_GYROSCOPE, BNO_REPORT_INTERVAL_US);
  ok &= bno.enableReport(SH2_GYROSCOPE_UNCALIBRATED, BNO_REPORT_INTERVAL_US);
  // ok &= bno.enableReport(SH2_GYROSCOPE_CALIBRATED, BNO_REPORT_INTERVAL_US);
  ok &= bno.enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, BNO_REPORT_INTERVAL_US);
  // ok &= bno.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, BNO_REPORT_INTERVAL_US);
  ok &= bno.enableReport(SH2_ROTATION_VECTOR, BNO_REPORT_INTERVAL_US);
  // ok &= bno.enableReport(SH2_GAME_ROTATION_VECTOR, BNO_REPORT_INTERVAL_US);

  return ok;
}

/**
 * TEST
 */

 #define PRINT_ROWS 0   // set to 0 to measure throughput with no serial output

enum BnoStream { S_ACC, S_GYRO_RAW, S_GYRO_UC, S_GYRO_CAL, S_MAG_UC, S_MAG_CAL, S_RV, S_GRV, S_COUNT };
const char* const STREAM_NAME[S_COUNT] =
    {"ACC", "GYRO_RAW", "GYRO_UC", "GYRO_CAL", "MAG_UC", "MAG_CAL", "RV", "GRV"};
uint32_t rateCount[S_COUNT] = {0};
uint32_t loopCount = 0;
uint64_t lastRatePrintUs = 0;

void printRates() {
  uint64_t now = time_us_64();
  if (now - lastRatePrintUs < 1000000ULL) return;
  float dt = (now - lastRatePrintUs) * 1e-6f;
  lastRatePrintUs = now;
  uint32_t total = 0;
  Serial.printf("# rates_hz loop=%.0f", loopCount / dt);
  for (int i = 0; i < S_COUNT; i++) {
    Serial.printf(" %s=%.0f", STREAM_NAME[i], rateCount[i] / dt);
    total += rateCount[i];
    rateCount[i] = 0;
  }
  Serial.printf(" total=%.0f\n", total / dt);
  loopCount = 0;
}

/******************************************************************************
 * Main
 */

void setup() {
  // Initialize serial
  Serial.begin(UART_CLOCK_HZ);

  // // Initialize LSM6DSOX
  // if (!sox.begin_I2C()) {
  //   while (1) {
  //     Serial.println("ERROR: could not initialize LSM6DSOX");
  //     delay(1000);
  //   }
  // }

  // // Configure LSM6DSOX
  // sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  // sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  // sox.setAccelDataRate(LSM6DS_RATE_208_HZ);
  // sox.setGyroDataRate(LSM6DS_RATE_208_HZ);

  // // Initialize LIS3MDL
  // if (!lis.begin_I2C()) {
  //   while (1) {
  //     Serial.println("ERROR: could not initialize LIS3MDL");
  //     delay(1000);
  //   }
  // }

  // // Configure LIS3MDL
  // lis.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  // lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  // lis.setDataRate(LIS3MDL_DATARATE_155_HZ);
  // lis.setRange(LIS3MDL_RANGE_4_GAUSS);

  // Let lines settle before trying to initialize sensor
  delay(1000);

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

  // Configure BNO085
  if (!bnoSetReports()) {
    while (1) {
      Serial.println("ERROR: could not set reports for BNO085");
      delay(1000);
    }
  }

  // Set faster I2C clock
  Wire.setClock(I2C_CLOCK_HZ);

  // TEST
  lastRatePrintUs = time_us_64();
}

void loop() {
  // sensors_event_t lsm_accel;
  // sensors_event_t lsm_gyro;
  // sensors_event_t lsm_temp;
  // sensors_event_t lis_mag;
  uint64_t timestamp;
  uint64_t bno_timestamp;
  uint8_t bno_status;

  // // Print if data is ready from LSM6DSOX
  // if (sox.accelerationAvailable()) {
  //   timestamp = time_us_64();
  //   sox.getEvent(&lsm_accel, &lsm_gyro, &lsm_temp);
  //   Serial.printf("%llu,LSM,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f\n",
  //                 (unsigned long long)timestamp,
  //                 lsm_accel.acceleration.x, lsm_accel.acceleration.y, lsm_accel.acceleration.z,
  //                 lsm_gyro.gyro.x, lsm_gyro.gyro.y, lsm_gyro.gyro.z);
  // }

  // // Print if data is ready from LIS3MDL
  // if (lis.magneticFieldAvailable()) {
  //   timestamp = time_us_64();
  //   lis.getEvent(&lis_mag);
  //   Serial.printf("%llu,LIS,%.3f,%.3f,%.3f\n",
  //                 (unsigned long long)timestamp,
  //                 lis_mag.magnetic.x, lis_mag.magnetic.y, lis_mag.magnetic.z);
  // }

  // Reset reporting if BNO085 was reset
  if (bno.wasReset()) {
    if (!bnoSetReports()) {
      Serial.print("# ERROR: Could not set reports for BNO085");
    }
  }

  // Read and print estimated orientation from BNO085, depending on the sensor type
  if (bno.getSensorEvent(&bno_val)) {
    // Get RP2350 timestamp and timestamp from BNO event
    timestamp = time_us_64();
    bno_timestamp = bno_val.timestamp;

    // Get the BNO085 status
    bno_status = bno_val.status & 0x03;

    // Log BNO085 report, based on the type of report received
    switch (bno_val.sensorId) {
      case SH2_ACCELEROMETER: {
        auto& a = bno_val.un.accelerometer;
#if PRINT_ROWS
        Serial.printf("%llu,BNO_ACC,%llu,%u,%.5f,%.5f,%.5f\n",
                      (unsigned long long)timestamp, bno_timestamp, bno_status,
                      a.x, a.y, a.z);
#endif
        rateCount[S_ACC]++;
        break;
      }

      case SH2_RAW_GYROSCOPE: {
        auto& g = bno_val.un.rawGyroscope;
#if PRINT_ROWS        
        Serial.printf("%llu,BNO_GYRO_RAW,%llu,%u,%d,%d,%d,%d,%lu\n",
                      (unsigned long long)timestamp, bno_timestamp, bno_status,
                      g.x, g.y, g.z, g.temperature, (unsigned long)g.timestamp);
#endif
        rateCount[S_GYRO_RAW]++;
        break;
      }

      case SH2_GYROSCOPE_UNCALIBRATED: {
        auto& g = bno_val.un.gyroscopeUncal;
#if PRINT_ROWS
        Serial.printf("%llu,BNO_GYRO_UC,%llu,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                      (unsigned long long)timestamp, bno_timestamp, bno_status,
                      g.x, g.y, g.z, g.biasX, g.biasY, g.biasZ);
#endif  
        rateCount[S_GYRO_UC]++;      
        break;
      }

      case SH2_GYROSCOPE_CALIBRATED: {
        auto& g = bno_val.un.gyroscope;
#if PRINT_ROWS
        Serial.printf("%llu,BNO_GYRO_UC,%llu,%u,%.6f,%.6f,%.6f\n",
                      (unsigned long long)timestamp, bno_timestamp, bno_status,
                      g.x, g.y, g.z);
#endif  
        rateCount[S_GYRO_CAL]++;      
        break;
      }

      case SH2_MAGNETIC_FIELD_UNCALIBRATED: {
        auto& m = bno_val.un.magneticFieldUncal;
#if PRINT_ROWS
        Serial.printf("%llu,BNO_MAG_UC,%llu,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                      (unsigned long long)timestamp, bno_timestamp, bno_status,
                      m.x, m.y, m.z, m.biasX, m.biasY, m.biasZ);
#endif
        rateCount[S_MAG_UC]++;
        break;
      }

      case SH2_MAGNETIC_FIELD_CALIBRATED: {
        auto& m = bno_val.un.magneticField;
#if PRINT_ROWS
        Serial.printf("%llu,BNO_MAG_UC,%llu,%u,%.3f,%.3f,%.3f\n",
                      (unsigned long long)timestamp, bno_timestamp, bno_status,
                      m.x, m.y, m.z);
#endif
        rateCount[S_MAG_CAL]++;
        break;
      }

      case SH2_ROTATION_VECTOR: {
        auto& q = bno_val.un.rotationVector;
#if PRINT_ROWS
        Serial.printf("%llu,BNO_RV,%llu,%u,%.6f,%.6f,%.6f,%.6f,%.4f\n",
                      (unsigned long long)timestamp, bno_timestamp, bno_status,
                      q.real, q.i, q.j, q.k, q.accuracy);
#endif
        rateCount[S_RV]++;
        break;
      }

      case SH2_GAME_ROTATION_VECTOR: {
        auto& q = bno_val.un.gameRotationVector;
#if PRINT_ROWS
        Serial.printf("%llu,BNO_GRV,%llu,%u,%.6f,%.6f,%.6f,%.6f\n",
                      (unsigned long long)timestamp, bno_timestamp, bno_status,
                      q.real, q.i, q.j, q.k);
#endif
        rateCount[S_GRV]++;
        break;
      }

      default: {
        Serial.printf("# WARN: unexpected BNO sensorId 0x%02X\n", bno_val.sensorId);
        break;
      }
    }
  }

  // TEST
  loopCount++;
  printRates();
}
