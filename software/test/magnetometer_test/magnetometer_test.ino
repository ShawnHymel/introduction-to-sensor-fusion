/**
 * magnetometer_test.ino
 *
 * Streams the BNO085's calibrated magnetometer with its accuracy status and a
 * compass heading, so the readings can be checked against a phone compass.
 *
 * Lay the board FLAT. The heading below uses only the horizontal magnetometer
 * components, so any tilt will skew it -- the vertical component of Earth's
 * field is large compared to the horizontal one.
 *
 * accuracy: 0 = unreliable, 1 = low, 2 = medium, 3 = high
 */

#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define I2C_CLOCK_HZ        400000
#define BNO_RESET_PIN       -1
#define REPORT_INTERVAL_US  20000    // 50 Hz
#define PRINT_PERIOD_US     500000   // twice per second

Adafruit_BNO08x bno(BNO_RESET_PIN);
sh2_SensorValue_t bno_val;

float    magX = 0, magY = 0, magZ = 0;
uint8_t  magAccuracy = 0;
bool     sawMag = false;
uint64_t lastPrintUs = 0;

// Degrees clockwise from magnetic north, assuming the board is flat.
float heading(float mx, float my) {
  float deg = atan2f(my, mx) * 180.0f / PI;
  if (deg < 0) deg += 360.0f;
  return deg;
}

const char* cardinal(float deg) {
  static const char* names[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                                "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  return names[(int)((deg + 11.25f) / 22.5f) & 0x0F];
}

void enableReports() {
  if (!bno.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, REPORT_INTERVAL_US)) {
    Serial.println("WARNING: could not enable calibrated magnetometer");
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);

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
  Wire.setClock(I2C_CLOCK_HZ);

  // Disable calibration
  sh2_setCalConfig(0);

  enableReports();

  Serial.println("Lay the board flat and rotate it slowly.");
  lastPrintUs = time_us_64();
}

void loop() {
  if (bno.wasReset()) {
    Serial.println("WARNING: BNO085 reset, re-enabling");
    sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
    enableReports();
  }

  for (int i = 0; i < 8; i++) {
    if (!bno.getSensorEvent(&bno_val)) break;
    if (bno_val.sensorId == SH2_MAGNETIC_FIELD_CALIBRATED) {
      auto& m = bno_val.un.magneticField;
      magX = m.x;  magY = m.y;  magZ = m.z;
      magAccuracy = bno_val.status & 0x03;
      sawMag = true;
    }
  }

  uint64_t now = time_us_64();
  if (now - lastPrintUs >= PRINT_PERIOD_US) {
    lastPrintUs = now;
    if (!sawMag) {
      Serial.println("(no magnetometer reports yet)");
      return;
    }
    float field = sqrtf(magX * magX + magY * magY + magZ * magZ);
    float h = heading(magX, magY);
    Serial.printf("accuracy=%u  field=%5.1f uT  mag=(%7.2f,%7.2f,%7.2f)  heading=%5.1f %-3s\n",
                  magAccuracy, field, magX, magY, magZ, h, cardinal(h));
  }
}