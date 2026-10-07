/**
 * magnetometer_calibration.ino
 *
 * Runs the BNO085's built-in dynamic calibration and saves the result to the
 * chip's flash. Written for the earlephilhower Pico core on a Feather RP2350.
 *
 * Calibration persists across power cycles and reflashing, so this only needs
 * to be run once per location (or whenever the magnetic environment changes).
 *
 * Commands (send with a newline):
 *   S   start calibration (enables dynamic cal, begins streaming status)
 *   W   write the current calibration to flash
 *   Z   stop and clear: disables dynamic calibration
 *   ?   show current state
 *
 * Procedure:
 *   1. Take the board somewhere away from monitors, speakers, laptops, and
 *      anything else with a motor or magnet.
 *   2. Send S.
 *   3. Rotate the board slowly through as many orientations as you can.
 *      Hold briefly on each of its six faces, then trace figure-8s.
 *   4. Watch the mag accuracy climb. Once it reads 3 and holds, send W.
 *   5. Check that field magnitude is in the 25-65 uT range. If it is far
 *      above that, something nearby is generating a field and calibration
 *      will not fix it -- move and start over.
 */

#include <Wire.h>
#include <Adafruit_BNO08x.h>

/******************************************************************************
 * Settings
 */

#define SKETCH_VERSION      "calibrate_bno085 1.0"
#define I2C_CLOCK_HZ        400000
#define BNO_RESET_PIN       -1
#define REPORT_INTERVAL_US  20000    // 50 Hz is plenty for calibration
#define STATUS_PERIOD_US    1000000  // print status once per second
#define MIN_ACCURACY_TO_SAVE 2       // refuse to save below this

// Field magnitudes outside this range suggest an environmental problem
// rather than a sensor offset.
#define FIELD_MIN_UT        20.0f
#define FIELD_MAX_UT        80.0f

/******************************************************************************
 * Globals
 */

Adafruit_BNO08x bno(BNO_RESET_PIN);
sh2_SensorValue_t bno_val;

bool     calibrating = false;
uint8_t  magAccuracy = 0;
float    magX = 0, magY = 0, magZ = 0;
float    biasX = 0, biasY = 0, biasZ = 0;
uint64_t lastStatusUs = 0;

char     cmdBuf[16];
uint8_t  cmdLen = 0;

/******************************************************************************
 * BNO085 control
 */

// These call into the SH-2 driver directly; the Adafruit wrapper doesn't
// expose them, but the driver keeps global state after sh2_open().
bool setDynamicCalibration(uint8_t sensors) {
  return sh2_setCalConfig(sensors) == SH2_OK;
}

bool saveCalibration() {
  return sh2_saveDcdNow() == SH2_OK;
}

bool enableReports() {
  bool ok = true;
  ok &= bno.enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, REPORT_INTERVAL_US);
  ok &= bno.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_INTERVAL_US);
  return ok;
}

void disableReports() {
  bno.enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, 0);
  bno.enableReport(SH2_GAME_ROTATION_VECTOR, 0);
}

/******************************************************************************
 * Commands
 */

void startCalibration() {
  if (!setDynamicCalibration(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG)) {
    Serial.println("ERROR: could not enable dynamic calibration");
    return;
  }
  if (!enableReports()) {
    Serial.println("ERROR: could not enable reports");
    return;
  }
  magAccuracy = 0;
  calibrating = true;
  lastStatusUs = time_us_64();
  Serial.println("Calibrating. Rotate the board slowly through all orientations.");
  Serial.println("Send W to save once accuracy reads 3.");
}

void writeCalibration() {
  if (!calibrating) {
    Serial.println("ERROR: not calibrating, send S first");
    return;
  }
  if (magAccuracy < MIN_ACCURACY_TO_SAVE) {
    Serial.printf("REFUSED: mag accuracy is %u, needs to be %d or higher.\n",
                  magAccuracy, MIN_ACCURACY_TO_SAVE);
    Serial.println("Keep moving the board and try again.");
    return;
  }
  if (saveCalibration()) {
    Serial.printf("SAVED to flash at accuracy %u.\n", magAccuracy);
    Serial.println("This persists across power cycles and reflashing.");
  } else {
    Serial.println("ERROR: save failed");
  }
}

void stopCalibration() {
  disableReports();
  if (!setDynamicCalibration(0)) {
    Serial.println("WARNING: could not disable dynamic calibration");
  }
  calibrating = false;
  Serial.println("Stopped. Dynamic calibration disabled.");
}

void showState() {
  Serial.printf("%s\n", SKETCH_VERSION);
  Serial.printf("state: %s\n", calibrating ? "calibrating" : "idle");
  uint8_t sensors = 0;
  if (sh2_getCalConfig(&sensors) == SH2_OK) {
    Serial.printf("dynamic calibration mask: 0x%02X\n", sensors);
  }
  Serial.println("commands: S=start  W=write to flash  Z=stop  ?=state");
}

void handleCommand(char c) {
  switch (c) {
    case 'S': case 's': startCalibration(); break;
    case 'W': case 'w': writeCalibration(); break;
    case 'Z': case 'z': stopCalibration();  break;
    case '?':           showState();        break;
    default:
      Serial.printf("Unknown command '%c'. Try S, W, Z, or ?\n", c);
      break;
  }
}

void readSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        handleCommand(cmdBuf[0]);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

/******************************************************************************
 * Status reporting
 */

const char* accuracyLabel(uint8_t a) {
  switch (a) {
    case 0:  return "unreliable";
    case 1:  return "low";
    case 2:  return "medium";
    case 3:  return "high";
    default: return "?";
  }
}

void printStatus() {
  if (!calibrating) return;
  uint64_t now = time_us_64();
  if (now - lastStatusUs < STATUS_PERIOD_US) return;
  lastStatusUs = now;

  // Field with the bias estimate removed, which is what calibration corrects.
  float cx = magX - biasX, cy = magY - biasY, cz = magZ - biasZ;
  float mag = sqrtf(cx * cx + cy * cy + cz * cz);

  Serial.printf("accuracy=%u (%-10s)  field=%6.1f uT  bias=(%6.1f,%6.1f,%6.1f)",
                magAccuracy, accuracyLabel(magAccuracy), mag,
                biasX, biasY, biasZ);

  if (mag < FIELD_MIN_UT || mag > FIELD_MAX_UT) {
    Serial.print("   <-- field is outside the expected 25-65 uT range");
  } else if (magAccuracy >= MIN_ACCURACY_TO_SAVE) {
    Serial.print("   <-- ready, send W to save");
  }
  Serial.println();
}

/******************************************************************************
 * Main
 */

void serviceBno() {
  if (bno.wasReset()) {
    Serial.println("WARNING: BNO085 reset, re-enabling reports");
    if (calibrating) {
      setDynamicCalibration(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
      enableReports();
    }
  }

  for (int i = 0; i < 8; i++) {
    if (!bno.getSensorEvent(&bno_val)) break;
    if (bno_val.sensorId == SH2_MAGNETIC_FIELD_UNCALIBRATED) {
      auto& m = bno_val.un.magneticFieldUncal;
      magX = m.x;  magY = m.y;  magZ = m.z;
      biasX = m.biasX;  biasY = m.biasY;  biasZ = m.biasZ;
      magAccuracy = bno_val.status & 0x03;
    }
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
  disableReports();

  Serial.printf("%s ready\n", SKETCH_VERSION);
  Serial.println("commands: S=start  W=write to flash  Z=stop  ?=state");
}

void loop() {
  readSerialCommand();
  serviceBno();
  printStatus();
}