/**
 * sensor_logger.ino
 *
 * Command-driven logger for the Adafruit BNO085 on an Adafruit Feather RP2350.
 * Written for the earlephilhower Pico core (uses time_us_64()).
 *
 * Protocol (send lines terminated with newline):
 *   S <G|R> <seconds> <name>   start recording; G = game rotation vector,
 *                              R = rotation vector; seconds = 0 runs until
 *                              stopped; name is a label with no spaces
 *   X                          stop the current recording
 *   ?                          identify (version, state, options)
 *
 * Output:
 *   "# ..." lines are metadata; everything else is data.
 *   Each recording is bracketed by a "# BEGIN" header block and a
 *   "# END" summary line.
 *
 * Note: GRV and RV cannot both run while a gyro report is enabled -- the
 * BNO085 silently drops RV. Each recording therefore carries exactly one
 * orientation reference, selected by the start command.
 */

#include <Wire.h>
#include <Adafruit_BNO08x.h>

/******************************************************************************
 * Settings
 */

#define UART_CLOCK_HZ       500000
#define SKETCH_VERSION      "sensor_logger 1.0"
#define I2C_CLOCK_HZ        400000
#define BNO_RESET_PIN       -1
#define REPORT_INTERVAL_US  10000    // 100 Hz
#define SETTLE_US           200000   // discard this much data after enabling
#define MAX_CMD_LEN         80
#define MAX_NAME_LEN        32
#define DRAIN_PER_PASS      8

/******************************************************************************
 * Globals
 */

Adafruit_BNO08x bno(BNO_RESET_PIN);
sh2_SensorValue_t bno_val;

enum BnoStream { S_ACC, S_GYRO, S_MAG, S_REF, S_COUNT };
const char* const STREAM_NAME[S_COUNT] = { "ACC", "GYRO_UC", "MAG_UC", "REF" };

enum State { ST_IDLE, ST_SETTLING, ST_RECORDING };
State state = ST_IDLE;

bool     useRV = false;              // false = GRV, true = RV
char     seqName[MAX_NAME_LEN + 1];
char     cmdEcho[MAX_CMD_LEN + 1];
uint32_t requestedSec = 0;
uint64_t settleEndUs = 0;
uint64_t recordStartUs = 0;
uint32_t seq[S_COUNT];
bool     okFlag = true;

char    cmdBuf[MAX_CMD_LEN + 1];
uint8_t cmdLen = 0;

/******************************************************************************
 * BNO085 report control
 */

const char* refTag() { return useRV ? "BNO_RV" : "BNO_GRV"; }

bool bnoEnableReports() {
  bool ok = true;
  ok &= bno.enableReport(SH2_ACCELEROMETER,               REPORT_INTERVAL_US);
  ok &= bno.enableReport(SH2_GYROSCOPE_UNCALIBRATED,      REPORT_INTERVAL_US);
  ok &= bno.enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, REPORT_INTERVAL_US);
  ok &= bno.enableReport(useRV ? SH2_ROTATION_VECTOR
                               : SH2_GAME_ROTATION_VECTOR, REPORT_INTERVAL_US);
  return ok;
}

// An interval of 0 tells the BNO085 to stop sending that report.
void bnoDisableReports() {
  bno.enableReport(SH2_ACCELEROMETER,               0);
  bno.enableReport(SH2_GYROSCOPE_UNCALIBRATED,      0);
  bno.enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, 0);
  bno.enableReport(SH2_ROTATION_VECTOR,             0);
  bno.enableReport(SH2_GAME_ROTATION_VECTOR,        0);
}

/******************************************************************************
 * Metadata output
 */

void printIdent() {
  Serial.printf("# %s\n", SKETCH_VERSION);
  Serial.printf("# state=%s\n",
                state == ST_IDLE      ? "idle"
              : state == ST_SETTLING  ? "settling"
                                      : "recording");
  Serial.println("# commands: S <G|R> <seconds> <name> | X | ?");
  Serial.printf("# report_interval_us=%d settle_us=%d i2c_hz=%d\n",
                REPORT_INTERVAL_US, SETTLE_US, I2C_CLOCK_HZ);
}

void printHeader() {
  Serial.println("# BEGIN");
  Serial.printf("# %s\n", SKETCH_VERSION);
  Serial.printf("# command: %s\n", cmdEcho);
  Serial.printf("# name=%s reference=%s requested_s=%lu\n",
                seqName, refTag(), (unsigned long)requestedSec);
  Serial.printf("# report_interval_us=%d settle_us=%d i2c_hz=%d\n",
                REPORT_INTERVAL_US, SETTLE_US, I2C_CLOCK_HZ);
  Serial.println("# columns: t_us,tag,seq,bno_delay_us,status,<values>");
  Serial.println("#   t_us         microseconds since the start of this recording");
  Serial.println("#   seq          per-stream counter; gaps mean rows were lost");
  Serial.println("#   bno_delay_us signed: how long before the report the sample was taken");
  Serial.println("#   status       BNO085 accuracy, 0 (unreliable) to 3 (high)");
  Serial.println("# BNO_ACC:     ax,ay,az [m/s^2]");
  Serial.println("# BNO_GYRO_UC: gx,gy,gz,bias_x,bias_y,bias_z [rad/s]");
  Serial.println("# BNO_MAG_UC:  mx,my,mz,bias_x,bias_y,bias_z [uT]");
  if (useRV) {
    Serial.println("# BNO_RV:      qw,qx,qy,qz,heading_accuracy [rad]");
  } else {
    Serial.println("# BNO_GRV:     qw,qx,qy,qz");
  }
  Serial.println("# DATA");
}

void printEnd(uint64_t elapsedUs) {
  Serial.printf("# END name=%s reference=%s duration_s=%.3f",
                seqName, refTag(), elapsedUs * 1e-6);
  for (int i = 0; i < S_COUNT; i++) {
    const char* label = (i == S_REF) ? refTag() : STREAM_NAME[i];
    Serial.printf(" %s=%lu", label, (unsigned long)seq[i]);
  }
  Serial.printf(" ok=%d\n", okFlag ? 1 : 0);
}

/******************************************************************************
 * Recording control
 */

void startRecording() {
  bnoDisableReports();
  if (!bnoEnableReports()) {
    Serial.println("# ERROR: could not enable BNO085 reports");
    state = ST_IDLE;
    return;
  }
  for (int i = 0; i < S_COUNT; i++) seq[i] = 0;
  okFlag = true;
  settleEndUs = time_us_64() + SETTLE_US;
  state = ST_SETTLING;
}

void stopRecording() {
  uint64_t elapsedUs = time_us_64() - recordStartUs;
  bnoDisableReports();
  state = ST_IDLE;
  printEnd(elapsedUs);
}

/******************************************************************************
 * Command handling
 */

void handleCommand(char* s) {
  while (*s == ' ') s++;
  if (*s == '\0') return;

  strncpy(cmdEcho, s, MAX_CMD_LEN);
  cmdEcho[MAX_CMD_LEN] = '\0';

  char c = *s;

  if (c == '?') {
    printIdent();
    return;
  }

  if (c == 'X' || c == 'x') {
    if (state == ST_IDLE) {
      Serial.println("# ERROR: not recording");
    } else if (state == ST_SETTLING) {
      bnoDisableReports();
      state = ST_IDLE;
      Serial.println("# ERROR: stopped during settling, no data recorded");
    } else {
      stopRecording();
    }
    return;
  }

  if (c == 'S' || c == 's') {
    if (state != ST_IDLE) {
      Serial.println("# ERROR: already recording, send X first");
      return;
    }

    strtok(s, " ");                        // discard the "S"
    char* refTok  = strtok(NULL, " ");
    char* secTok  = strtok(NULL, " ");
    char* nameTok = strtok(NULL, " ");

    if (!refTok || !secTok) {
      Serial.println("# ERROR: usage S <G|R> <seconds> <name>");
      return;
    }
    if (*refTok == 'R' || *refTok == 'r') {
      useRV = true;
    } else if (*refTok == 'G' || *refTok == 'g') {
      useRV = false;
    } else {
      Serial.println("# ERROR: reference must be G or R");
      return;
    }

    long sec = atol(secTok);
    if (sec < 0) {
      Serial.println("# ERROR: seconds must be >= 0");
      return;
    }
    requestedSec = (uint32_t)sec;

    if (nameTok) {
      strncpy(seqName, nameTok, MAX_NAME_LEN);
      seqName[MAX_NAME_LEN] = '\0';
    } else {
      strcpy(seqName, "unnamed");
    }

    startRecording();
    return;
  }

  Serial.printf("# ERROR: unknown command '%c'\n", c);
}

void readSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < MAX_CMD_LEN) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

/******************************************************************************
 * BNO085 servicing
 */

void serviceBno() {
  if (state == ST_IDLE) return;

  if (bno.wasReset()) {
    if (state == ST_RECORDING) {
      Serial.printf("# WARN: BNO085 reset at t_us=%llu, data may have a gap\n",
                    (unsigned long long)(time_us_64() - recordStartUs));
      okFlag = false;
    }
    bnoEnableReports();
  }

  for (int i = 0; i < DRAIN_PER_PASS; i++) {
    if (!bno.getSensorEvent(&bno_val)) break;

    uint64_t now = time_us_64();

    // Still settling: read and discard so the queue stays clear.
    if (state == ST_SETTLING) {
      if (now >= settleEndUs) {
        recordStartUs = now;
        state = ST_RECORDING;
        printHeader();
      } else {
        continue;
      }
    }

    uint64_t t     = now - recordStartUs;
    long     delay = (long)(int32_t)(uint32_t)bno_val.timestamp;
    unsigned st    = bno_val.status & 0x03;

    switch (bno_val.sensorId) {
      case SH2_ACCELEROMETER: {
        auto& a = bno_val.un.accelerometer;
        Serial.printf("%llu,BNO_ACC,%lu,%ld,%u,%.5f,%.5f,%.5f\n",
                      (unsigned long long)t, (unsigned long)seq[S_ACC]++,
                      delay, st, a.x, a.y, a.z);
        break;
      }
      case SH2_GYROSCOPE_UNCALIBRATED: {
        auto& g = bno_val.un.gyroscopeUncal;
        Serial.printf("%llu,BNO_GYRO_UC,%lu,%ld,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                      (unsigned long long)t, (unsigned long)seq[S_GYRO]++,
                      delay, st, g.x, g.y, g.z, g.biasX, g.biasY, g.biasZ);
        break;
      }
      case SH2_MAGNETIC_FIELD_UNCALIBRATED: {
        auto& m = bno_val.un.magneticFieldUncal;
        Serial.printf("%llu,BNO_MAG_UC,%lu,%ld,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                      (unsigned long long)t, (unsigned long)seq[S_MAG]++,
                      delay, st, m.x, m.y, m.z, m.biasX, m.biasY, m.biasZ);
        break;
      }
      case SH2_ROTATION_VECTOR: {
        auto& q = bno_val.un.rotationVector;
        Serial.printf("%llu,BNO_RV,%lu,%ld,%u,%.6f,%.6f,%.6f,%.6f,%.4f\n",
                      (unsigned long long)t, (unsigned long)seq[S_REF]++,
                      delay, st, q.real, q.i, q.j, q.k, q.accuracy);
        break;
      }
      case SH2_GAME_ROTATION_VECTOR: {
        auto& q = bno_val.un.gameRotationVector;
        Serial.printf("%llu,BNO_GRV,%lu,%ld,%u,%.6f,%.6f,%.6f,%.6f\n",
                      (unsigned long long)t, (unsigned long)seq[S_REF]++,
                      delay, st, q.real, q.i, q.j, q.k);
        break;
      }
      default:
        Serial.printf("# WARN: unexpected sensorId 0x%02X\n", bno_val.sensorId);
        break;
    }
  }
}

void checkDuration() {
  if (state != ST_RECORDING || requestedSec == 0) return;
  if (time_us_64() - recordStartUs >= (uint64_t)requestedSec * 1000000ULL) {
    stopRecording();
  }
}

/******************************************************************************
 * Main
 */

void setup() {

  // Initialize serial
  Serial.begin(UART_CLOCK_HZ);

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

  // Disable BNO085 reporting (for now)
  bnoDisableReports();

  // Say we're ready
  Serial.printf("# %s ready\n", SKETCH_VERSION);
  Serial.println("# commands: S <G|R> <seconds> <name> | X | ?");
}

void loop() {
  readSerialCommand();
  serviceBno();
  checkDuration();
}