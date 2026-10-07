/**
 * sensor_logger.ino
 *
 * Serial command-driven logger for RP2350 connected to Adafruit BNO085.
 * Written for the earlephilower Pico core (required for time_us_64()).
 *
 * Protocol:
 *  S <N|G|R> <seconds> Start recording. G=game rotation vector, N = no 
 *                      reference vector, R = rotation vector, seconds = 0 runs
 *                      until stopped.
 *  X                   Stop the current recording
 *
 * Output:
 *  "# ..." are metadata
 *  Each recording starts with "# BEGIN" and ends with "# END"
 *
 * Note: GRV and RV are mutually exclusive. The BNO085 cannot compute both at
 * the same time.
 */

#include <Adafruit_BNO08x.h>

/******************************************************************************
 * Settings
 */

#define UART_CLOCK_HZ       500000
#define I2C_CLOCK_HZ        400000
#define BNO_RESET_PIN       -1
#define REPORT_INTERVAL_US  10000    // 100 Hz
#define SETTLE_TIME_US      200000   // Wait this long (us) before logging data
#define MAX_CMD_LEN         80
#define MAX_NAME_LEN        32
#define DRAIN_PER_PASS      8

/******************************************************************************
 * Structs and enums
 */

// Switch to record none or one of the reference rotation vector
typedef enum {
  NONE,         // No reference output
  GRV,          // Game rotation vector (6-DoF fusion, no magnetometer)
  RV            // Rotation vector (9-DoF fusion, includes magnetometer)
} BnoRef;

// Used to record the number of reports received for each type of stream
typedef enum {
  S_ACC,
  S_GYRO,
  S_MAG,
  S_REF,
  S_COUNT
} BnoStream;

// Simple state machine to determine what the logger is doing
typedef enum {
  ST_IDLE,      // Waiting for a command
  ST_SETTLING,  // Waiting the initial period before recoding
  ST_RECORDING  // Recording data
} State;

/******************************************************************************
 * Globals
 */

Adafruit_BNO08x bno(BNO_RESET_PIN);
sh2_SensorValue_t bno_val;
BnoRef bno_ref = NONE;
State state = ST_IDLE;
uint64_t state_timestamp_us = 0;
uint64_t record_start_us = 0;
char cmd_buf[MAX_CMD_LEN + 1];
uint32_t seq_count[S_COUNT];
uint8_t cmd_len = 0;
uint32_t req_sec = 0;
bool ok_flag = true;

/******************************************************************************
 * Functions
 */

// Enable specific reports from the BNO085
bool bnoEnableReports(BnoRef ref) {
  bool ok = true;

  // Raw reports
  ok &= bno.enableReport(SH2_ACCELEROMETER, REPORT_INTERVAL_US);
  ok &= bno.enableReport(SH2_GYROSCOPE_UNCALIBRATED, REPORT_INTERVAL_US);
  ok &= bno.enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, REPORT_INTERVAL_US);

  // Reference report, depending on the setting
  switch (ref) {
    case NONE:
      break;
    case GRV:
      ok &= bno.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_INTERVAL_US);
      break;
    case RV:
      ok &= bno.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US);
      break;
    default:
      break;
  }

  return ok;
}

// Disable all reporting from the BNO085
bool bnoDisableReports() {
  bool ok = true;

  // Disable all reports
  ok &= bno.enableReport(SH2_ACCELEROMETER, 0);
  ok &= bno.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 0);
  ok &= bno.enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, 0);
  ok &= bno.enableReport(SH2_GAME_ROTATION_VECTOR, 0);
  ok &= bno.enableReport(SH2_ROTATION_VECTOR, 0);

  return ok;
}

// Start recording data from the BNO085
void startRecording() {
  // Enable only the requested reports
  bnoDisableReports();
  if (!bnoEnableReports(bno_ref)) {
    Serial.println("# ERROR: could not enable BNO085 reports");
    state = ST_IDLE;
    return;
  }

  // Reset sequence counters
  for (int i = 0; i < S_COUNT; i++) {
    seq_count[i] = 0;
  }

  // Reset OK flag
  ok_flag = true;

  // Start the settling period
  state_timestamp_us = time_us_64();
  state = ST_SETTLING;
}

// Stop recording data from the BNO085
void stopRecording() {
  uint64_t elapsed_us = time_us_64() - record_start_us;
  bnoDisableReports();
  state = ST_IDLE;
  printEnd(elapsed_us);
}

// Print header describing the columns for each report
void printHeader() {
  Serial.println("# BEGIN");
  Serial.println("# BNO_ACC: t_us,tag,seq,bno_delay_us,status,ax,ay,az");
  Serial.println("# BNO_GYRO_UC: t_us,tag,seq,bno_delay_us,status,gx,gy,gz,gbias_x,gbias_y,gbias_z");
  Serial.println("# BNO_MAG_UC: t_us,tag,seq,bno_delay_us,status,mx,my,mz,mbias_x,mbias_y,mbias_z");
  switch (bno_ref) {
    case GRV:
      Serial.println("# BNO_GRV: t_us,tag,seq,bno_delay_us,status,qw,qx,qy,qz");
      break;
    case RV:
      Serial.println("# BNO_RV: t_us,tag,seq,bno_delay_us,status,qw,qx,qy,qz,heading_accuracy");
      break;
    default:
      break;
  }
  Serial.println("# DATA");
}
// Parse serial command and perform action
void handleCommand(char* s) {

  // Remove whitespace
  while (*s == ' ') {
    s++;
  }

  // If string is empty, return early
  if (*s == '\0') {
    return;
  }

  // Get first character
  char c = *s;

  // Stop recording
  if (c == 'X' || c == 'x') {
    switch (state) {
      case ST_IDLE:
        Serial.println("# ERROR: not recording");
        break;
      case ST_SETTLING:
        bnoDisableReports();
        state = ST_IDLE;
        Serial.println("# ERROR: stopped during settling, no data recorded");
        break;
      default:
        stopRecording();
        break;
    }

    return;
  }

  // Parse the recording command and start the logging process
  if (c == 'S' || c == 's') {
    // Throw an error if we're already recording
    if (state != ST_IDLE) {
      Serial.println("# ERROR: already recording, send X first");
      return;
    }

    // Disacrd the "S"
    strtok(s, " "); 

    // Tokenize the rest of the command
    char* tok_ref = strtok(NULL, " ");
    char* tok_sec = strtok(NULL, " ");

    // Notify user if wrong syntax
    if (!tok_ref || !tok_sec) {
      Serial.println("# ERROR: usage S <N|G|R> <seconds>");
      return;
    }

    // Set reference vector
    if (*tok_ref == 'N' || *tok_ref == 'n') {
      bno_ref = NONE;
    } else if (*tok_ref == 'R' || *tok_ref == 'r') {
      bno_ref = RV;
    } else if (*tok_ref == 'G' || *tok_ref == 'g') {
      bno_ref = GRV;
    } else {
      Serial.println("# ERROR: reference must be N, G, or R");
      return;
    }

    // Set length of time to log
    long sec = atol(tok_sec);
    if (sec < 0) {
      Serial.println("# ERROR: seconds must be >= 0");
      return;
    }
    req_sec = (uint32_t)sec;

    // Start recording
    startRecording();

    return;
  }

  Serial.printf("# ERROR: unknown command '%c'\n", c);
}

// Print end metadata
void printEnd(uint64_t elapsedUs) {
  Serial.printf("# END duration_s=%.3f acc=%lu gyro=%lu mag=%lu ref=%lu ok=%d\n",
                elapsedUs * 1e-6,
                (unsigned long)seq_count[S_ACC], 
                (unsigned long)seq_count[S_GYRO],
                (unsigned long)seq_count[S_MAG], 
                (unsigned long)seq_count[S_REF],
                ok_flag ? 1 : 0);
}

// Fill command buffer with serial data, parse command if newline spotted
void readSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd_len > 0) {
        cmd_buf[cmd_len] = '\0';
        handleCommand(cmd_buf);
        cmd_len = 0;
      }
    } else if (cmd_len < MAX_CMD_LEN) {
      cmd_buf[cmd_len++] = c;
    }
  }
}

// Read from BNO085 and print data
void serviceBno() {
  // Do nothing if idle state
  if (state == ST_IDLE) {
    return;
  }

  // Check if BNO085 was reset, log warning, re-enable reports
  if (bno.wasReset()) {
    if (state == ST_RECORDING) {
      Serial.printf("# WARN: BNO085 reset at t_us=%llu, data may have a gap\n",
                    (unsigned long long)(time_us_64() - record_start_us));
      ok_flag = false;
    }
    bnoEnableReports(bno_ref);
  }

  // Read and print reports a few times per service call
  for (int i = 0; i < DRAIN_PER_PASS; i++) {
    // If there are no reports, stop reading
    if (!bno.getSensorEvent(&bno_val)) {
      break;
    }

    // Get a timestamp
    uint64_t now = time_us_64();

    // If still in settling period, read and discard reports
    if (state == ST_SETTLING) {
      if (now >= state_timestamp_us + SETTLE_TIME_US) {
        record_start_us = now;
        state = ST_RECORDING;
        printHeader();
      } else {
        continue;
      }
    }

    // Get MCU timestamp (from recording start), BNO timestamp, and BNO status
    uint64_t t = now - state_timestamp_us;
    long delay = (long)(int32_t)(uint32_t)bno_val.timestamp;
    unsigned st = bno_val.status & 0x03;

    // Print out appropriate report
    switch (bno_val.sensorId) {

      // Accelerometer report
      case SH2_ACCELEROMETER: {
        auto &a = bno_val.un.accelerometer;
        Serial.printf("%llu,BNO_ACC,%lu,%ld,%u,%.5f,%.5f,%.5f\n",
                      (unsigned long long)t, (unsigned long)seq_count[S_ACC],
                      delay, st, a.x, a.y, a.z);
        seq_count[S_ACC]++;
        break;
      }

      // Gyroscope report
      case SH2_GYROSCOPE_UNCALIBRATED: {
        auto &g = bno_val.un.gyroscopeUncal;
        Serial.printf("%llu,BNO_GYRO_UC,%lu,%ld,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                      (unsigned long long)t, (unsigned long)seq_count[S_GYRO],
                      delay, st, g.x, g.y, g.z, g.biasX, g.biasY, g.biasZ);
        seq_count[S_GYRO]++;
        break;
      }

      // Magnetometer report
      case SH2_MAGNETIC_FIELD_UNCALIBRATED: {
        auto &m = bno_val.un.magneticFieldUncal;
        Serial.printf("%llu,BNO_MAG_UC,%lu,%ld,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                      (unsigned long long)t, (unsigned long)seq_count[S_MAG],
                      delay, st, m.x, m.y, m.z, m.biasX, m.biasY, m.biasZ);
        seq_count[S_MAG]++;
        break;
      }

      // Rotation vector reference report
      case SH2_ROTATION_VECTOR: {
        auto &q = bno_val.un.rotationVector;
        Serial.printf("%llu,BNO_RV,%lu,%ld,%u,%.6f,%.6f,%.6f,%.6f,%.4f\n",
                      (unsigned long long)t, (unsigned long)seq_count[S_REF],
                      delay, st, q.real, q.i, q.j, q.k, q.accuracy);
        seq_count[S_REF]++;
        break;
      }

      // Rotation game rotation vector report
      case SH2_GAME_ROTATION_VECTOR: {
        auto &q = bno_val.un.gameRotationVector;
        Serial.printf("%llu,BNO_GRV,%lu,%ld,%u,%.6f,%.6f,%.6f,%.6f\n",
                      (unsigned long long)t, (unsigned long)seq_count[S_REF],
                      delay, st, q.real, q.i, q.j, q.k);
        seq_count[S_REF]++;
        break;
      }

      // Default: throw error
      default:
        Serial.printf("# WARN: unexpected sensorId 0x%02X\n", bno_val.sensorId);
        break;
    }
  }
}

// Check reporting duration and stop recording if specified time has elapsed
void checkDuration() {
  // Early return if we're not recording
  if (state != ST_RECORDING || req_sec == 0) {
    return;
  }

  // If requested time has elapsed, stop recording
  if (time_us_64() - record_start_us >= (uint64_t)req_sec * 1000000ULL) {
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

  // Disable dynamic calibration
  sh2_setCalConfig(0);

  // Disable BNO085 reporting (for now)
  bnoDisableReports();

  // Say we're ready
  Serial.println("# commands: S <G|R> <seconds> <name> | X");
}

void loop() {
  readSerialCommand();
  serviceBno();
  checkDuration();
}
