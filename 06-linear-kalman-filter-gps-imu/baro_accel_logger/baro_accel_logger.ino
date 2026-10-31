/**
 * baro_accel_logger.ino
 *
 * Serial command-driven logger for RP2350 connected to an Adafruit BNO085
 * and an Adafruit BMP581. Logs vertical (z-axis) acceleration and barometric
 * altitude in a single CSV so we can build a 1D Kalman filter for vertical
 * position and velocity in Python (episode 5).
 * Written for the earlephilhower Pico core (required for time_us_64()).
 *
 * Libraries: Adafruit BNO08x, Adafruit BMP5xx
 *
 * Assumption: the board stays flat and face up, so sensor z is vertical.
 * az includes gravity (about +9.81 m/s^2 at rest). Subtract the at-rest mean
 * in Python, which also removes accelerometer bias.
 *
 * Wiring:
 *  BNO085 -> STEMMA QT (I2C, default address 0x4A)
 *  BMP581 -> STEMMA QT (I2C, default address 0x47, alt 0x46)
 *
 * Protocol:
 *  S <seconds>   Start recording. seconds = 0 runs until stopped.
 *  X             Stop the current recording
 *
 * Output:
 *  "# ..." are metadata
 *  Each recording starts with "# BEGIN" and ends with "# END"
 *  "# CFG key=value" lines record settings
 *  "# TAG: columns" describes the columns
 *
 * Rows:
 *  One row per accelerometer sample (100 Hz). The barometer runs at 50 Hz, so
 *  each row carries the most recent barometer sample:
 *   baro_new    = 1 on the first row after a new barometer sample (run the
 *                 Kalman update on these rows), 0 when the value is repeated
 *   baro_age_us = how long before this row the barometer sample was read
 *  Barometer columns are empty until the first barometer sample arrives.
 *
 * Barometer: normal mode at a fixed ODR with the IIR filter bypassed, so
 * Python sees unfiltered pressure. Each sample is read only when the chip
 * reports data ready, so there are no duplicated samples.
 */

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP5xx.h>
#include <math.h>

/******************************************************************************
 * Settings
 */

#define UART_CLOCK_HZ       500000
#define I2C_CLOCK_HZ        400000
#define SETTLE_TIME_US      500000   // Discard data this long after start
#define MAX_CMD_LEN         80
#define DRAIN_PER_PASS      8

// BNO085
#define BNO_RESET_PIN       -1
#define ACC_INTERVAL_US     10000    // 100 Hz

// BMP581
#define BMP_ADDR_PRIMARY    0x47     // Adafruit breakout default
#define BMP_ADDR_ALT        0x46
#define BMP_ODR_HZ          50.0     // Must match setOutputDataRate() below
#define BMP_POLL_US         1000     // How often to check data-ready

// Barometric formula. Absolute altitude is offset by weather; only changes
// matter for this demo, so zero it in Python.
#define SEA_LEVEL_PA        101325.0

/******************************************************************************
 * Structs and enums
 */

// Simple state machine to determine what the logger is doing
typedef enum {
  ST_IDLE,      // Waiting for a command
  ST_SETTLING,  // Waiting the initial period before recording
  ST_RECORDING  // Recording data
} State;

// Most recent barometer sample, attached to each accelerometer row
typedef struct {
  bool valid;         // At least one sample since the recording started
  bool fresh;         // Not yet printed on a row
  uint64_t t_us;      // MCU time the sample was read (absolute)
  double press_pa;
  double temp_c;
  double alt_m;
} BaroSample;

/******************************************************************************
 * Globals
 */

Adafruit_BNO08x bno(BNO_RESET_PIN);
sh2_SensorValue_t bno_val;
Adafruit_BMP5xx bmp;
uint8_t bmp_addr = 0;
State state = ST_IDLE;
uint64_t state_timestamp_us = 0;
uint64_t record_start_us = 0;
uint32_t req_sec = 0;
char cmd_buf[MAX_CMD_LEN + 1];
uint8_t cmd_len = 0;
bool ok_flag = true;

// Counters
uint32_t acc_count = 0;
uint32_t baro_count = 0;
uint32_t baro_gap_count = 0;
uint32_t baro_drop_count = 0;
uint32_t baro_err_count = 0;

// Barometer timing and held sample
uint64_t next_bmp_poll_us = 0;
uint64_t last_bmp_us = 0;
BaroSample baro;

/******************************************************************************
 * BMP581 functions
 */

// Find and configure the BMP581, then start it in normal mode
bool bmpInit() {
  // Find the chip at either address
  if (bmp.begin(BMP_ADDR_PRIMARY, &Wire)) {
    bmp_addr = BMP_ADDR_PRIMARY;
  } else if (bmp.begin(BMP_ADDR_ALT, &Wire)) {
    bmp_addr = BMP_ADDR_ALT;
  } else {
    return false;
  }

  // Raw-ish pressure: x16 oversampling, no on-chip IIR (filter in Python)
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_1X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_BYPASS);
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
  bmp.enablePressure(true);

  // Data-ready flag (polled; the INT pin isn't wired)
  bmp.configureInterrupt(BMP5XX_INTERRUPT_LATCHED,
                         BMP5XX_INTERRUPT_ACTIVE_HIGH,
                         BMP5XX_INTERRUPT_PUSH_PULL,
                         BMP5XX_INTERRUPT_DATA_READY, true);

  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);
  return true;
}

/******************************************************************************
 * BNO085 functions
 */

// Enable only the accelerometer report
bool bnoEnableReports() {
  return bno.enableReport(SH2_ACCELEROMETER, ACC_INTERVAL_US);
}

// Disable all reports this logger (or a previous sketch) might have enabled
bool bnoDisableReports() {
  bool ok = true;
  ok &= bno.enableReport(SH2_ACCELEROMETER, 0);
  ok &= bno.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 0);
  ok &= bno.enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, 0);
  ok &= bno.enableReport(SH2_GAME_ROTATION_VECTOR, 0);
  ok &= bno.enableReport(SH2_ROTATION_VECTOR, 0);
  return ok;
}

/******************************************************************************
 * Recording functions
 */

// Print header with settings and columns
void printHeader() {
  Serial.println("# BEGIN");
  Serial.printf("# CFG acc_interval_us=%d\n", ACC_INTERVAL_US);
  Serial.printf("# CFG settle_time_us=%d\n", SETTLE_TIME_US);
  Serial.println("# CFG bno_dynamic_cal=0");
  Serial.printf("# CFG bmp_addr=0x%02X\n", bmp_addr);
  Serial.printf("# CFG bmp_odr_hz=%.1f\n", BMP_ODR_HZ);
  Serial.println("# CFG bmp_osr_p=16");
  Serial.println("# CFG bmp_osr_t=1");
  Serial.println("# CFG bmp_iir=bypass");
  Serial.printf("# CFG sea_level_pa=%.1f\n", SEA_LEVEL_PA);
  Serial.println("# ACC_BARO: t_us,tag,seq,az,acc_status,"
                 "baro_new,baro_age_us,press_pa,temp_c,alt_m");
  Serial.println("# DATA");
}

// Start the settling period
void startRecording() {
  bnoDisableReports();
  if (!bnoEnableReports()) {
    Serial.println("# ERROR: could not enable BNO085 reports");
    state = ST_IDLE;
    return;
  }

  // Reset counters and the held barometer sample
  acc_count = 0;
  baro_count = 0;
  baro_gap_count = 0;
  baro_drop_count = 0;
  baro_err_count = 0;
  last_bmp_us = 0;
  baro.valid = false;
  baro.fresh = false;
  ok_flag = true;

  state_timestamp_us = time_us_64();
  state = ST_SETTLING;
}

// Stop recording and print end metadata
void stopRecording() {
  uint64_t elapsed_us = time_us_64() - record_start_us;
  bnoDisableReports();
  state = ST_IDLE;
  Serial.printf("# END duration_s=%.3f acc=%lu baro=%lu baro_gaps=%lu "
                "baro_dropped=%lu baro_err=%lu ok=%d\n",
                elapsed_us * 1e-6,
                (unsigned long)acc_count,
                (unsigned long)baro_count,
                (unsigned long)baro_gap_count,
                (unsigned long)baro_drop_count,
                (unsigned long)baro_err_count,
                ok_flag ? 1 : 0);
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

  // Parse the recording command and start the settling period
  if (c == 'S' || c == 's') {
    // Throw an error if we're already recording
    if (state != ST_IDLE) {
      Serial.println("# ERROR: already recording, send X first");
      return;
    }

    // Discard the "S" and get the duration
    strtok(s, " ");
    char* tok_sec = strtok(NULL, " ");

    // Notify user if wrong syntax
    if (!tok_sec) {
      Serial.println("# ERROR: usage S <seconds>");
      return;
    }

    // Set length of time to log
    long sec = atol(tok_sec);
    if (sec < 0) {
      Serial.println("# ERROR: seconds must be >= 0");
      return;
    }
    req_sec = (uint32_t)sec;

    startRecording();
    return;
  }

  Serial.printf("# ERROR: unknown command '%c'\n", c);
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

// Switch from settling to recording once the settle time has passed
void serviceSettling() {
  if (state != ST_SETTLING) {
    return;
  }
  if (time_us_64() - state_timestamp_us >= SETTLE_TIME_US) {
    printHeader();
    record_start_us = time_us_64();
    state = ST_RECORDING;
  }
}

// Poll the BMP581 data-ready flag and hold each new sample for the next row
void serviceBmp() {
  // Do nothing if idle
  if (state == ST_IDLE) {
    return;
  }

  // Only poll every BMP_POLL_US to keep the I2C bus free for the BNO085
  uint64_t now = time_us_64();
  if (now < next_bmp_poll_us) {
    return;
  }
  next_bmp_poll_us = now + BMP_POLL_US;

  // Wait for a new sample
  if (!bmp.dataReady()) {
    return;
  }

  // Read temperature and pressure
  bool recording = (state == ST_RECORDING);
  if (!bmp.performReading()) {
    if (recording) {
      baro_err_count++;
    }
    return;
  }
  now = time_us_64();

  // Library reports hPa. Altitude in double so the formula adds no rounding.
  double press_pa = (double)bmp.pressure * 100.0;

  // During settling, hold the sample (so the first rows have a value) but
  // don't mark it new or count it
  if (recording) {
    // Count gaps longer than 1.5 sample periods (missed samples)
    const uint64_t gap_us = (uint64_t)(1.5e6 / BMP_ODR_HZ);
    if (last_bmp_us != 0 && now - last_bmp_us > gap_us) {
      baro_gap_count++;
    }
    last_bmp_us = now;

    // A previous sample that never made it onto a row was overwritten
    if (baro.fresh) {
      baro_drop_count++;
    }
    baro_count++;
  }

  baro.valid = true;
  baro.fresh = recording;
  baro.t_us = now;
  baro.press_pa = press_pa;
  baro.temp_c = bmp.temperature;
  baro.alt_m = 44330.0 * (1.0 - pow(press_pa / SEA_LEVEL_PA, 0.190295));
}

// Read accelerometer reports and print one row per sample
void serviceBno() {
  // Do nothing if idle
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
    bnoEnableReports();
  }

  // Read and print reports a few times per service call
  for (int i = 0; i < DRAIN_PER_PASS; i++) {
    // If there are no reports, stop reading
    if (!bno.getSensorEvent(&bno_val)) {
      break;
    }

    // Discard reports while settling or of an unexpected type
    if (state != ST_RECORDING || bno_val.sensorId != SH2_ACCELEROMETER) {
      continue;
    }

    // MCU timestamp (from recording start) and BNO status
    uint64_t now = time_us_64();
    uint64_t t = now - record_start_us;
    unsigned st = bno_val.status & 0x03;
    float az = bno_val.un.accelerometer.z;

    // Print row with the most recent barometer sample (empty if none yet)
    if (baro.valid) {
      Serial.printf("%llu,ACC_BARO,%lu,%.5f,%u,%d,%lu,%.4f,%.4f,%.4f\n",
                    (unsigned long long)t, (unsigned long)acc_count,
                    az, st, baro.fresh ? 1 : 0,
                    (unsigned long)(now - baro.t_us),
                    baro.press_pa, baro.temp_c, baro.alt_m);
      baro.fresh = false;
    } else {
      Serial.printf("%llu,ACC_BARO,%lu,%.5f,%u,0,,,,\n",
                    (unsigned long long)t, (unsigned long)acc_count, az, st);
    }
    acc_count++;
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
      Serial.println("# ERROR: could not initialize BNO085");
      if (bno.begin_I2C()) {
        break;
      }
      delay(1000);
    }
  }

  // Initialize BMP581. Both sensors are needed for the single CSV, so keep
  // trying if it fails.
  if (!bmpInit()) {
    while (1) {
      Serial.println("# ERROR: could not initialize BMP581");
      if (bmpInit()) {
        break;
      }
      delay(1000);
    }
  }

  // Set I2C speed (after both begin() calls, in case either resets it)
  Wire.setClock(I2C_CLOCK_HZ);

  // Disable dynamic calibration so the accelerometer bias can't shift
  // mid-recording (the at-rest mean is removed in Python instead)
  sh2_setCalConfig(0);

  // Disable BNO085 reporting (for now)
  bnoDisableReports();

  // Say we're ready
  Serial.println("# commands: S <seconds> | X");
}

void loop() {
  readSerialCommand();
  serviceSettling();
  serviceBmp();         // Before the BNO, so a new sample lands on the next row
  serviceBno();
  checkDuration();
}
