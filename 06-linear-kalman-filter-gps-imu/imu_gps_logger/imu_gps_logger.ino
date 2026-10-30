/**
 * imu_gps_logger.ino
 *
 * Serial command-driven logger for RP2350 connected to an Adafruit BNO085
 * and an Adafruit Mini GPS PA1010D. Logs 3-axis acceleration plus the
 * absolute orientation quaternion (100 Hz) and GPS fixes (1 Hz) so we can
 * build a 4-state Kalman filter for horizontal position and velocity in
 * Python (episode 6).
 * Written for the earlephilhower Pico core (required for time_us_64()).
 *
 * Libraries: Adafruit BNO08x
 * (NMEA is parsed here so lat/lon stay in double precision.)
 *
 * The board can be in any orientation. Acceleration is logged in the sensor
 * frame and includes gravity. In Python, rotate it into the world frame with
 * the quaternion, then subtract gravity. The rotation vector uses the
 * magnetometer, so its yaw is referenced to magnetic north (the BNO08x
 * default world frame is East-North-Up; verify this in Python by pointing
 * the board's +y axis at a known heading).
 *
 * Wiring:
 *  BNO085  -> STEMMA QT (I2C, default address 0x4A)
 *  PA1010D -> GPS_USE_I2C = 1: STEMMA QT (I2C, address 0x10), daisy-chained
 *             GPS_USE_I2C = 0: UART, GPS TX -> Feather RX,
 *             GPS RX -> Feather TX (Serial1), VIN -> 3.3V, GND -> GND
 *             Over I2C, sentence arrival times are only known to within one
 *             poll period (GPS_POLL_US), so wire PPS if timing matters.
 *  PA1010D PPS -> GPS_PPS_PIN (optional, set to -1 if not wired). The GPS
 *             drives this pin, so no pull resistor is needed (RP2350 E9 safe).
 *
 * Protocol:
 *  S <seconds>   Start recording. seconds = 0 runs until stopped.
 *  X             Stop the current recording
 *
 * Output:
 *  "# ..." are metadata
 *  While idle, "# GPS fix=... sats=... hdop=..." prints every 2 s so you can
 *  wait for a fix before starting
 *  Each recording starts with "# BEGIN" and ends with "# END"
 *  "# CFG key=value" lines record settings
 *  "# TAG: columns" describes the columns (one line per stream)
 *
 * Streams (interleave and sort by t_us in Python):
 *  IMU: one row per accelerometer sample. Each row carries the most recent
 *       rotation vector sample; rv_age_us is how long before the row it
 *       was received.
 *  GPS: one row per GPS epoch (RMC + GGA with the same UTC time).
 *       t_us     = MCU time the first sentence of the epoch started arriving
 *                  (lags the fix by the receiver's output latency)
 *       pps_t_us = MCU time of the most recent PPS edge before that. At 1 Hz
 *                  the fix is valid at this edge, so use it to align GPS and
 *                  IMU time. Empty if PPS isn't wired or no fix yet.
 *       Position fields are empty when there is no fix.
 *       speed_mps / course_deg come from RMC (Doppler, for the level-up).
 */

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

/******************************************************************************
 * Settings
 */

#define UART_CLOCK_HZ       500000
#define I2C_CLOCK_HZ        400000
#define SETTLE_TIME_US      500000   // Discard data this long after start
#define MAX_CMD_LEN         80
#define DRAIN_PER_PASS      8

// Notification (LED) for GPS fix
#define LED_PIN             LED_BUILTIN
#define READY_MIN_SATS      6
#define READY_MAX_HDOP      2.0
#define READY_TIMEOUT_US    3000000  // LED off if no GPS update for 3 s

// BNO085
#define BNO_RESET_PIN       -1
#define ACC_INTERVAL_US     10000    // 100 Hz (BNO085 may deliver ~125 Hz)
#define RV_INTERVAL_US      10000    // 100 Hz

// GPS
#define GPS_USE_I2C         1        // 1 = I2C (STEMMA QT), 0 = UART
#define GPS_I2C_ADDR        0x10
#define GPS_I2C_CHUNK       32       // Bytes per I2C read
#define GPS_POLL_US         20000    // I2C poll interval (50 Hz)
#define GPS_MAX_CHUNKS      8        // Max reads per poll while data flows
#define GPS_SERIAL          Serial1
#define GPS_BAUD            9600     // PA1010D default (UART only)
#define GPS_RATE_MS         1000     // 1 Hz (keep 1 Hz for the demo)
#define GPS_PPS_PIN         5        // -1 to disable PPS timestamping
#define GPS_FIFO_SIZE       512      // UART RX buffer (bytes)
#define MAX_NMEA_LEN        120
#define MAX_NMEA_FIELDS     24
#define LOG_RAW_NMEA        0        // 1 = also print "# NMEA t_us $..." lines
#define IDLE_STATUS_US      2000000  // GPS status interval while idle

#define KNOTS_TO_MPS        0.514444

/******************************************************************************
 * Structs and enums
 */

// Simple state machine to determine what the logger is doing
typedef enum {
  ST_IDLE,      // Waiting for a command
  ST_SETTLING,  // Waiting the initial period before recording
  ST_RECORDING  // Recording data
} State;

// Most recent rotation vector sample, attached to each IMU row
typedef struct {
  bool valid;
  uint64_t t_us;      // MCU time received (absolute)
  float qi, qj, qk, qr;
  float acc_rad;      // BNO085 heading accuracy estimate (radians)
  uint8_t status;     // 0-3 calibration status
} RotSample;

// One GPS epoch, assembled from RMC and GGA sentences with the same UTC time
typedef struct {
  bool active;        // At least one sentence received
  uint64_t t_us;      // MCU time the first sentence started arriving
  bool pps_valid;
  uint64_t pps_us;    // Most recent PPS edge when the epoch started
  char utc[16];       // hhmmss.sss
  bool has_gga;
  bool has_rmc;

  // From GGA
  int fix;            // 0 = none, 1 = GPS, 2 = DGPS/SBAS
  int sats;
  bool has_hdop;
  double hdop;
  bool has_pos;
  double lat_deg;
  double lon_deg;
  bool has_alt;
  double alt_m;

  // From RMC
  char date[8];       // ddmmyy
  bool rmc_valid;     // RMC status 'A'
  bool has_vel;
  double speed_mps;
  double course_deg;
} GpsEpoch;

/******************************************************************************
 * Globals
 */

Adafruit_BNO08x bno(BNO_RESET_PIN);
sh2_SensorValue_t bno_val;
State state = ST_IDLE;
uint64_t state_timestamp_us = 0;
uint64_t record_start_us = 0;
uint32_t req_sec = 0;
char cmd_buf[MAX_CMD_LEN + 1];
uint8_t cmd_len = 0;
bool ok_flag = true;

// IMU
RotSample rv;

// GPS parsing
char nmea_buf[MAX_NMEA_LEN + 1];
uint8_t nmea_len = 0;
bool nmea_overflow = false;
uint64_t nmea_start_us = 0;
uint64_t next_gps_poll_us = 0;
GpsEpoch epoch;

// Last flushed GPS epoch, for the idle status line
int status_fix = 0;
int status_sats = 0;
double status_hdop = NAN;
uint64_t next_status_us = 0;
uint64_t status_us = 0;

// PPS (written in the interrupt)
volatile uint64_t pps_us = 0;
volatile uint32_t pps_count = 0;

// Counters
uint32_t imu_count = 0;
uint32_t gps_count = 0;
uint32_t gps_fix_count = 0;
uint32_t gps_cksum_err = 0;
uint32_t gps_overflow = 0;
uint32_t pps_at_start = 0;

/******************************************************************************
 * PPS
 */

void ppsIsr() {
  pps_us = time_us_64();
  pps_count++;
}

// Copy the 64-bit PPS time without the interrupt changing it mid-read
bool readPps(uint64_t* out_us, uint32_t* out_count) {
  noInterrupts();
  uint64_t t = pps_us;
  uint32_t n = pps_count;
  interrupts();
  if (out_us) {
    *out_us = t;
  }
  if (out_count) {
    *out_count = n;
  }
  return n > 0;
}

/******************************************************************************
 * GPS configuration
 */

// Send a PMTK command, adding the leading '$' and the checksum
void sendPmtk(const char* body) {
  uint8_t cs = 0;
  for (const char* p = body; *p; p++) {
    cs ^= (uint8_t)*p;
  }
  char line[MAX_NMEA_LEN];
  int len = snprintf(line, sizeof(line), "$%s*%02X\r\n", body, cs);
#if GPS_USE_I2C
  Wire.beginTransmission(GPS_I2C_ADDR);
  Wire.write((const uint8_t*)line, len);
  Wire.endTransmission();
#else
  GPS_SERIAL.write((const uint8_t*)line, len);
  GPS_SERIAL.flush();
#endif
  delay(100);
}

// Check the GPS answers on I2C
bool gpsI2cPresent() {
  Wire.beginTransmission(GPS_I2C_ADDR);
  return Wire.endTransmission() == 0;
}

void gpsInit() {
#if GPS_USE_I2C
  // Wire is already running for the BNO085. Keep trying if the GPS is
  // missing.
  while (!gpsI2cPresent()) {
    Serial.println("# ERROR: could not find PA1010D on I2C (0x10)");
    delay(1000);
  }
#else
  GPS_SERIAL.setFIFOSize(GPS_FIFO_SIZE);   // Must be before begin()
  GPS_SERIAL.begin(GPS_BAUD);
#endif
  delay(500);

  // Output only RMC and GGA, every fix
  sendPmtk("PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");

  // Fix rate
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "PMTK220,%d", GPS_RATE_MS);
  sendPmtk(cmd);

  // Enable SBAS corrections where available (WAAS/EGNOS)
  sendPmtk("PMTK313,1");
  sendPmtk("PMTK301,2");

  // Disable static navigation (position pinning) so stationary noise is real
  sendPmtk("PMTK386,0");

  if (GPS_PPS_PIN >= 0) {
    pinMode(GPS_PPS_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), ppsIsr, RISING);
  }
}

/******************************************************************************
 * NMEA parsing
 */

// Check and strip the "*hh" checksum. Returns false if missing or wrong.
bool nmeaChecksumOk(char* line) {
  char* star = strchr(line, '*');
  if (line[0] != '$' || !star || strlen(star) < 3) {
    return false;
  }
  uint8_t cs = 0;
  for (char* p = line + 1; p < star; p++) {
    cs ^= (uint8_t)*p;
  }
  uint8_t got = (uint8_t)strtoul(star + 1, NULL, 16);
  *star = '\0';
  return cs == got;
}

// Split on commas in place, keeping empty fields (strtok would skip them)
int splitFields(char* s, char** fields, int max_fields) {
  int n = 0;
  fields[n++] = s;
  while (*s && n < max_fields) {
    if (*s == ',') {
      *s = '\0';
      fields[n++] = s + 1;
    }
    s++;
  }
  return n;
}

// Convert NMEA ddmm.mmmm (or dddmm.mmmm) plus hemisphere to signed degrees
bool parseLatLon(const char* val, const char* hemi, double* out) {
  if (!val[0] || !hemi[0]) {
    return false;
  }
  double raw = atof(val);
  int deg = (int)(raw / 100.0);
  double minutes = raw - deg * 100.0;
  double d = deg + minutes / 60.0;
  if (hemi[0] == 'S' || hemi[0] == 'W') {
    d = -d;
  }
  *out = d;
  return true;
}

// Print a double, or nothing if missing, followed by a separator
void printOptDouble(bool has, double v, int decimals, char sep) {
  if (has) {
    Serial.printf("%.*f", decimals, v);
  }
  Serial.print(sep);
}

void printOptInt(bool has, int v, char sep) {
  if (has) {
    Serial.printf("%d", v);
  }
  Serial.print(sep);
}

// Print a finished GPS epoch (if recording) and update the status
void flushEpoch() {
  if (!epoch.active) {
    return;
  }

  if (epoch.has_gga) {
    status_fix = epoch.fix;
    status_sats = epoch.sats;
    status_hdop = epoch.has_hdop ? epoch.hdop : NAN;
    status_us = time_us_64();
  }

  // Only print epochs that started after the recording started
  if (state == ST_RECORDING && epoch.t_us >= record_start_us) {
    uint64_t t = epoch.t_us - record_start_us;
    Serial.printf("%llu,GPS,%lu,%s,%s,",
                  (unsigned long long)t, (unsigned long)gps_count,
                  epoch.utc, epoch.has_rmc ? epoch.date : "");
    printOptInt(epoch.has_gga, epoch.fix, ',');
    printOptInt(epoch.has_gga, epoch.sats, ',');
    printOptDouble(epoch.has_hdop, epoch.hdop, 2, ',');
    printOptDouble(epoch.has_pos, epoch.lat_deg, 8, ',');
    printOptDouble(epoch.has_pos, epoch.lon_deg, 8, ',');
    printOptDouble(epoch.has_alt, epoch.alt_m, 2, ',');
    printOptInt(epoch.has_rmc, epoch.rmc_valid ? 1 : 0, ',');
    printOptDouble(epoch.has_vel, epoch.speed_mps, 3, ',');
    printOptDouble(epoch.has_vel, epoch.course_deg, 2, ',');

    // PPS time relative to recording start (can be negative for the first
    // epoch if the edge came just before the recording started)
    if (epoch.pps_valid) {
      Serial.printf("%lld",
                    (long long)((int64_t)epoch.pps_us -
                                (int64_t)record_start_us));
    }
    Serial.println();

    gps_count++;
    if (epoch.has_pos && epoch.fix > 0) {
      gps_fix_count++;
    }
  }

  epoch.active = false;
}

// Start a new epoch for this UTC time, flushing a different unfinished one
void beginEpoch(const char* utc, uint64_t t_us) {
  if (epoch.active && strcmp(epoch.utc, utc) != 0) {
    flushEpoch();
  }
  if (epoch.active) {
    return;
  }
  memset(&epoch, 0, sizeof(epoch));
  epoch.active = true;
  epoch.t_us = t_us;
  strncpy(epoch.utc, utc, sizeof(epoch.utc) - 1);
  epoch.pps_valid = readPps(&epoch.pps_us, NULL);
}

void handleGga(char** f, int n, uint64_t t_us) {
  // $xxGGA,time,lat,N,lon,E,fix,sats,hdop,alt,M,geoid,M,...
  if (n < 10 || !f[1][0]) {
    return;
  }
  beginEpoch(f[1], t_us);
  epoch.has_gga = true;
  epoch.fix = atoi(f[6]);
  epoch.sats = atoi(f[7]);
  epoch.has_hdop = f[8][0] != '\0';
  epoch.hdop = atof(f[8]);
  epoch.has_pos = (epoch.fix > 0) &&
                  parseLatLon(f[2], f[3], &epoch.lat_deg) &&
                  parseLatLon(f[4], f[5], &epoch.lon_deg);
  epoch.has_alt = (epoch.fix > 0) && f[9][0] != '\0';
  epoch.alt_m = atof(f[9]);
}

void handleRmc(char** f, int n, uint64_t t_us) {
  // $xxRMC,time,status,lat,N,lon,E,speed_kn,course,date,...
  if (n < 10 || !f[1][0]) {
    return;
  }
  beginEpoch(f[1], t_us);
  epoch.has_rmc = true;
  epoch.rmc_valid = (f[2][0] == 'A');
  strncpy(epoch.date, f[9], sizeof(epoch.date) - 1);

  // Course is empty when stationary; report 0 so speed is still usable
  epoch.has_vel = epoch.rmc_valid && f[7][0] != '\0';
  epoch.speed_mps = atof(f[7]) * KNOTS_TO_MPS;
  epoch.course_deg = f[8][0] ? atof(f[8]) : 0.0;
}

void handleNmea(char* line, uint64_t t_us) {
  bool recording = (state == ST_RECORDING);

#if LOG_RAW_NMEA
  if (recording && t_us >= record_start_us) {
    Serial.printf("# NMEA %llu %s\n",
                  (unsigned long long)(t_us - record_start_us), line);
  }
#endif

  if (!nmeaChecksumOk(line)) {
    if (recording) {
      gps_cksum_err++;
    }
    return;
  }

  char* f[MAX_NMEA_FIELDS];
  int n = splitFields(line + 1, f, MAX_NMEA_FIELDS);

  // Sentence ID: talker (GP, GN, ...) plus type, e.g. "GNGGA"
  if (strlen(f[0]) < 5) {
    return;
  }
  const char* type = f[0] + 2;

  if (strcmp(type, "GGA") == 0) {
    handleGga(f, n, t_us);
  } else if (strcmp(type, "RMC") == 0) {
    handleRmc(f, n, t_us);
  } else {
    return;
  }

  // Epoch is complete once both sentences for this time are in
  if (epoch.active && epoch.has_gga && epoch.has_rmc) {
    flushEpoch();
  }
}

// Feed one GPS byte into the sentence buffer. t_us is when it was received.
void gpsFeed(char c, uint64_t t_us) {
  // A '$' always starts a new sentence (recovers from partial lines)
  if (c == '$') {
    nmea_len = 0;
    nmea_overflow = false;
    nmea_start_us = t_us;
    nmea_buf[nmea_len++] = c;
    return;
  }

  // Waiting for the start of a sentence (this also skips the '\n' filler
  // bytes the PA1010D returns over I2C when it has no data)
  if (nmea_len == 0) {
    return;
  }

  if (c == '\r' || c == '\n') {
    nmea_buf[nmea_len] = '\0';
    if (!nmea_overflow) {
      handleNmea(nmea_buf, nmea_start_us);
    } else if (state == ST_RECORDING) {
      gps_overflow++;
    }
    nmea_len = 0;
  } else if (nmea_len < MAX_NMEA_LEN) {
    nmea_buf[nmea_len++] = c;
  } else {
    nmea_overflow = true;
  }
}

// Read GPS bytes; runs in every state so fix status is always current
void serviceGps() {
#if GPS_USE_I2C
  // Poll at a fixed interval to keep the bus free for the BNO085
  uint64_t now = time_us_64();
  if (now < next_gps_poll_us) {
    return;
  }
  next_gps_poll_us = now + GPS_POLL_US;

  // Read chunks until the GPS returns only filler (its buffer is empty)
  for (int i = 0; i < GPS_MAX_CHUNKS; i++) {
    uint8_t n = Wire.requestFrom(GPS_I2C_ADDR, GPS_I2C_CHUNK);
    uint64_t t = time_us_64();
    bool got_data = false;
    while (Wire.available()) {
      char c = Wire.read();
      if (c != '\n') {
        got_data = true;
      }
      gpsFeed(c, t);
    }
    if (n == 0 || !got_data) {
      break;
    }
  }
#else
  while (GPS_SERIAL.available()) {
    gpsFeed(GPS_SERIAL.read(), time_us_64());
  }
#endif
}

/******************************************************************************
 * BNO085 functions
 */

// Enable accelerometer and (magnetometer-referenced) rotation vector
bool bnoEnableReports() {
  bool ok = true;
  ok &= bno.enableReport(SH2_ACCELEROMETER, ACC_INTERVAL_US);
  ok &= bno.enableReport(SH2_ROTATION_VECTOR, RV_INTERVAL_US);
  return ok;
}

// Disable all reports this logger (or a previous sketch) might have enabled
bool bnoDisableReports() {
  bool ok = true;
  ok &= bno.enableReport(SH2_ACCELEROMETER, 0);
  ok &= bno.enableReport(SH2_LINEAR_ACCELERATION, 0);
  ok &= bno.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 0);
  ok &= bno.enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, 0);
  ok &= bno.enableReport(SH2_GAME_ROTATION_VECTOR, 0);
  ok &= bno.enableReport(SH2_ROTATION_VECTOR, 0);
  return ok;
}

// Read IMU reports and print one row per accelerometer sample
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
    rv.valid = false;
    bnoEnableReports();
  }

  for (int i = 0; i < DRAIN_PER_PASS; i++) {
    // If there are no reports, stop reading
    if (!bno.getSensorEvent(&bno_val)) {
      break;
    }
    uint64_t now = time_us_64();

    // Hold the latest rotation vector (also during settling, so the first
    // rows have one)
    if (bno_val.sensorId == SH2_ROTATION_VECTOR) {
      rv.valid = true;
      rv.t_us = now;
      rv.qi = bno_val.un.rotationVector.i;
      rv.qj = bno_val.un.rotationVector.j;
      rv.qk = bno_val.un.rotationVector.k;
      rv.qr = bno_val.un.rotationVector.real;
      rv.acc_rad = bno_val.un.rotationVector.accuracy;
      rv.status = bno_val.status & 0x03;
      continue;
    }

    // Discard accelerometer reports while settling, and unexpected types
    if (state != ST_RECORDING || bno_val.sensorId != SH2_ACCELEROMETER) {
      continue;
    }

    uint64_t t = now - record_start_us;
    unsigned st = bno_val.status & 0x03;
    float ax = bno_val.un.accelerometer.x;
    float ay = bno_val.un.accelerometer.y;
    float az = bno_val.un.accelerometer.z;

    // Print row with the most recent rotation vector (empty if none yet)
    Serial.printf("%llu,IMU,%lu,%.5f,%.5f,%.5f,%u,",
                  (unsigned long long)t, (unsigned long)imu_count,
                  ax, ay, az, st);
    if (rv.valid) {
      Serial.printf("%.6f,%.6f,%.6f,%.6f,%.4f,%u,%lu\n",
                    rv.qi, rv.qj, rv.qk, rv.qr, rv.acc_rad, rv.status,
                    (unsigned long)(now - rv.t_us));
    } else {
      Serial.println(",,,,,,");
    }
    imu_count++;
  }
}

/******************************************************************************
 * Recording functions
 */

// Print header with settings and columns
void printHeader() {
  Serial.println("# BEGIN");
  Serial.printf("# CFG acc_interval_us=%d\n", ACC_INTERVAL_US);
  Serial.printf("# CFG rv_interval_us=%d\n", RV_INTERVAL_US);
  Serial.println("# CFG rv_report=rotation_vector");
  Serial.println("# CFG bno_dynamic_cal=gyro+mag");
  Serial.printf("# CFG settle_time_us=%d\n", SETTLE_TIME_US);
#if GPS_USE_I2C
  Serial.println("# CFG gps_link=i2c");
  Serial.printf("# CFG gps_poll_us=%d\n", GPS_POLL_US);
#else
  Serial.println("# CFG gps_link=uart");
  Serial.printf("# CFG gps_baud=%d\n", GPS_BAUD);
#endif
  Serial.printf("# CFG gps_rate_ms=%d\n", GPS_RATE_MS);
  Serial.printf("# CFG gps_pps_pin=%d\n", GPS_PPS_PIN);
  Serial.println("# CFG gps_sbas=1");
  Serial.println("# CFG gps_static_nav=0");
  Serial.println("# IMU: t_us,tag,seq,ax,ay,az,acc_status,"
                 "qi,qj,qk,qr,rv_acc_rad,rv_status,rv_age_us");
  Serial.println("# GPS: t_us,tag,seq,utc,date,fix,sats,hdop,"
                 "lat_deg,lon_deg,alt_m,rmc_valid,speed_mps,course_deg,"
                 "pps_t_us");
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

  // Reset counters and the held rotation vector
  imu_count = 0;
  gps_count = 0;
  gps_fix_count = 0;
  gps_cksum_err = 0;
  gps_overflow = 0;
  rv.valid = false;
  ok_flag = true;

  state_timestamp_us = time_us_64();
  state = ST_SETTLING;
}

// Stop recording and print end metadata
void stopRecording() {
  uint64_t elapsed_us = time_us_64() - record_start_us;
  uint32_t pps_now = 0;
  readPps(NULL, &pps_now);
  bnoDisableReports();
  state = ST_IDLE;
  Serial.printf("# END duration_s=%.3f imu=%lu gps=%lu gps_fix=%lu "
                "pps=%lu gps_cksum_err=%lu gps_overflow=%lu ok=%d\n",
                elapsed_us * 1e-6,
                (unsigned long)imu_count,
                (unsigned long)gps_count,
                (unsigned long)gps_fix_count,
                (unsigned long)(pps_now - pps_at_start),
                (unsigned long)gps_cksum_err,
                (unsigned long)gps_overflow,
                ok_flag ? 1 : 0);
  next_status_us = time_us_64() + IDLE_STATUS_US;
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

    // Warn (but allow) recording without a fix, e.g. for an indoor test
    if (status_fix == 0) {
      Serial.println("# WARN: no GPS fix yet");
    }

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
    readPps(NULL, &pps_at_start);
    state = ST_RECORDING;
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

// While idle, print GPS status periodically so the user can wait for a fix
void serviceIdleStatus() {
  if (state != ST_IDLE) {
    return;
  }
  uint64_t now = time_us_64();
  if (now < next_status_us) {
    return;
  }
  next_status_us = now + IDLE_STATUS_US;

  uint32_t n_pps = 0;
  readPps(NULL, &n_pps);
  Serial.printf("# GPS fix=%d sats=%d hdop=%.2f pps=%lu ready=%d\n",
                status_fix, status_sats, status_hdop, (unsigned long)n_pps,
                gpsReady() ? 1 : 0);
}

// True when the GPS has a fix that's good enough to start recording
bool gpsReady() {
  if (time_us_64() - status_us > READY_TIMEOUT_US) {
    return false;   // No recent GPS data (e.g. module unplugged)
  }
  return (status_fix == 1 || status_fix == 2) &&
         status_sats >= READY_MIN_SATS &&
         status_hdop <= READY_MAX_HDOP;   // false if hdop is NaN
}

// LED on = GPS ready
void updateLed() {
  digitalWrite(LED_BUILTIN, gpsReady() ? HIGH : LOW);
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

  // Set I2C speed (after begin(), in case it resets it)
  Wire.setClock(I2C_CLOCK_HZ);

  // Keep gyro and magnetometer calibration running (the absolute heading
  // needs a calibrated magnetometer), but turn accelerometer calibration off
  // so its bias can't shift mid-recording
  sh2_setCalConfig(SH2_CAL_GYRO | SH2_CAL_MAG);

  // Disable BNO085 reporting (for now)
  bnoDisableReports();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Configure the GPS and PPS interrupt
  gpsInit();

  // Say we're ready
  Serial.println("# commands: S <seconds> | X");
}

void loop() {
  readSerialCommand();
  serviceSettling();
  serviceGps();
  serviceBno();
  checkDuration();
  serviceIdleStatus();
  updateLed();
}
