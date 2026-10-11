/**
 * bno_sd_logger.ino
 *
 * Two-core BNO085 logger that writes directly to a microSD card.
 * Core 0: serial commands, BNO085 over I2C, timestamps every report and
 *         pushes it into a cross-core queue. Never touches the SD card.
 * Core 1: owns the SD card. Formats each report as CSV into a per-stream
 *         RAM buffer and writes full 4 KB blocks to the card. SD write
 *         stalls only block core 1; the queue absorbs them.
 *
 * Written for the earlephilhower Pico core on the Adafruit Feather RP2350.
 *
 * Wiring:
 *   BNO085: STEMMA QT (I2C, GPIO2/3)
 *   SD (Adafruit 4682, SPI mode): 3V -> 3.3V, GND -> GND,
 *     CLK -> SCK (GPIO22), SI -> MO (GPIO23), SO -> MI (GPIO20),
 *     CS -> D25 (GPIO25). D1, DAT2 unconnected.
 *
 * Protocol (USB serial):
 *  S <N|G|R> <seconds> [name]  Start recording. G = game rotation vector,
 *                              R = rotation vector, N = no reference.
 *                              seconds = 0 runs until stopped. name is
 *                              optional (letters, digits, _ and -).
 *  X                           Stop the current recording.
 *
 * Output: one folder per recording, numbered so nothing is overwritten:
 *   <name>_0000/acc.csv    BNO_ACC rows
 *   <name>_0000/gyro.csv   BNO_GYRO_UC rows
 *   <name>_0000/mag.csv    BNO_MAG_UC rows
 *   <name>_0000/ref.csv    BNO_GRV or BNO_RV rows (omitted for N)
 *   <name>_0000/meta.json  settings, counts, drops, SD stats, clean flag
 * Columns match the original serial logger. t_us counts from the end of the
 * settling period (the first logged report is at ~0).
 *
 * Note: GRV and RV are mutually exclusive. The BNO085 cannot compute both at
 * the same time.
 */

#include <atomic>
#include <Adafruit_BNO08x.h>
#include <SdFat.h>
#include <pico/util/queue.h>

/******************************************************************************
 * Settings
 */

#define I2C_CLOCK_HZ        400000
#define BNO_RESET_PIN       -1
#define REPORT_INTERVAL_US  10000    // 100 Hz per report
#define SETTLE_TIME_US      200000   // Discard reports this long after start
#define DRAIN_PER_PASS      8

#define SD_CS_PIN           25
#define SD_SPI_MHZ          16       // Falls back to 4 MHz if mount fails

#define QUEUE_LEN           4096     // Records (48 B each, ~192 KB SRAM).
                                     // ~10 s of buffering at 400 records/s
#define STREAM_BUF_SIZE     4096     // Per-file write buffer (multiple of 512)
#define SLOW_WRITE_US       10000    // Count SD writes slower than this

#define STATUS_PERIOD_US    1000000  // Status line once a second
#define MAX_CMD_LEN         80
#define MAX_NAME_LEN        20
#define MAX_FOLDER_LEN      32
#define DEFAULT_NAME        "rec"

/******************************************************************************
 * Types
 */

// Reference orientation report to record alongside the raw reports
typedef enum : uint8_t {
  REF_NONE,     // No reference output
  REF_GRV,      // Game rotation vector (6-DoF fusion, no magnetometer)
  REF_RV        // Rotation vector (9-DoF fusion, includes magnetometer)
} BnoRef;

// One output file per stream
typedef enum : uint8_t {
  S_ACC,
  S_GYRO,
  S_MAG,
  S_REF,
  S_COUNT
} Stream;

// Record types passed from core 0 to core 1
typedef enum : uint8_t {
  REC_START,    // Create the folder and open the files
  REC_DATA,     // One BNO085 report
  REC_STOP      // Flush, close, and write meta.json
} RecType;

// Core 0 state machine
typedef enum : uint8_t {
  ST_IDLE,      // Waiting for a command
  ST_SETTLING,  // Discarding reports right after enabling them
  ST_RECORDING  // Logging reports
} State;

// One entry in the inter-core queue (48 bytes)
typedef struct {
  uint64_t t_us;      // MCU time since recording start
  uint32_t seq;       // Per-stream sequence number
  int32_t delay_us;   // BNO085 report delay (bno_val.timestamp)
  float v[6];         // Report values (up to 6 floats)
  uint8_t type;       // RecType
  uint8_t stream;     // Stream
  uint8_t status;     // BNO085 accuracy status (0-3)
  uint8_t sensor_id;  // SH2 sensor ID (tells GRV from RV)
} Record;

// Core 0 -> core 1, written before REC_START
typedef struct {
  char name[MAX_NAME_LEN + 1];
  BnoRef ref;
  uint32_t req_sec;
} StartInfo;

// Core 0 -> core 1, written before REC_STOP
typedef struct {
  uint64_t duration_us;
  uint32_t sampled[S_COUNT];   // Reports received from the BNO085
  uint32_t dropped[S_COUNT];   // Reports lost because the queue was full
  uint32_t max_queue;          // Queue high-water mark
  uint32_t bno_resets;         // BNO085 resets during the recording
} StopInfo;

// Core 1 -> core 0, after the recording is closed
typedef struct {
  char folder[MAX_FOLDER_LEN];
  uint32_t written[S_COUNT];   // Rows written per file
  uint32_t write_errors;       // Short or failed SD writes
  uint32_t max_write_us;       // Longest single SD write (stall size)
  uint32_t slow_writes;        // Writes slower than SLOW_WRITE_US
  uint32_t close_ms;           // Time to flush, close, and write meta.json
  bool meta_ok;                // meta.json written
  bool clean;                  // No drops, resets, or write errors
} Result;

/******************************************************************************
 * Constant tables (shared, read-only)
 */

const char* STREAM_FILES[S_COUNT] = {"acc.csv", "gyro.csv", "mag.csv", "ref.csv"};
const char* STREAM_KEYS[S_COUNT] = {"acc", "gyro", "mag", "ref"};
const char* REF_NAMES[] = {"NONE", "BNO_GRV", "BNO_RV"};

const char* HDR_ACC  = "t_us,tag,seq,bno_delay_us,status,ax,ay,az";
const char* HDR_GYRO = "t_us,tag,seq,bno_delay_us,status,gx,gy,gz,gbias_x,gbias_y,gbias_z";
const char* HDR_MAG  = "t_us,tag,seq,bno_delay_us,status,mx,my,mz,mbias_x,mbias_y,mbias_z";
const char* HDR_GRV  = "t_us,tag,seq,bno_delay_us,status,qw,qx,qy,qz";
const char* HDR_RV   = "t_us,tag,seq,bno_delay_us,status,qw,qx,qy,qz,heading_accuracy";

// Column header for a stream, given the reference setting
const char* headerFor(uint8_t s, BnoRef ref) {
  switch (s) {
    case S_ACC:  return HDR_ACC;
    case S_GYRO: return HDR_GYRO;
    case S_MAG:  return HDR_MAG;
    default:     return (ref == REF_RV) ? HDR_RV : HDR_GRV;
  }
}

/******************************************************************************
 * Shared between cores
 *
 * The atomic flags use release/acquire ordering: everything a core writes
 * before a release store is visible to the other core once its acquire load
 * sees the new value. StartInfo and StopInfo are handed over through the
 * queue, whose spin lock orders memory the same way.
 */

bool core1_separate_stack = true;

queue_t data_q;                           // Core 0 -> core 1: records
queue_t result_q;                         // Core 1 -> core 0: results
StartInfo start_info;                     // Written before REC_START
StopInfo stop_info;                       // Written before REC_STOP

std::atomic<bool> queues_ready{false};    // Core 0 -> core 1
std::atomic<int> sd_state{0};             // Core 1: 0 = pending, 1 = ok, -1 = failed
uint8_t sd_mhz = 0;                       // Published by sd_state

std::atomic<bool> files_ready{false};     // Core 1 -> core 0
bool files_ok = false;                    // Published by files_ready
char folder_name[MAX_FOLDER_LEN];         // Published by files_ready
uint32_t open_ms = 0;                     // Published by files_ready

/******************************************************************************
 * Core 0 globals
 */

Adafruit_BNO08x bno(BNO_RESET_PIN);
sh2_SensorValue_t bno_val;
State state = ST_IDLE;
BnoRef bno_ref = REF_NONE;
uint32_t req_sec = 0;
uint64_t settle_start_us = 0;
uint64_t record_start_us = 0;
uint64_t last_status_us = 0;
uint32_t seq_count[S_COUNT];
uint32_t drop_count[S_COUNT];
uint32_t max_queue = 0;
uint32_t bno_resets = 0;
char cmd_buf[MAX_CMD_LEN + 1];
uint8_t cmd_len = 0;

/******************************************************************************
 * Core 0: BNO085
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
    case REF_GRV:
      ok &= bno.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_INTERVAL_US);
      break;
    case REF_RV:
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
  ok &= bno.enableReport(SH2_ACCELEROMETER, 0);
  ok &= bno.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 0);
  ok &= bno.enableReport(SH2_MAGNETIC_FIELD_UNCALIBRATED, 0);
  ok &= bno.enableReport(SH2_GAME_ROTATION_VECTOR, 0);
  ok &= bno.enableReport(SH2_ROTATION_VECTOR, 0);
  return ok;
}

/******************************************************************************
 * Core 0: recording control
 */

// Hand a data record to core 1 without ever blocking
void pushRecord(const Record& r) {
  if (!queue_try_add(&data_q, &r)) {
    drop_count[r.stream]++;
  }
  uint32_t level = queue_get_level(&data_q);
  if (level > max_queue) {
    max_queue = level;
  }
}

// Ask core 1 to close the recording, wait for it, and print the summary
void finishRecording(uint64_t duration_us) {
  // Hand stats to core 1 (the queue's spin lock orders these writes)
  stop_info.duration_us = duration_us;
  for (int s = 0; s < S_COUNT; s++) {
    stop_info.sampled[s] = seq_count[s];
    stop_info.dropped[s] = drop_count[s];
  }
  stop_info.max_queue = max_queue;
  stop_info.bno_resets = bno_resets;

  Record r = {};
  r.type = REC_STOP;
  uint32_t t0 = millis();
  while (!queue_try_add(&data_q, &r)) {
    if (millis() - t0 >= 10000) {
      Serial.println("# ERROR: queue full for 10 s, could not stop cleanly");
      return;
    }
  }

  // Wait for core 1 to flush and close everything
  Result res;
  t0 = millis();
  uint32_t last_report = t0;
  while (!queue_try_remove(&result_q, &res)) {
    uint32_t now = millis();
    if (now - last_report >= 1000) {
      last_report = now;
      Serial.printf("# WAIT closing files, queue=%lu\n",
                    (unsigned long)queue_get_level(&data_q));
    }
    if (now - t0 >= 120000) {
      Serial.println("# ERROR: core 1 did not close the files within 120 s");
      return;
    }
  }

  uint32_t total_dropped = 0;
  for (int s = 0; s < S_COUNT; s++) {
    total_dropped += drop_count[s];
  }

  Serial.printf("# END folder=%s duration_s=%.3f acc=%lu gyro=%lu mag=%lu "
                "ref=%lu dropped=%lu max_queue=%lu/%u max_write_us=%lu "
                "slow_writes=%lu close_ms=%lu ok=%d\n",
                res.folder, duration_us * 1e-6,
                (unsigned long)res.written[S_ACC],
                (unsigned long)res.written[S_GYRO],
                (unsigned long)res.written[S_MAG],
                (unsigned long)res.written[S_REF],
                (unsigned long)total_dropped,
                (unsigned long)max_queue, QUEUE_LEN,
                (unsigned long)res.max_write_us,
                (unsigned long)res.slow_writes,
                (unsigned long)res.close_ms,
                res.clean ? 1 : 0);
}

// Open the files on core 1, then enable the BNO085 and start settling
void startRecording(const char* name, BnoRef ref, uint32_t sec) {
  if (sd_state.load(std::memory_order_acquire) != 1) {
    Serial.println("# ERROR: SD card not mounted");
    return;
  }

  // Reset counters
  for (int s = 0; s < S_COUNT; s++) {
    seq_count[s] = 0;
    drop_count[s] = 0;
  }
  max_queue = 0;
  bno_resets = 0;
  bno_ref = ref;
  req_sec = sec;

  // Ask core 1 to create the folder and open the files (the queue's spin
  // lock makes start_info visible to core 1)
  strncpy(start_info.name, name, MAX_NAME_LEN);
  start_info.name[MAX_NAME_LEN] = '\0';
  start_info.ref = ref;
  start_info.req_sec = sec;
  files_ready.store(false, std::memory_order_relaxed);

  Record r = {};
  r.type = REC_START;
  queue_add_blocking(&data_q, &r);

  // Wait until the files are open, so file setup never eats into the queue
  uint32_t t0 = millis();
  uint32_t last_report = t0;
  while (!files_ready.load(std::memory_order_acquire)) {
    uint32_t now = millis();
    if (now - last_report >= 1000) {
      last_report = now;
      Serial.println("# WAIT opening files");
    }
    if (now - t0 >= 30000) {
      Serial.println("# ERROR: core 1 did not open the files within 30 s");
      return;
    }
  }
  if (!files_ok) {
    Serial.println("# ERROR: could not create the recording folder or files");
    return;
  }

  // Enable only the requested reports
  bnoDisableReports();
  if (!bnoEnableReports(ref)) {
    Serial.println("# ERROR: could not enable BNO085 reports");
    bnoDisableReports();
    finishRecording(0);   // Close the (empty) files
    return;
  }

  Serial.printf("# BEGIN folder=%s ref=%s seconds=%lu open_ms=%lu\n",
                folder_name, REF_NAMES[ref], (unsigned long)sec,
                (unsigned long)open_ms);

  // Start the settling period
  settle_start_us = time_us_64();
  state = ST_SETTLING;
}

// Stop the BNO085 and close the recording
void stopRecording() {
  uint64_t duration_us =
      (state == ST_RECORDING) ? time_us_64() - record_start_us : 0;
  bnoDisableReports();
  state = ST_IDLE;
  finishRecording(duration_us);
}

/******************************************************************************
 * Core 0: servicing
 */

// Read reports from the BNO085 and hand them to core 1
void serviceBno() {
  if (state == ST_IDLE) {
    return;
  }

  // Check if BNO085 was reset, log warning, re-enable reports
  if (bno.wasReset()) {
    if (state == ST_RECORDING) {
      Serial.printf("# WARN: BNO085 reset at t_us=%llu, data may have a gap\n",
                    (unsigned long long)(time_us_64() - record_start_us));
      bno_resets++;
    }
    bnoEnableReports(bno_ref);
  }

  // Read a few reports per pass
  for (int i = 0; i < DRAIN_PER_PASS; i++) {
    if (!bno.getSensorEvent(&bno_val)) {
      break;
    }
    uint64_t now = time_us_64();

    // Discard reports until the settling period is over
    if (state == ST_SETTLING) {
      if (now >= settle_start_us + SETTLE_TIME_US) {
        record_start_us = now;
        last_status_us = now;
        state = ST_RECORDING;
        Serial.println("# RECORDING");
      } else {
        continue;
      }
    }

    // Common fields
    Record r;
    r.type = REC_DATA;
    r.t_us = now - record_start_us;
    r.delay_us = (int32_t)(uint32_t)bno_val.timestamp;
    r.status = bno_val.status & 0x03;
    r.sensor_id = bno_val.sensorId;

    // Report-specific fields
    switch (bno_val.sensorId) {
      case SH2_ACCELEROMETER: {
        auto& a = bno_val.un.accelerometer;
        r.stream = S_ACC;
        r.v[0] = a.x; r.v[1] = a.y; r.v[2] = a.z;
        break;
      }
      case SH2_GYROSCOPE_UNCALIBRATED: {
        auto& g = bno_val.un.gyroscopeUncal;
        r.stream = S_GYRO;
        r.v[0] = g.x; r.v[1] = g.y; r.v[2] = g.z;
        r.v[3] = g.biasX; r.v[4] = g.biasY; r.v[5] = g.biasZ;
        break;
      }
      case SH2_MAGNETIC_FIELD_UNCALIBRATED: {
        auto& m = bno_val.un.magneticFieldUncal;
        r.stream = S_MAG;
        r.v[0] = m.x; r.v[1] = m.y; r.v[2] = m.z;
        r.v[3] = m.biasX; r.v[4] = m.biasY; r.v[5] = m.biasZ;
        break;
      }
      case SH2_ROTATION_VECTOR: {
        auto& q = bno_val.un.rotationVector;
        r.stream = S_REF;
        r.v[0] = q.real; r.v[1] = q.i; r.v[2] = q.j; r.v[3] = q.k;
        r.v[4] = q.accuracy;
        break;
      }
      case SH2_GAME_ROTATION_VECTOR: {
        auto& q = bno_val.un.gameRotationVector;
        r.stream = S_REF;
        r.v[0] = q.real; r.v[1] = q.i; r.v[2] = q.j; r.v[3] = q.k;
        break;
      }
      default:
        Serial.printf("# WARN: unexpected sensorId 0x%02X\n", bno_val.sensorId);
        continue;
    }

    // Sequence numbers advance even if the record is dropped, so drops
    // show up as gaps in the file
    r.seq = seq_count[r.stream]++;
    pushRecord(r);
  }
}

// Stop if the requested duration has elapsed
void checkDuration() {
  if (state != ST_RECORDING || req_sec == 0) {
    return;
  }
  if (time_us_64() - record_start_us >= (uint64_t)req_sec * 1000000ULL) {
    stopRecording();
  }
}

// Print a status line once a second while recording
void printStatus() {
  if (state != ST_RECORDING || !Serial) {
    return;
  }
  uint64_t now = time_us_64();
  if (now - last_status_us < STATUS_PERIOD_US) {
    return;
  }
  last_status_us = now;

  uint32_t dropped = 0;
  for (int s = 0; s < S_COUNT; s++) {
    dropped += drop_count[s];
  }
  Serial.printf("# STATUS t_s=%.1f acc=%lu gyro=%lu mag=%lu ref=%lu "
                "queue=%lu max_queue=%lu dropped=%lu\n",
                (now - record_start_us) * 1e-6,
                (unsigned long)seq_count[S_ACC],
                (unsigned long)seq_count[S_GYRO],
                (unsigned long)seq_count[S_MAG],
                (unsigned long)seq_count[S_REF],
                (unsigned long)queue_get_level(&data_q),
                (unsigned long)max_queue, (unsigned long)dropped);
}

// Keep only letters, digits, '_' and '-'; fall back to the default name
void sanitizeName(const char* in, char* out) {
  int n = 0;
  for (const char* p = in; p && *p && n < MAX_NAME_LEN; p++) {
    if (isalnum((unsigned char)*p) || *p == '_' || *p == '-') {
      out[n++] = *p;
    }
  }
  out[n] = '\0';
  if (n == 0) {
    strcpy(out, DEFAULT_NAME);
  }
}

// Parse serial command and perform action
void handleCommand(char* s) {
  while (*s == ' ') {
    s++;
  }
  if (*s == '\0') {
    return;
  }

  char c = *s;

  // Stop recording
  if (c == 'X' || c == 'x') {
    if (state == ST_IDLE) {
      Serial.println("# ERROR: not recording");
    } else {
      if (state == ST_SETTLING) {
        Serial.println("# WARN: stopped during settling, no data recorded");
      }
      stopRecording();
    }
    return;
  }

  // Start recording
  if (c == 'S' || c == 's') {
    if (state != ST_IDLE) {
      Serial.println("# ERROR: already recording, send X first");
      return;
    }

    strtok(s, " ");                       // Discard the "S"
    char* tok_ref = strtok(NULL, " ");
    char* tok_sec = strtok(NULL, " ");
    char* tok_name = strtok(NULL, " ");   // Optional

    if (!tok_ref || !tok_sec) {
      Serial.println("# ERROR: usage S <N|G|R> <seconds> [name]");
      return;
    }

    BnoRef ref;
    if (*tok_ref == 'N' || *tok_ref == 'n') {
      ref = REF_NONE;
    } else if (*tok_ref == 'G' || *tok_ref == 'g') {
      ref = REF_GRV;
    } else if (*tok_ref == 'R' || *tok_ref == 'r') {
      ref = REF_RV;
    } else {
      Serial.println("# ERROR: reference must be N, G, or R");
      return;
    }

    long sec = atol(tok_sec);
    if (sec < 0) {
      Serial.println("# ERROR: seconds must be >= 0");
      return;
    }

    char name[MAX_NAME_LEN + 1];
    sanitizeName(tok_name, name);
    startRecording(name, ref, (uint32_t)sec);
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

/******************************************************************************
 * Core 0 main
 */

void setup() {
  // Queues must exist before core 1 uses them
  queue_init(&data_q, sizeof(Record), QUEUE_LEN);
  queue_init(&result_q, sizeof(Result), 1);
  queues_ready.store(true, std::memory_order_release);

  Serial.begin(115200);

  // Initialize I2C, let lines settle
  Wire.begin();
  delay(100);

  // Initialize BNO085. Keep trying if it fails.
  while (!bno.begin_I2C()) {
    Serial.println("# ERROR: could not initialize BNO085, retrying");
    delay(1000);
  }
  Wire.setClock(I2C_CLOCK_HZ);

  // Disable dynamic calibration, and all reports for now
  sh2_setCalConfig(0);
  bnoDisableReports();

  // Wait (up to 5 s) for core 1 to mount the card
  uint32_t t0 = millis();
  while (sd_state.load(std::memory_order_acquire) == 0 &&
         millis() - t0 < 5000) {
    delay(10);
  }
}

void loop() {
  // Announce once a serial connection is open
  static bool announced = false;
  if (!announced && Serial) {
    if (sd_state.load(std::memory_order_acquire) == 1) {
      Serial.printf("# READY sd_mhz=%u queue=%u free_heap=%d "
                    "commands: S <N|G|R> <seconds> [name] | X\n",
                    sd_mhz, QUEUE_LEN, rp2040.getFreeHeap());
    } else {
      Serial.println("# ERROR: SD card failed to mount");
    }
    announced = true;
  }

  readSerialCommand();
  serviceBno();
  checkDuration();
  printStatus();
}

/******************************************************************************
 * Core 1 globals
 */

SdFs sd;
FsFile files[S_COUNT];
bool active[S_COUNT];
char bufs[S_COUNT][STREAM_BUF_SIZE];
uint32_t buf_len[S_COUNT];
BnoRef cur_ref = REF_NONE;
uint32_t cur_req_sec = 0;
char cur_name[MAX_NAME_LEN + 1];
Result result;

/******************************************************************************
 * Core 1: buffered writing
 */

// Write a stream's buffer to its file, timing the write
void flushStream(uint8_t s) {
  if (buf_len[s] == 0) {
    return;
  }
  uint32_t t0 = micros();
  size_t n = files[s].write(bufs[s], buf_len[s]);
  uint32_t dt = micros() - t0;

  if (n != buf_len[s]) {
    result.write_errors++;
  }
  if (dt > result.max_write_us) {
    result.max_write_us = dt;
  }
  if (dt > SLOW_WRITE_US) {
    result.slow_writes++;
  }
  buf_len[s] = 0;
}

// Append bytes to a stream, writing full STREAM_BUF_SIZE blocks to the card
void appendBytes(uint8_t s, const char* data, uint32_t n) {
  while (n > 0) {
    uint32_t space = STREAM_BUF_SIZE - buf_len[s];
    uint32_t k = (n < space) ? n : space;
    memcpy(bufs[s] + buf_len[s], data, k);
    buf_len[s] += k;
    data += k;
    n -= k;
    if (buf_len[s] == STREAM_BUF_SIZE) {
      flushStream(s);
    }
  }
}

// Append a formatted line to a stream
void appendLine(uint8_t s, const char* fmt, ...) {
  char line[192];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  if (n > 0) {
    appendBytes(s, line, (n < (int)sizeof(line)) ? n : sizeof(line) - 1);
  }
}

// Format one report as a CSV row in the original logger's format
void writeRecord(const Record& r) {
  uint8_t s = r.stream;
  if (s >= S_COUNT || !active[s]) {
    return;
  }
  unsigned long long t = r.t_us;
  unsigned long seq = r.seq;
  long dly = r.delay_us;
  unsigned st = r.status;
  const float* v = r.v;

  switch (r.sensor_id) {
    case SH2_ACCELEROMETER:
      appendLine(s, "%llu,BNO_ACC,%lu,%ld,%u,%.5f,%.5f,%.5f\n",
                 t, seq, dly, st, v[0], v[1], v[2]);
      break;
    case SH2_GYROSCOPE_UNCALIBRATED:
      appendLine(s, "%llu,BNO_GYRO_UC,%lu,%ld,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                 t, seq, dly, st, v[0], v[1], v[2], v[3], v[4], v[5]);
      break;
    case SH2_MAGNETIC_FIELD_UNCALIBRATED:
      appendLine(s, "%llu,BNO_MAG_UC,%lu,%ld,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                 t, seq, dly, st, v[0], v[1], v[2], v[3], v[4], v[5]);
      break;
    case SH2_ROTATION_VECTOR:
      appendLine(s, "%llu,BNO_RV,%lu,%ld,%u,%.6f,%.6f,%.6f,%.6f,%.4f\n",
                 t, seq, dly, st, v[0], v[1], v[2], v[3], v[4]);
      break;
    case SH2_GAME_ROTATION_VECTOR:
      appendLine(s, "%llu,BNO_GRV,%lu,%ld,%u,%.6f,%.6f,%.6f,%.6f\n",
                 t, seq, dly, st, v[0], v[1], v[2], v[3]);
      break;
    default:
      return;
  }
  result.written[s]++;
}

/******************************************************************************
 * Core 1: recording lifecycle
 */

// Close any open files without writing anything else
void closeAll() {
  for (int s = 0; s < S_COUNT; s++) {
    if (files[s].isOpen()) {
      files[s].close();
    }
    active[s] = false;
    buf_len[s] = 0;
  }
}

// Create <name>_NNNN/ and open one CSV per active stream
void openRecording() {
  uint32_t t0 = millis();
  memset(&result, 0, sizeof(result));

  // Copy the start request (made visible by the queue's spin lock)
  memcpy(cur_name, start_info.name, sizeof(cur_name));
  cur_ref = start_info.ref;
  cur_req_sec = start_info.req_sec;

  // Find the first unused folder number
  bool ok = false;
  for (int i = 0; i < 10000; i++) {
    snprintf(result.folder, sizeof(result.folder), "%s_%04d", cur_name, i);
    if (!sd.exists(result.folder)) {
      ok = true;
      break;
    }
  }

  // Create the folder and open the files
  if (ok) {
    ok = sd.mkdir(result.folder);
  }
  for (int s = 0; s < S_COUNT && ok; s++) {
    active[s] = (s != S_REF) || (cur_ref != REF_NONE);
    buf_len[s] = 0;
    if (!active[s]) {
      continue;
    }
    char path[MAX_FOLDER_LEN + 16];
    snprintf(path, sizeof(path), "%s/%s", result.folder, STREAM_FILES[s]);
    ok = files[s].open(path, O_WRONLY | O_CREAT | O_EXCL);
    if (ok) {
      appendLine(s, "%s\n", headerFor(s, cur_ref));
    }
  }
  if (!ok) {
    closeAll();
  }

  // Publish the outcome to core 0
  memcpy(folder_name, result.folder, sizeof(folder_name));
  open_ms = millis() - t0;
  files_ok = ok;
  files_ready.store(true, std::memory_order_release);
}

// Write a JSON array of quoted column names from a CSV header string
void printColumns(FsFile& f, const char* csv) {
  f.print("[\"");
  for (const char* p = csv; *p; p++) {
    if (*p == ',') {
      f.print("\", \"");
    } else {
      f.write(*p);
    }
  }
  f.print("\"]");
}

// Write a JSON object of per-stream counts for the active streams
void printCounts(FsFile& f, const uint32_t* counts) {
  f.print("{");
  bool first = true;
  for (int s = 0; s < S_COUNT; s++) {
    if (!active[s]) {
      continue;
    }
    f.printf("%s\"%s\": %lu", first ? "" : ", ", STREAM_KEYS[s],
             (unsigned long)counts[s]);
    first = false;
  }
  f.print("}");
}

// Write meta.json describing the recording
bool writeMeta() {
  char path[MAX_FOLDER_LEN + 16];
  snprintf(path, sizeof(path), "%s/meta.json", result.folder);
  FsFile f;
  if (!f.open(path, O_WRONLY | O_CREAT | O_TRUNC)) {
    return false;
  }

  f.printf("{\n");
  f.printf("  \"name\": \"%s\",\n", cur_name);
  f.printf("  \"folder\": \"%s\",\n", result.folder);
  f.printf("  \"reference\": \"%s\",\n", REF_NAMES[cur_ref]);
  f.printf("  \"requested_seconds\": %lu,\n", (unsigned long)cur_req_sec);
  f.printf("  \"duration_s\": %.3f,\n", stop_info.duration_us * 1e-6);
  f.printf("  \"report_interval_us\": %u,\n", REPORT_INTERVAL_US);

  f.printf("  \"columns\": {");
  bool first = true;
  for (int s = 0; s < S_COUNT; s++) {
    if (!active[s]) {
      continue;
    }
    f.printf("%s\n    \"%s\": ", first ? "" : ",", STREAM_KEYS[s]);
    printColumns(f, headerFor(s, cur_ref));
    first = false;
  }
  f.printf("\n  },\n");

  f.print("  \"counts\": ");
  printCounts(f, result.written);
  f.print(",\n  \"sampled\": ");
  printCounts(f, stop_info.sampled);
  f.print(",\n  \"dropped\": ");
  printCounts(f, stop_info.dropped);
  f.print(",\n");

  f.printf("  \"bno_resets\": %lu,\n", (unsigned long)stop_info.bno_resets);
  f.printf("  \"max_queue\": %lu,\n", (unsigned long)stop_info.max_queue);
  f.printf("  \"queue_len\": %u,\n", QUEUE_LEN);
  f.printf("  \"sd_mhz\": %u,\n", sd_mhz);
  f.printf("  \"max_write_us\": %lu,\n", (unsigned long)result.max_write_us);
  f.printf("  \"slow_writes\": %lu,\n", (unsigned long)result.slow_writes);
  f.printf("  \"write_errors\": %lu,\n", (unsigned long)result.write_errors);
  f.printf("  \"clean\": %s\n", result.clean ? "true" : "false");
  f.printf("}\n");

  return f.close();
}

// Flush and close the CSVs, write meta.json, and report back to core 0
void closeRecording() {
  uint32_t t0 = millis();

  for (int s = 0; s < S_COUNT; s++) {
    if (!active[s]) {
      continue;
    }
    flushStream(s);
    if (!files[s].close()) {
      result.write_errors++;
    }
  }

  // Clean = nothing lost anywhere between the BNO085 and the card
  result.clean = (stop_info.bno_resets == 0) && (result.write_errors == 0);
  for (int s = 0; s < S_COUNT; s++) {
    if (!active[s]) {
      continue;
    }
    if (stop_info.dropped[s] != 0 ||
        stop_info.sampled[s] != result.written[s]) {
      result.clean = false;
    }
  }

  result.meta_ok = writeMeta();
  if (!result.meta_ok) {
    result.clean = false;
  }

  closeAll();
  result.close_ms = millis() - t0;
  queue_add_blocking(&result_q, &result);
}

/******************************************************************************
 * Core 1 main
 */

void setup1() {
  while (!queues_ready.load(std::memory_order_acquire)) {
  }

  if (sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(SD_SPI_MHZ)))) {
    sd_mhz = SD_SPI_MHZ;
    sd_state.store(1, std::memory_order_release);
  } else if (sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(4)))) {
    sd_mhz = 4;
    sd_state.store(1, std::memory_order_release);
  } else {
    sd_state.store(-1, std::memory_order_release);
  }
}

void loop1() {
  Record r;
  if (!queue_try_remove(&data_q, &r)) {
    return;
  }

  switch (r.type) {
    case REC_START:
      openRecording();
      break;
    case REC_DATA:
      writeRecord(r);
      break;
    case REC_STOP:
      closeRecording();
      break;
    default:
      break;
  }
}
