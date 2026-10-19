/**
 * occupancy_logger.ino
 *
 * Serial command-driven logger for RP2350 connected to a BS612 PIR sensor
 * and a DFRobot C4001 mmWave sensor (UART). Logs raw detections plus a
 * ground-truth button so we can fit and test a Bayes filter in Python.
 * Written for the earlephilower Pico core (required for time_us_64()).
 *
 * Wiring:
 *  PIR REL      -> PIR_PIN (actively driven, no pull needed)
 *  PIR SENS pot -> SENS_PIN (wiper, also wired to PIR SENS)
 *  Truth button -> TRUTH_PIN to GND (internal pull-up; press toggles state)
 *  mmWave       -> Serial1 (green/T -> RX, blue/R -> TX), mode switch on UART
 *
 * Protocol:
 *  S <seconds> [delay]  Start recording. seconds = 0 runs until stopped.
 *                       Optional delay (seconds) waits before recording
 *                       starts, e.g. so you can leave the room.
 *  X                    Stop the current recording
 *
 * Output:
 *  "# ..." are metadata
 *  Each recording starts with "# BEGIN" and ends with "# END"
 *  "# CFG key=value" lines record settings (no colons, so they are not
 *  mistaken for column specs)
 *  mmwave = -1 means no recent data from the sensor (treat as missing)
 *
 * Start-up:
 *  After S, the logger waits for the optional start delay, a fresh mmWave
 *  presence line, and the PIR warm-up period (time since boot) before
 *  printing "# BEGIN".
 *
 * Ground truth:
 *  Press the button to toggle "present" / "absent". The onboard LED shows
 *  the current truth state (on = present).
 *
 * Note: the DFRobot library is only used to configure the sensor in setup().
 * Its UART motionDetection() drops most readings, so we parse the sensor's
 * "$DFHPD" presence lines ourselves.
 */

#include "DFRobot_C4001.h"

/******************************************************************************
 * Settings
 */

#define UART_CLOCK_HZ       500000
#define SAMPLE_INTERVAL_US  100000   // 10 Hz
#define MAX_CMD_LEN         80

// Pins
#define PIR_PIN             24
#define TRUTH_PIN           25
#define SENS_PIN            A0
#define TRUTH_LED_PIN       LED_BUILTIN
#define DEBOUNCE_US         30000

// Start-up
#define PIR_WARMUP_MS       30000    // Don't record until this long after boot
#define MMWAVE_WAIT_MS      10000    // Give up if no mmWave line in this long

// mmWave settings
#define MMWAVE_BAUD         9600     // Fixed by the sensor
#define MMWAVE_MIN_CM       30       // Min detection distance
#define MMWAVE_MAX_CM       240      // Max detection distance
#define MMWAVE_TRIG_CM      240      // Trigger distance (240 to 2000 cm)
#define MMWAVE_TRIG_SENS    4        // 0 (min) to 9 (max)
#define MMWAVE_KEEP_SENS    0        // 0 (min) to 9 (max)
#define MMWAVE_TRIG_DELAY   0        // 0.01 s units
#define MMWAVE_KEEP_DELAY   4        // 0.5 s units (4 = 2 s, minimum)
#define MMWAVE_TIMEOUT_MS   3000     // No presence line in this long = stale
#define MMWAVE_BUF_LEN      64

/******************************************************************************
 * Structs and enums
 */

// Simple state machine to determine what the logger is doing
typedef enum {
  ST_IDLE,      // Waiting for a command
  ST_WAITING,   // Waiting for fresh mmWave data and PIR warm-up
  ST_RECORDING  // Recording data
} State;

// mmWave settings read back from the sensor in setup()
typedef struct {
  uint16_t min_cm;
  uint16_t max_cm;
  uint16_t trig_cm;
  uint8_t trig_sens;
  uint8_t keep_sens;
  uint8_t trig_delay;
  uint16_t keep_timeout;
} MmwaveCfg;

/******************************************************************************
 * Globals
 */

DFRobot_C4001_UART mmwave(&Serial1, MMWAVE_BAUD);
MmwaveCfg mmwave_cfg;
State state = ST_IDLE;
uint64_t record_start_us = 0;
uint64_t next_sample_us = 0;
uint32_t req_sec = 0;
uint32_t req_delay_ms = 0;
uint32_t seq_count = 0;
uint32_t late_count = 0;
uint32_t stale_count = 0;
char cmd_buf[MAX_CMD_LEN + 1];
uint8_t cmd_len = 0;

// Waiting state
uint32_t wait_start_ms = 0;
uint32_t wait_start_line = 0;

// For parsing mmWave serial output
char mmwave_buf[MMWAVE_BUF_LEN];
uint8_t mmwave_len = 0;
int mmwave_det = -1;          // -1 = unknown/stale, 0 = absent, 1 = present
uint32_t mmwave_last_ms = 0;  // Time of last presence line
uint32_t mmwave_lines = 0;    // Number of presence lines received

// Ground-truth button state
bool truth = false;
bool btn_prev = true;         // Pulled up, so idle = HIGH
uint64_t btn_change_us = 0;

/******************************************************************************
 * Functions
 */

// Configure the mmWave sensor and read the settings back (blocking, slow)
bool mmwaveConfigure() {
  bool ok = true;

  ok &= mmwave.setSensorMode(eExitMode);
  ok &= mmwave.setDetectionRange(MMWAVE_MIN_CM, MMWAVE_MAX_CM, MMWAVE_TRIG_CM);
  ok &= mmwave.setTrigSensitivity(MMWAVE_TRIG_SENS);
  ok &= mmwave.setKeepSensitivity(MMWAVE_KEEP_SENS);
  ok &= mmwave.setDelay(MMWAVE_TRIG_DELAY, MMWAVE_KEEP_DELAY);

  // Read back now. Each getter stops/restarts the sensor, so never call them
  // during a recording. Ranges read back rounded to whole meters.
  mmwave_cfg.min_cm = mmwave.getMinRange();
  mmwave_cfg.max_cm = mmwave.getMaxRange();
  mmwave_cfg.trig_cm = mmwave.getTrigRange();
  mmwave_cfg.trig_sens = mmwave.getTrigSensitivity();
  mmwave_cfg.keep_sens = mmwave.getKeepSensitivity();
  mmwave_cfg.trig_delay = mmwave.getTrigDelay();
  mmwave_cfg.keep_timeout = mmwave.getKeepTimerout();

  return ok;
}

// Read any waiting mmWave bytes and update mmwave_det
void serviceMmwave() {
  while (Serial1.available()) {
    // Read from serial
    char c = Serial1.read();

    // Parse line
    if (c == '\n' || c == '\r') {
      mmwave_buf[mmwave_len] = '\0';

      // Presence line looks like "$DFHPD,1,..." or "$DFHPD,0,..."
      if (mmwave_len >= 8 && strncmp(mmwave_buf, "$DFHPD,", 7) == 0) {
        mmwave_det = (mmwave_buf[7] == '1') ? 1 : 0;
        mmwave_last_ms = millis();
        mmwave_lines++;
      }
      mmwave_len = 0;

    // Add character to line buffer
    } else if (mmwave_len < MMWAVE_BUF_LEN - 1) {
      mmwave_buf[mmwave_len++] = c;
    }
  }

  // Mark as unknown if the sensor goes quiet
  if (millis() - mmwave_last_ms > MMWAVE_TIMEOUT_MS) {
    mmwave_det = -1;
  }
}

// Poll the ground-truth button and toggle the truth state on each press
void serviceButton() {
  bool btn = digitalRead(TRUTH_PIN);
  uint64_t now = time_us_64();

  // Ignore changes during the debounce window
  if (btn != btn_prev && now - btn_change_us > DEBOUNCE_US) {
    btn_change_us = now;
    btn_prev = btn;

    // Toggle on press (falling edge, since the button pulls to ground)
    if (!btn) {
      truth = !truth;
      digitalWrite(TRUTH_LED_PIN, truth ? HIGH : LOW);
    }
  }
}

// Print header with settings and columns
void printHeader() {
  Serial.println("# BEGIN");
  Serial.printf("# CFG uptime_s=%.1f\n", millis() * 1e-3);
  Serial.printf("# CFG sample_interval_us=%d\n", SAMPLE_INTERVAL_US);
  Serial.printf("# CFG start_delay_s=%.1f\n", req_delay_ms * 1e-3);
  Serial.printf("# CFG mmwave_link=uart\n");
  Serial.printf("# CFG mmwave_timeout_ms=%d\n", MMWAVE_TIMEOUT_MS);

  // Requested settings
  Serial.printf("# CFG mmwave_min_cm=%d\n", MMWAVE_MIN_CM);
  Serial.printf("# CFG mmwave_max_cm=%d\n", MMWAVE_MAX_CM);
  Serial.printf("# CFG mmwave_trig_cm=%d\n", MMWAVE_TRIG_CM);
  Serial.printf("# CFG mmwave_trig_sens=%d\n", MMWAVE_TRIG_SENS);
  Serial.printf("# CFG mmwave_keep_sens=%d\n", MMWAVE_KEEP_SENS);
  Serial.printf("# CFG mmwave_trig_delay=%d\n", MMWAVE_TRIG_DELAY);
  Serial.printf("# CFG mmwave_keep_delay=%d\n", MMWAVE_KEEP_DELAY);

  // Settings read back from the sensor in setup()
  Serial.printf("# CFG rb_mmwave_min_cm=%u\n", mmwave_cfg.min_cm);
  Serial.printf("# CFG rb_mmwave_max_cm=%u\n", mmwave_cfg.max_cm);
  Serial.printf("# CFG rb_mmwave_trig_cm=%u\n", mmwave_cfg.trig_cm);
  Serial.printf("# CFG rb_mmwave_trig_sens=%u\n", mmwave_cfg.trig_sens);
  Serial.printf("# CFG rb_mmwave_keep_sens=%u\n", mmwave_cfg.keep_sens);
  Serial.printf("# CFG rb_mmwave_trig_delay=%u\n", mmwave_cfg.trig_delay);
  Serial.printf("# CFG rb_mmwave_keep_delay=%u\n", mmwave_cfg.keep_timeout);

  Serial.printf("# CFG truth_initial=%d\n", truth ? 1 : 0);
  Serial.println("# OCC: t_us,tag,seq,pir,mmwave,truth,sens_raw");
  Serial.println("# DATA");
}

// Wait for fresh mmWave data and PIR warm-up before recording
void startWaiting() {
  wait_start_ms = millis();
  wait_start_line = mmwave_lines;
  state = ST_WAITING;
  if (req_delay_ms > 0) {
    Serial.printf("# waiting %.1f s start delay\n", req_delay_ms * 1e-3);
  }
  if (millis() < PIR_WARMUP_MS) {
    Serial.printf("# waiting %.1f s for PIR warm-up\n",
                  (PIR_WARMUP_MS - millis()) * 1e-3);
  }
}

// Start recording data
void startRecording() {
  seq_count = 0;
  late_count = 0;
  stale_count = 0;
  printHeader();
  record_start_us = time_us_64();
  next_sample_us = record_start_us;
  state = ST_RECORDING;
}

// Stop recording data
void stopRecording() {
  uint64_t elapsed_us = time_us_64() - record_start_us;
  state = ST_IDLE;
  Serial.printf("# END duration_s=%.3f occ=%lu late=%lu stale=%lu\n",
                elapsed_us * 1e-6,
                (unsigned long)seq_count,
                (unsigned long)late_count,
                (unsigned long)stale_count);
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
      case ST_WAITING:
        state = ST_IDLE;
        Serial.println("# ERROR: stopped while waiting, no data recorded");
        break;
      default:
        stopRecording();
        break;
    }
    return;
  }

  // Parse the recording command and start waiting for good data
  if (c == 'S' || c == 's') {
    // Throw an error if we're already recording
    if (state != ST_IDLE) {
      Serial.println("# ERROR: already recording, send X first");
      return;
    }

    // Discard the "S" and get the duration and optional start delay
    strtok(s, " ");
    char* tok_sec = strtok(NULL, " ");
    char* tok_delay = strtok(NULL, " ");

    // Notify user if wrong syntax
    if (!tok_sec) {
      Serial.println("# ERROR: usage S <seconds> [delay]");
      return;
    }

    // Set length of time to log
    long sec = atol(tok_sec);
    if (sec < 0) {
      Serial.println("# ERROR: seconds must be >= 0");
      return;
    }

    // Set optional start delay
    long delay_s = tok_delay ? atol(tok_delay) : 0;
    if (delay_s < 0) {
      Serial.println("# ERROR: delay must be >= 0");
      return;
    }

    req_sec = (uint32_t)sec;
    req_delay_ms = (uint32_t)delay_s * 1000UL;

    // Wait for good data before recording
    startWaiting();

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

// Start recording once the mmWave is sending fresh data and the PIR is warm
void serviceWaiting() {
  // Do nothing if not waiting
  if (state != ST_WAITING) {
    return;
  }

  // Need the start delay to pass, a new presence line since the S command,
  // and PIR warmed up
  uint32_t delay_end_ms = wait_start_ms + req_delay_ms;
  bool delay_ready = millis() - wait_start_ms >= req_delay_ms;
  bool mmwave_ready = (mmwave_lines > wait_start_line) && (mmwave_det != -1);
  bool pir_ready = millis() >= PIR_WARMUP_MS;

  // Start recording if ready
  if (delay_ready && mmwave_ready && pir_ready) {
    startRecording();
    return;
  }

  // Give up if the mmWave never sends a fresh line (delay and PIR warm-up
  // excluded)
  if (delay_ready && pir_ready && !mmwave_ready &&
      millis() - max(delay_end_ms, (uint32_t)PIR_WARMUP_MS) > MMWAVE_WAIT_MS) {
    Serial.println("# ERROR: no data from mmWave sensor");
    state = ST_IDLE;
  }
}

// Read sensors and print one row on a fixed schedule
void serviceSensors() {
  // Do nothing if not recording
  if (state != ST_RECORDING) {
    return;
  }

  // Wait for the next sample time
  uint64_t now = time_us_64();
  if (now < next_sample_us) {
    return;
  }

  // Count samples that started more than one interval late (missed a slot)
  if (now - next_sample_us >= SAMPLE_INTERVAL_US) {
    late_count++;
  }
  next_sample_us += SAMPLE_INTERVAL_US;

  // Read sensors (mmwave_det is kept current by serviceMmwave())
  int pir = digitalRead(PIR_PIN);
  int sens_raw = analogRead(SENS_PIN);
  if (mmwave_det == -1) {
    stale_count++;
  }

  // Print row (timestamp from recording start)
  uint64_t t = now - record_start_us;
  Serial.printf("%llu,OCC,%lu,%d,%d,%d,%d\n",
                (unsigned long long)t, (unsigned long)seq_count,
                pir, mmwave_det, truth ? 1 : 0, sens_raw);
  seq_count++;
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

  // Initialize pins (no internal pull-downs: RP2350 erratum E9)
  pinMode(PIR_PIN, INPUT);
  pinMode(TRUTH_PIN, INPUT_PULLUP);
  pinMode(TRUTH_LED_PIN, OUTPUT);
  digitalWrite(TRUTH_LED_PIN, LOW);
  analogReadResolution(12);

  // Initialize mmWave UART (always succeeds; the setters below block
  // forever if the sensor doesn't respond, so check wiring if stuck here)
  mmwave.begin();

  // Configure mmWave sensor. Halt if it fails.
  if (!mmwaveConfigure()) {
    while (1) {
      Serial.println("# ERROR: could not configure mmWave sensor");
      delay(1000);
    }
  }

  // Flush leftover command responses before parsing presence lines
  while (Serial1.available()) {
    Serial1.read();
  }
  mmwave_last_ms = millis();

  // Say we're ready
  Serial.println("# commands: S <seconds> [delay] | X");
}

void loop() {
  serviceMmwave();      // Always, so we never parse old buffered data
  readSerialCommand();
  serviceButton();
  serviceWaiting();
  serviceSensors();
  checkDuration();
}
