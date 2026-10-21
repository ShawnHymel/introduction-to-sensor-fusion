/**
 * temp_logger.ino
 *
 * Serial command-driven logger for an RP2350 connected to four temperature
 * sensors: NTC thermistor, TMP36, MCP9700A, and TMP117 (reference). Logs raw
 * readings and converted temperatures so we can calibrate and fit
 * inverse-variance weights in Python.
 * Written for the earlephilhower Pico core (required for time_us_64()).
 *
 * Wiring:
 *  NTC      -> NTC_PIN (3.3V -> 10k fixed -> pin -> NTC -> GND)
 *  TMP36    -> TMP36_PIN (Vout), powered from 3.3V
 *  MCP9700A -> MCP9700_PIN (Vout), powered from 3.3V
 *  TMP117   -> STEMMA QT (I2C, default address 0x48)
 *
 * Protocol:
 *  S <seconds> [delay]  Start recording. seconds = 0 runs until stopped.
 *                       Optional delay (seconds) waits before recording
 *                       starts, e.g. to let the block settle after handling.
 *  X                    Stop the current recording
 *
 * Output:
 *  "# ..." are metadata
 *  Each recording starts with "# BEGIN" and ends with "# END"
 *  "# CFG key=value" lines record settings (no colons, so they are not
 *  mistaken for column specs)
 *  Raw analog values are 12-bit ADC counts; tmp117_raw is the signed 16-bit
 *  register value (7.8125 mC per count). "nan" means a failed reading.
 *
 * Note: the TMP117 is read directly over I2C rather than through the
 * Adafruit library, so reads never block waiting for a conversion. In its
 * default mode it converts once per second (8 averages), so each row holds
 * the most recent conversion, up to ~1 s old.
 */

#include <Wire.h>

/******************************************************************************
 * Settings
 */

#define UART_CLOCK_HZ       500000
#define SAMPLE_INTERVAL_US  1000000  // 1 Hz
#define MAX_CMD_LEN         80

// Pins
#define NTC_PIN             A0
#define TMP36_PIN           A1
#define MCP9700_PIN         A2

// ADC
#define ADC_BITS            12
#define ADC_MAX             4095.0f
#define VREF                3.3f

// NTC (Vishay NTCLE100E3103HT1)
#define NTC_R_FIXED         10000.0f  // Fixed resistor (top of divider)
#define NTC_R_NOMINAL       10000.0f  // NTC resistance at nominal temp
#define NTC_T_NOMINAL_C     25.0f
#define NTC_BETA            3977.0f   // B25/85 from datasheet

// TMP36 and MCP9700A share this transfer function
#define LIN_OFFSET_V        0.5f      // Output at 0 C
#define LIN_SLOPE_V_PER_C   0.01f     // 10 mV/C

// TMP117
#define TMP117_ADDR         0x48
#define TMP117_REG_TEMP     0x00
#define TMP117_REG_ID       0x0F
#define TMP117_LSB_C        0.0078125f
#define TMP117_STARTUP_MS   2000      // Don't record until first conversions

/******************************************************************************
 * Structs and enums
 */

// Simple state machine to determine what the logger is doing
typedef enum {
  ST_IDLE,      // Waiting for a command
  ST_WAITING,   // Waiting for the start delay and TMP117 start-up
  ST_RECORDING  // Recording data
} State;

/******************************************************************************
 * Globals
 */

State state = ST_IDLE;
bool tmp117_ok = false;
uint64_t record_start_us = 0;
uint64_t next_sample_us = 0;
uint32_t req_sec = 0;
uint32_t req_delay_ms = 0;
uint32_t wait_start_ms = 0;
uint32_t seq_count = 0;
uint32_t late_count = 0;
uint32_t i2c_err_count = 0;
char cmd_buf[MAX_CMD_LEN + 1];
uint8_t cmd_len = 0;

/******************************************************************************
 * Functions
 */

// Read a 16-bit big-endian TMP117 register. Returns false on I2C failure.
bool tmp117ReadReg(uint8_t reg, uint16_t* val) {
  Wire.beginTransmission(TMP117_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(TMP117_ADDR, 2) != 2) {
    return false;
  }
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  *val = ((uint16_t)hi << 8) | lo;
  return true;
}

// Check that a TMP117 is present (device ID low 12 bits = 0x117)
bool tmp117Probe() {
  uint16_t id;
  return tmp117ReadReg(TMP117_REG_ID, &id) && (id & 0x0FFF) == 0x0117;
}

// Convert NTC divider ADC counts to deg C (beta equation)
float ntcToC(int adc) {
  if (adc <= 0 || adc >= (int)ADC_MAX) {
    return NAN;  // Open or shorted
  }
  // NTC is the bottom leg, so R_ntc = R_fixed * adc / (max - adc).
  // Ratiometric: supply voltage cancels out.
  float r_ntc = NTC_R_FIXED * adc / (ADC_MAX - adc);
  float inv_t = 1.0f / (NTC_T_NOMINAL_C + 273.15f) +
                logf(r_ntc / NTC_R_NOMINAL) / NTC_BETA;
  return 1.0f / inv_t - 273.15f;
}

// Convert linear sensor (TMP36, MCP9700A) ADC counts to deg C
float linToC(int adc) {
  float v = adc * VREF / ADC_MAX;
  return (v - LIN_OFFSET_V) / LIN_SLOPE_V_PER_C;
}

// Print header with settings and columns
void printHeader() {
  Serial.println("# BEGIN");
  Serial.printf("# CFG uptime_s=%.1f\n", millis() * 1e-3);
  Serial.printf("# CFG sample_interval_us=%d\n", SAMPLE_INTERVAL_US);
  Serial.printf("# CFG start_delay_s=%.1f\n", req_delay_ms * 1e-3);
  Serial.printf("# CFG adc_bits=%d\n", ADC_BITS);
  Serial.printf("# CFG vref=%.3f\n", VREF);
  Serial.printf("# CFG ntc_r_fixed=%.1f\n", NTC_R_FIXED);
  Serial.printf("# CFG ntc_r_nominal=%.1f\n", NTC_R_NOMINAL);
  Serial.printf("# CFG ntc_t_nominal_c=%.2f\n", NTC_T_NOMINAL_C);
  Serial.printf("# CFG ntc_beta=%.1f\n", NTC_BETA);
  Serial.printf("# CFG lin_offset_v=%.3f\n", LIN_OFFSET_V);
  Serial.printf("# CFG lin_slope_v_per_c=%.4f\n", LIN_SLOPE_V_PER_C);
  Serial.printf("# CFG tmp117_lsb_c=%.7f\n", TMP117_LSB_C);
  Serial.printf("# CFG tmp117_mode=default_1s_8avg\n");
  Serial.printf("# CFG tmp117_ok=%d\n", tmp117_ok ? 1 : 0);
  Serial.println("# TEMP: t_us,tag,seq,ntc_raw,tmp36_raw,mcp9700_raw,"
                 "tmp117_raw,ntc_c,tmp36_c,mcp9700_c,tmp117_c");
  Serial.println("# DATA");
}

// Wait for the start delay and TMP117 start-up before recording
void startWaiting() {
  // Re-check the TMP117 in case it was plugged in after boot
  if (!tmp117_ok) {
    tmp117_ok = tmp117Probe();
    if (!tmp117_ok) {
      Serial.println("# WARNING: TMP117 not found, logging nan");
    }
  }

  wait_start_ms = millis();
  state = ST_WAITING;
  if (req_delay_ms > 0) {
    Serial.printf("# waiting %.1f s start delay\n", req_delay_ms * 1e-3);
  }
}

// Start recording data
void startRecording() {
  seq_count = 0;
  late_count = 0;
  i2c_err_count = 0;
  printHeader();
  record_start_us = time_us_64();
  next_sample_us = record_start_us;
  state = ST_RECORDING;
}

// Stop recording data
void stopRecording() {
  uint64_t elapsed_us = time_us_64() - record_start_us;
  state = ST_IDLE;
  Serial.printf("# END duration_s=%.3f temp=%lu late=%lu i2c_err=%lu\n",
                elapsed_us * 1e-6,
                (unsigned long)seq_count,
                (unsigned long)late_count,
                (unsigned long)i2c_err_count);
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

  // Parse the recording command and start waiting
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

// Start recording once the start delay has passed and the TMP117 is ready
void serviceWaiting() {
  // Do nothing if not waiting
  if (state != ST_WAITING) {
    return;
  }

  bool delay_ready = millis() - wait_start_ms >= req_delay_ms;
  bool tmp117_ready = millis() >= TMP117_STARTUP_MS;

  if (delay_ready && tmp117_ready) {
    startRecording();
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

  // Read analog sensors
  int ntc_raw = analogRead(NTC_PIN);
  int tmp36_raw = analogRead(TMP36_PIN);
  int mcp9700_raw = analogRead(MCP9700_PIN);

  // Read TMP117 (latest conversion, non-blocking)
  uint16_t reg;
  bool tmp117_read = tmp117_ok && tmp117ReadReg(TMP117_REG_TEMP, &reg);
  if (tmp117_ok && !tmp117_read) {
    i2c_err_count++;
  }

  // Format TMP117 fields ("nan" if missing)
  char tmp117_raw_str[8] = "nan";
  char tmp117_c_str[12] = "nan";
  if (tmp117_read) {
    int16_t raw = (int16_t)reg;
    snprintf(tmp117_raw_str, sizeof(tmp117_raw_str), "%d", raw);
    snprintf(tmp117_c_str, sizeof(tmp117_c_str), "%.4f", raw * TMP117_LSB_C);
  }

  // Print row (timestamp from recording start)
  uint64_t t = now - record_start_us;
  Serial.printf("%llu,TEMP,%lu,%d,%d,%d,%s,%.3f,%.3f,%.3f,%s\n",
                (unsigned long long)t, (unsigned long)seq_count,
                ntc_raw, tmp36_raw, mcp9700_raw, tmp117_raw_str,
                ntcToC(ntc_raw), linToC(tmp36_raw), linToC(mcp9700_raw),
                tmp117_c_str);
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

  // Initialize ADC and I2C
  analogReadResolution(ADC_BITS);
  Wire.begin();

  // Check for the TMP117 (recording still works without it, logging nan)
  tmp117_ok = tmp117Probe();
  if (!tmp117_ok) {
    Serial.println("# WARNING: TMP117 not found");
  }

  // Say we're ready
  Serial.println("# commands: S <seconds> [delay] | X");
}

void loop() {
  readSerialCommand();
  serviceWaiting();
  serviceSensors();
  checkDuration();
}
