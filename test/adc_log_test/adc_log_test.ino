/**
 * BNO085 Accel, Gyro, and GRV Logger for RP2350
 *
 * Dual-core logger: core 0 samples from the sensor and core 1 writes the logs
 * to the SD card. Requires the earlephilhower Pico core on the Adafruit
 * Feather RP3250.
 *
 * SD wiring (SPI mode):
 * | Feather    | microSD card breakout |
 * |------------|-----------------------|
 * | 3.3V       | 3V                    |
 * | GND        | GND                   |
 * | SCK (GP22) | CLK                   |
 * | MO (GP23)  | SI (CMD)              |
 * | MI (GP20)  | SO (D0)               |
 * | D25 (GP25) | CS (D3)               |
 * 
 * Mostly written by Claude. Modified to make it more readable.
 */

#include <atomic>
#include <SdFat.h>
#include <pico/util/queue.h>

/******************************************************************************
 * Settings
 */

// Pins
#define ADC_PIN           A0
#define LED_PIN           LED_BUILTIN
#define SD_CS_PIN         25
#define SD_SPI_MHZ        16

// Logging configuration
#define LOG_PREFIX        "LOG"
#define MAX_LOGS          100000    // LOG00000.CSV to LOG99999.CSV
#define SAMPLE_RATE_HZ    1000
#define WAIT_LOG_S        3         // How long to wait before starting to log
#define DURATION_S        5
#define QUEUE_LEN         8192      // ~8 s of buffering at 1 kHz (128 KB SRAM)

/******************************************************************************
 * Types
 */

typedef enum : uint8_t {
  REC_SAMPLE,
  REC_STOP
} RecType;

typedef struct {
  uint64_t t_us;
  uint32_t seq;
  uint16_t adc;
  uint8_t type;
  uint8_t pad;
} Record;

typedef struct {
  uint32_t received;   // Records written by core 1
  bool failed;         // Out-of-order record, failed write, or failed close
} Result;

/******************************************************************************
 * Globals
 */

// SD card and file system variables
SdFs sd;
FsFile file;
Result result = {};
uint32_t expected_seq = 0;
char filename[16];

// Data shared between cores
bool core1_separate_stack = true;
queue_t data_q;                           // Core 0 -> core 1
queue_t result_q;                         // Core 1 -> core 0
std::atomic<bool> queues_ready{false};    // Core 0 -> core 1
std::atomic<bool> file_ready{false};      // Core 1 -> core 0
bool file_ok = false;                     // Published by file_ready

/******************************************************************************
 * Functions
 */

// Simple forever while loop that flashes the LED at a given rate
void flashErrorTerminate(uint32_t delay_ms) {
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    delay(delay_ms);
    digitalWrite(LED_PIN, LOW);
    delay(delay_ms);
  }
}

// Find the first unused LOGnnnnn.CSV name. Returns false if all are taken.
bool findNextFilename(char* name, size_t len) {
  for (uint32_t i = 0; i < MAX_LOGS; i++) {
    snprintf(name, len, LOG_PREFIX "%05lu.CSV", (unsigned long)i);
    if (!sd.exists(name)) {
      return true;
    }
  }
  return false;
}

/******************************************************************************
 * Core 0: Sample from sensor(s), send data to core 1
 */

void setup() {
  const uint32_t period_us = 1000000UL / SAMPLE_RATE_HZ;
  const uint32_t n_samples = SAMPLE_RATE_HZ * DURATION_S;
  uint32_t dropped = 0;
  uint32_t late = 0;

  // Initialize queues
  queue_init(&data_q, sizeof(Record), QUEUE_LEN);
  queue_init(&result_q, sizeof(Result), 1);
  queues_ready.store(true, std::memory_order_release);

  // Initialize LEd
  pinMode(LED_PIN, OUTPUT);

  // Initialize serial port
  Serial.begin(115200);

  // Initialize the sensor
  analogReadResolution(12);

  // Wait for core 1 to mount the card and open the file
  while (!file_ready.load(std::memory_order_acquire)) {
  }
  if (!file_ok) {
    Serial.println("ERROR: core 1 could not mount the card or open the file");
    flashErrorTerminate(100);
  }

  // Blink LED for a few seconds to alert the person that logging will begin
  Serial.printf("Starting in %i seconds...\n", WAIT_LOG_S);
  for (uint8_t i = 0; i < WAIT_LOG_S; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
  Serial.printf("Logging to %s\n", filename);

  // Turn on LED to show that we're sampling
  digitalWrite(LED_PIN, HIGH);

  // Start sampling
  uint64_t start = time_us_64();
  uint64_t next = start;
  for (uint32_t seq = 0; seq < n_samples; seq++) {
    // Wait for the sample time
    while (time_us_64() < next) {
    }
    uint64_t now = time_us_64();
    if (now - next > period_us) {
      late++;
    }
    next += period_us;

    // Sample and hand off without ever blocking
    Record r;
    r.t_us = now - start;
    r.seq = seq;
    r.adc = analogRead(ADC_PIN);
    r.type = REC_SAMPLE;
    r.pad = 0;
    if (!queue_try_add(&data_q, &r)) {
      dropped++;
    }
  }

  // Tell core 1 we're done; it drains the queue, closes the file, replies
  Record stop = {};
  stop.type = REC_STOP;
  queue_add_blocking(&data_q, &stop);
  Result res;
  queue_remove_blocking(&result_q, &res);

  // Turn off LED to show we're done recording
  digitalWrite(LED_PIN, LOW);

  // Summary
  bool pass = (dropped == 0) && (late == 0) && !res.failed &&
              (res.received == n_samples);
  if (pass) {
    Serial.println("Logging complete!");
  } else {
    Serial.printf("FAIL: dropped=%lu late=%lu received=%lu/%lu sd_failed=%d\n",
                  (unsigned long)dropped, (unsigned long)late,
                  (unsigned long)res.received, (unsigned long)n_samples,
                  res.failed ? 1 : 0);
    flashErrorTerminate(500);
  }
}

void loop() {
}

/******************************************************************************
 * Core 1: SD writer
 */

void setup1() {
  // Wait for the flag from core 0
  while (!queues_ready.load(std::memory_order_acquire)) {
  }

  // Mount (16 MHz, fall back to 4 MHz) and open, before sampling starts
  bool ok = false;
  if (sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(SD_SPI_MHZ)))) {
    ok = true;
  } else if (sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(4)))) {
    ok = true;
  }

  // Open the next free log file
  if (ok) {
    ok = findNextFilename(filename, sizeof(filename)) &&
         file.open(filename, O_WRONLY | O_CREAT | O_EXCL);
    if (ok) {
      file.println("t_us,seq,adc");
    }
  }

  // Tell core 0 we're done
  file_ok = ok;
  file_ready.store(true, std::memory_order_release);
}

void loop1() {
  Record r;

  // Get item from queue
  if (!queue_try_remove(&data_q, &r)) {
    return;
  }

  // End of run: close the file and report back
  if (r.type == REC_STOP) {
    if (!file.close()) {
      result.failed = true;
    }
    queue_add_blocking(&result_q, &result);
    return;
  }

  // Check ordering
  if (r.seq != expected_seq) {
    result.failed = true;
  }
  expected_seq = r.seq + 1;
  result.received++;

  // Write one line
  size_t n = file.printf("%llu,%lu,%u\n", (unsigned long long)r.t_us,
                         (unsigned long)r.seq, r.adc);
  if (n == 0) {
    result.failed = true;
  }
}