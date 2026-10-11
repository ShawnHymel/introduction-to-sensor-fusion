/**
 * step3_dual_core_sd.ino
 *
 * Step 3 of building the logger: two cores with real SD writes.
 * Same structure as step 2, but core 1 writes each record to STEP3.CSV
 * instead of faking stalls. No preallocation. Core 1 opens the file BEFORE
 * core 0 starts sampling, and closes it after sampling ends.
 *
 * Diagnostics: core 0 logs when each late sample happened; core 1 logs when
 * each slow SD write happened. Both use the same hardware timer, so if they
 * line up, SD activity on core 1 is delaying core 0.
 *
 * Runs once after the Serial Monitor connects.
 *
 * SD wiring (Adafruit 4682, SPI mode):
 *   3V -> 3.3V, GND -> GND, CLK -> SCK (GPIO22), SI -> MO (GPIO23),
 *   SO -> MI (GPIO20), CS -> D25 (GPIO25). D1, DAT2 unconnected.
 */

#include <atomic>
#include <SdFat.h>
#include <pico/util/queue.h>

/******************************************************************************
 * Settings
 */

#define SD_CS_PIN         25
#define SD_SPI_MHZ        16
#define FILENAME          "STEP3.CSV"
#define ADC_PIN           A0
#define SAMPLE_RATE_HZ    1000
#define DURATION_S        10
#define QUEUE_LEN         8192     // ~8 s of buffering at 1 kHz (128 KB SRAM)
#define SLOW_WRITE_US     2000     // Core 1 logs writes slower than this
#define MAX_EVENTS        16       // Events logged per core

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
  uint32_t received;      // Sample records popped by core 1
  uint32_t seq_errors;    // Records that arrived out of sequence
  uint32_t failed_writes; // file.printf() calls that reported failure
  uint32_t max_write_us;  // Longest single file.printf() (stall size)
  uint32_t slow_writes;   // Writes slower than SLOW_WRITE_US
  uint32_t close_us;
  bool close_ok;
  // When the slow writes happened (absolute time_us_64) and how long
  uint32_t n_events;
  uint64_t event_t_us[MAX_EVENTS];
  uint32_t event_dur_us[MAX_EVENTS];
} Result;

/******************************************************************************
 * Shared between cores
 *
 * The atomic flags use release/acquire ordering: everything a core writes
 * before a release store is visible to the other core once its acquire load
 * sees the new value. So the plain variables below need no volatile.
 */

bool core1_separate_stack = true;
queue_t data_q;                           // Core 0 -> core 1
queue_t result_q;                         // Core 1 -> core 0
std::atomic<bool> queues_ready{false};    // Core 0 -> core 1
std::atomic<bool> file_ready{false};      // Core 1 -> core 0
bool file_ok = false;                     // Published by file_ready
uint8_t sd_mhz = 0;                       // Published by file_ready
uint32_t open_us = 0;                     // Published by file_ready

/******************************************************************************
 * Core 0: sampling
 */

void setup() {
  queue_init(&data_q, sizeof(Record), QUEUE_LEN);
  queue_init(&result_q, sizeof(Result), 1);
  queues_ready.store(true, std::memory_order_release);

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  analogReadResolution(12);
  Serial.printf("Step 3: %u Hz for %u s, real SD writes to %s\n",
                SAMPLE_RATE_HZ, DURATION_S, FILENAME);

  // Wait for core 1 to mount the card and open the file
  while (!file_ready.load(std::memory_order_acquire)) {
  }
  if (!file_ok) {
    Serial.println("ERROR: core 1 could not mount the card or open the file");
    return;
  }
  Serial.printf("SD at %u MHz, open_us=%lu\n", sd_mhz, (unsigned long)open_us);

  const uint32_t period_us = 1000000UL / SAMPLE_RATE_HZ;
  const uint32_t n_samples = SAMPLE_RATE_HZ * DURATION_S;
  uint32_t dropped = 0, late = 0, max_queue = 0;
  uint32_t n_late_events = 0;
  uint64_t late_t_us[MAX_EVENTS];
  uint32_t late_by_us[MAX_EVENTS];
  uint64_t start = time_us_64();
  uint64_t next = start;

  for (uint32_t seq = 0; seq < n_samples; seq++) {
    // Wait for the sample time
    while (time_us_64() < next) {
    }
    uint64_t now = time_us_64();
    if (now - next > period_us) {
      late++;
      if (n_late_events < MAX_EVENTS) {
        late_t_us[n_late_events] = now;
        late_by_us[n_late_events] = (uint32_t)(now - next);
        n_late_events++;
      }
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

    uint32_t level = queue_get_level(&data_q);
    if (level > max_queue) {
      max_queue = level;
    }
  }

  // Tell core 1 we're done; it drains the queue, closes the file, replies
  Record stop = {};
  stop.type = REC_STOP;
  queue_add_blocking(&data_q, &stop);
  Result res;
  queue_remove_blocking(&result_q, &res);

  // Summary
  Serial.printf("core 0: sampled=%lu dropped=%lu late=%lu max_queue=%lu/%u\n",
                (unsigned long)n_samples, (unsigned long)dropped,
                (unsigned long)late, (unsigned long)max_queue, QUEUE_LEN);
  Serial.printf("core 1: received=%lu seq_errors=%lu failed_writes=%lu "
                "max_write_us=%lu slow_writes(>%u us)=%lu close_us=%lu "
                "close_ok=%d\n",
                (unsigned long)res.received, (unsigned long)res.seq_errors,
                (unsigned long)res.failed_writes,
                (unsigned long)res.max_write_us, SLOW_WRITE_US,
                (unsigned long)res.slow_writes, (unsigned long)res.close_us,
                res.close_ok ? 1 : 0);

  // Timelines, in ms since sampling started
  Serial.println("core 0 late samples (t_ms, late_by_us):");
  for (uint32_t i = 0; i < n_late_events; i++) {
    Serial.printf("  %9.3f  %lu\n", (late_t_us[i] - start) * 1e-3,
                  (unsigned long)late_by_us[i]);
  }
  Serial.println("core 1 slow writes (t_ms, dur_us):");
  for (uint32_t i = 0; i < res.n_events; i++) {
    Serial.printf("  %9.3f  %lu\n", ((int64_t)(res.event_t_us[i] - start)) * 1e-3,
                  (unsigned long)res.event_dur_us[i]);
  }

  // Verdict, with the reason(s) if it failed
  bool pass = true;
  if (dropped)                     { Serial.println("FAIL: samples dropped (queue full)"); pass = false; }
  if (late)                        { Serial.println("FAIL: samples taken late (core 0 delayed)"); pass = false; }
  if (res.seq_errors)              { Serial.println("FAIL: records out of sequence"); pass = false; }
  if (res.received != n_samples)   { Serial.println("FAIL: core 1 received the wrong count"); pass = false; }
  if (res.failed_writes)           { Serial.println("FAIL: SD writes failed"); pass = false; }
  if (!res.close_ok)               { Serial.println("FAIL: file close failed"); pass = false; }
  if (pass) {
    Serial.println("PASS");
  }
}

void loop() {
}

/******************************************************************************
 * Core 1: SD writer
 */

SdFs sd;
FsFile file;
Result result = {};
uint32_t expected_seq = 0;

void setup1() {
  while (!queues_ready.load(std::memory_order_acquire)) {
  }

  // Mount (16 MHz, fall back to 4 MHz) and open, before sampling starts
  bool ok = false;
  if (sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(SD_SPI_MHZ)))) {
    sd_mhz = SD_SPI_MHZ;
    ok = true;
  } else if (sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(4)))) {
    sd_mhz = 4;
    ok = true;
  }

  if (ok) {
    uint32_t t0 = micros();
    ok = file.open(FILENAME, O_WRONLY | O_CREAT | O_TRUNC);
    open_us = micros() - t0;
    if (ok) {
      file.println("t_us,seq,adc");
    }
  }

  // Publish file_ok, sd_mhz, and open_us to core 0
  file_ok = ok;
  file_ready.store(true, std::memory_order_release);
}

void loop1() {
  Record r;
  if (!queue_try_remove(&data_q, &r)) {
    return;
  }

  // End of run: close the file and report back
  if (r.type == REC_STOP) {
    uint32_t t0 = micros();
    result.close_ok = file.close();
    result.close_us = micros() - t0;
    queue_add_blocking(&result_q, &result);
    return;
  }

  // Check ordering
  if (r.seq != expected_seq) {
    result.seq_errors++;
  }
  expected_seq = r.seq + 1;
  result.received++;

  // Write one line, timing it
  uint64_t t0 = time_us_64();
  size_t n = file.printf("%llu,%lu,%u\n", (unsigned long long)r.t_us,
                         (unsigned long)r.seq, r.adc);
  uint32_t dt = (uint32_t)(time_us_64() - t0);

  if (n == 0) {
    result.failed_writes++;
  }
  if (dt > result.max_write_us) {
    result.max_write_us = dt;
  }
  if (dt > SLOW_WRITE_US) {
    result.slow_writes++;
    if (result.n_events < MAX_EVENTS) {
      result.event_t_us[result.n_events] = t0;
      result.event_dur_us[result.n_events] = dt;
      result.n_events++;
    }
  }
}
