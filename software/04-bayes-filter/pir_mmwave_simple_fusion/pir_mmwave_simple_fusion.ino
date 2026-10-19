/**
 * PIR + mmWave simple rule-based fusion demo
 */

#include "DFRobot_C4001.h"

/******************************************************************************
 * Settings
 */

#define FUSION_INTERVAL_MS  100   // Time between checking sensors for presence
#define TIMEOUT_MS          5000  // No detection in this time means empty room
#define LED_PIN             LED_BUILTIN

// PIR settings
#define PIR_PIN             24

// mmWave settings
#define MMWAVE_ADDR         DEVICE_ADDR_0   // DEVICE_ADDR_0=0x2A, _1=0x2B
#define MMWAVE_MIN_CM       30              // Min detection distance
#define MMWAVE_MAX_CM       240             // Max detection distance
#define MMWAVE_TRIG_CM      240             // Trigger dist (240 to 2000 cm)
#define MMWAVE_TRIG_SENS    0               // 0 (min) to 9 (max)
#define MMWAVE_KEEP_SENS    0               // 0 (min) to 9 (max)
#define MMWAVE_TRIG_DELAY   0               // 0.01 s units
#define MMWAVE_KEEP_DELAY   4               // 0.5 s units (4 = 2 s, minimum)
#define MMWAVE_TIMEOUT_MS   3000            // No comms = sensor problem

/******************************************************************************
 * Globals
 */

DFRobot_C4001_UART mmwave(&Serial1, 9600);
bool fused_det;                   // True if room is considered occupied
uint32_t prev_fusion_ms = 0;      // Last fusion calculation timestamp
uint32_t prev_evidence_ms = 0;    // Last time fusion detected presence

// For parsing mmWave serial output
char mmwave_buf[64];
uint8_t mmwave_len = 0;
int mmwave_det = -1;               // -1 = unknown/stale, 0 = absent, 1 = present
uint32_t mmwave_laste_ms = 0;

/******************************************************************************
 * Functions
 */

// Reads any waiting bytes and updates mmwave_det.
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
        mmwave_laste_ms = millis();
      }
      mmwave_len = 0;

    // Add character to line buffer
    } else if (mmwave_len < sizeof(mmwave_buf) - 1) {
      mmwave_buf[mmwave_len++] = c;
    }
  }

  // Mark as unknown if the sensor goes quiet
  if (millis() - mmwave_laste_ms > MMWAVE_TIMEOUT_MS) {
    mmwave_det = -1;
  }
}

/******************************************************************************
 * Main
 */

void setup() {
  bool mmwave_ok = true;

  // Initialize LED
  pinMode(LED_PIN, OUTPUT);

  // Initialize serial and wait for a connection
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Starting PIR+mmWave simple fusion demo");

  // Initialize PIR pin (input, no pull-up/down)
  pinMode(PIR_PIN, INPUT);

  // Attempt to connect to mmWave sensor
  while(!mmwave.begin()){
    Serial.println("ERROR: mmWave sensor not found");
    delay(1000);
  }

  // Configure mmWave sensor in "exist" mode (presence detection)
  Serial.println("Configuring mmWave sensor...");
  mmwave_ok &= mmwave.setSensorMode(eExitMode);
  mmwave_ok &= mmwave.setDetectionRange(MMWAVE_MIN_CM, 
                                        MMWAVE_MAX_CM,
                                        MMWAVE_TRIG_CM);
  mmwave_ok &= mmwave.setTrigSensitivity(MMWAVE_TRIG_SENS);
  mmwave_ok &= mmwave.setKeepSensitivity(MMWAVE_KEEP_SENS);
  mmwave_ok &= mmwave.setDelay(MMWAVE_TRIG_DELAY, MMWAVE_KEEP_DELAY);
  
  // Make sure we didn't receive any errors
  if (!mmwave_ok) {
    Serial.println("ERROR: Could not configure mmWave sensor");
    while(1);
  }

  // Print mmWave settings
  sSensorStatus_t mmwave_st = mmwave.getStatus();
  Serial.printf("work status=%u (1=running), mode=%u (0=exist), init=%u\n",
                mmwave_st.workStatus, mmwave_st.workMode, mmwave_st.initStatus);

  // Flush any received mmWave serial comms
  while (Serial1.available()) {
    Serial1.read();
  }
  mmwave_laste_ms = millis();
  Serial.println("mmWave sensor configured");
}

void loop() {
  // Get current timestamp
  uint32_t timestamp = millis();

  // Service mmWave (read from serial) every loop
  serviceMmwave();

  // Run fusion on a fixed schedule
  if (timestamp - prev_fusion_ms >= FUSION_INTERVAL_MS) {
    prev_fusion_ms = timestamp;
    
    // Read from PIR sensor
    int pir_det = digitalRead(PIR_PIN);

    // Simple rule-based fusion: only PIR can trigger the flag, but the mmWave
    // detection can keep it high
    if (pir_det) {
      fused_det = true;
      prev_evidence_ms = timestamp;
    } else if (mmwave_det == 1) {
      prev_evidence_ms = timestamp;
    }

    // If neither sensor detects presence, consider room to be empty
    if (fused_det && (timestamp - prev_evidence_ms > TIMEOUT_MS)) {
      fused_det = false;
    }

    // Turn on LED based on fused detection
    if (fused_det) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
    
    // Print fusion output and raw detections 
    Serial.printf("%lu, fused det: %i PIR: %i, mmWave: %i\n", 
                  timestamp, fused_det, pir_det, mmwave_det);
  }
}
