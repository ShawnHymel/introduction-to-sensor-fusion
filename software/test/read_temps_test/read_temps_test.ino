// Episode 2 test sketch: read NTC, TMP36, AHT20, TMP117 once per second
// Board: Adafruit Feather RP2350 (Arduino-Pico core)
// Libraries: Adafruit TMP117, Adafruit AHTX0 (install via Library Manager)
//
// Wiring:
//   NTC:   3.3V -> 10k fixed -> A0 -> NTC -> GND
//   TMP36: 3.3V, GND, Vout -> A1
//   AHT20 + TMP117 on STEMMA QT (I2C)

#include <Wire.h>
#include <Adafruit_TMP117.h>
#include <Adafruit_AHTX0.h>

// ---- Analog settings ----
const int   PIN_NTC     = A0;
const int   PIN_TMP36   = A1;
const float VREF        = 3.3;     // ADC reference voltage
const float ADC_MAX     = 4095.0;  // 12-bit

// ---- NTC parameters (check your thermistor's datasheet) ----
const float R_FIXED     = 10000.0; // fixed resistor (top of divider)
const float R_NOMINAL   = 10000.0; // NTC resistance at 25 C
const float T_NOMINAL   = 25.0;    // deg C
const float BETA        = 3977.0;  // From datasheet

const unsigned long PERIOD_MS = 1000;

Adafruit_TMP117 tmp117;
Adafruit_AHTX0  aht;
bool tmp117_ok = false;
bool aht_ok    = false;

float readNTC() {
  int adc = analogRead(PIN_NTC);
  if (adc <= 0 || adc >= ADC_MAX) return NAN;  // open or shorted
  // NTC is the bottom leg: Vout/Vin = R_ntc / (R_fixed + R_ntc)
  // Ratiometric, so supply voltage cancels out
  float r_ntc = R_FIXED * adc / (ADC_MAX - adc);
  // Beta equation: 1/T = 1/T0 + (1/B) * ln(R/R0)
  float invT = 1.0 / (T_NOMINAL + 273.15) + log(r_ntc / R_NOMINAL) / BETA;
  return 1.0 / invT - 273.15;
}

float readTMP36() {
  int adc = analogRead(PIN_TMP36);
  float v = adc * VREF / ADC_MAX;
  return (v - 0.5) * 100.0;  // 10 mV/C, 500 mV offset at 0 C
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  analogReadResolution(12);
  Wire.begin();

  tmp117_ok = tmp117.begin();
  aht_ok    = aht.begin();
  if (!tmp117_ok) Serial.println("# TMP117 not found");
  if (!aht_ok)    Serial.println("# AHT20 not found");
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last < PERIOD_MS) return;
  last += PERIOD_MS;

  float t_ntc   = readNTC();
  float t_tmp36 = readTMP36();
  float t_tmp117 = NAN;
  float t_aht    = NAN;

  sensors_event_t ev, hum;
  if (tmp117_ok && tmp117.getEvent(&ev)) t_tmp117 = ev.temperature;
  if (aht_ok && aht.getEvent(&hum, &ev)) t_aht = ev.temperature;

  // "label:value" format works in the Arduino Serial Plotter
  Serial.print("NTC:");     Serial.print(t_ntc, 2);
  Serial.print(",TMP36:");  Serial.print(t_tmp36, 2);
  Serial.print(",AHT20:");  Serial.print(t_aht, 2);
  Serial.print(",TMP117:"); Serial.println(t_tmp117, 3);
}
