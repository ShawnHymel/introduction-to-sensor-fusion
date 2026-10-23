/**
 * Solution: Inverse-Variance Weighting temperature sensor fusion
 *
 * Look for all the TODO tags and write the required code.
 */

#include <Adafruit_TMP117.h>

/******************************************************************************
 * Settings
 */

// Offsets (copy from Jupyter Notebook)
#define NTC_OFFSET          0.0548f
#define TMP_OFFSET          0.9011f
#define MCP_OFFSET          0.6260f

// Inverse-variance weights (copy from Jupyter Notebook)
#define NTC_WEIGHT          0.730f
#define TMP_WEIGHT          0.113f
#define MCP_WEIGHT          0.157f

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

// TMP36 and MCP9700A convert voltage to Celsius in the same way
#define LIN_OFFSET_V        0.5f      // Output at 0 C
#define LIN_SLOPE_V_PER_C   0.01f     // 10 mV/C

/******************************************************************************
 * Globals
 */

Adafruit_TMP117 tmp117;

/******************************************************************************
 * Functions
 */

// Convert NTC divider ADC to deg C (beta equation)
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

// Convert linear sensor (TMP36, MCP9700A) ADC to deg C
float linToC(int adc) {
  float v = adc * VREF / ADC_MAX;

  return (v - LIN_OFFSET_V) / LIN_SLOPE_V_PER_C;
}

/******************************************************************************
 * Main
 */

void setup() {
  // Initialize serial and wait for a connection
  Serial.begin(115200);

  // Set ADC conversion resolution (defaults to 10 bits without this)
  analogReadResolution(ADC_BITS);

  // Initialize TMP117
  while(!tmp117.begin()) {
    Serial.println("ERROR: TMP117 sensor not found");
    delay(1000);
  }
}

void loop() {
  sensors_event_t ref;
  float ivw_temp;

  // Read analog sensors
  int ntc_raw = analogRead(NTC_PIN);
  int tmp36_raw = analogRead(TMP36_PIN);
  int mcp9700_raw = analogRead(MCP9700_PIN);

  // Read reference temperature from TMP117
  tmp117.getEvent(&ref);

  // Convert NTC reading to Celsius
  float ntc_temp = ntcToC(ntc_raw);

  // Convert TMP36 and MCP9700 readings to Celsius
  float tmp_temp = linToC(tmp36_raw);
  float mcp_temp = linToC(mcp9700_raw);

  // Print raw temperature readings
  Serial.printf("NTC: %.3f C, TMP36: %.3f C, MCP9700: %.3f C\n", ntc_temp, tmp_temp, mcp_temp);
  Serial.printf("Reference (TMP117): %.3f C\n", ref.temperature);
  
  // TODO: apply offsets to each of the analog sensor readings
  ntc_temp = ntc_temp - NTC_OFFSET;
  tmp_temp = tmp_temp - TMP_OFFSET;
  mcp_temp = mcp_temp - MCP_OFFSET;

  // TODO: use inverse-variance weighting to fuse analog temperature readings
  ivw_temp = NTC_WEIGHT * ntc_temp + TMP_WEIGHT * tmp_temp + MCP_WEIGHT * mcp_temp;

  // Print inverse-variance weighted temperature
  Serial.printf("IVW fused: %.3f C\n", ivw_temp);
  Serial.println();

  delay(1000);
}