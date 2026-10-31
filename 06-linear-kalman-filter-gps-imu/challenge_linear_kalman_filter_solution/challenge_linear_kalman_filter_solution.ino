/**
 * Solution to the linear Kalman filter challenge
 */

#include <math.h>

#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP5xx.h>

/******************************************************************************
 * Settings
 */

#define UART_CLOCK_HZ           115200
#define I2C_CLOCK_HZ            400000
#define BNO_RESET_PIN           -1        // -1 for "no reset pin"
#define BNO_REPORT_INTERVAL_US  10000     // 100 Hz

/******************************************************************************
 * Globals
 */

Adafruit_BNO08x bno(BNO_RESET_PIN);
Adafruit_BMP5xx bmp;

// Kalman filter state: x = [p, v] and the error covariance P (2x2)
float x[2] = {0.0f, 0.0f};
float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
float u = 0.0f;                     // Held acceleration (input for the next step)
const float R = SIGMA_Z * SIGMA_Z;  // Measurement noise covariance (1x1)
int64_t t_prev = 0;                 // Timestamp of the previous predict step (us)
bool have_t_prev = false;           // Records if we have a previous timestep

/******************************************************************************
 * Kalman filter
 */

// Predict the state at time t (us) using the accel value from previous step
void predict(int64_t t, float a) {
  // Skip the prediction on the first timestep (no dt)
  if (have_t_prev) {
    // Calculate the difference between this and the previous timestep
    float dt = (t - t_prev) * 1e-6f;

    // TODO: Compute the state transition matrix F
    float F[2][2] = {
      {1.0f, dt},
      {0.0f, 1.0f}
    };

    // TODO: Compute the input/control matrix B
    float B[2] = {
      0.5f * dt * dt,
      dt
    };

    // TODO: Compute the noise covariance Q
    
  }

  // Hold the acceleration as the input for the next timestep
  u = a;

  // Save the previous timestamp
  t_prev = t;
  have_t_prev = true;
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
