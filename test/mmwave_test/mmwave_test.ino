#include "DFRobot_C4001.h"

DFRobot_C4001_I2C radar(&Wire ,DEVICE_ADDR_0);

void setup() {
  bool radar_ok = true;

  // Initialize serial and wait for a connection
  Serial.begin(115200);
  while(!Serial);

  // Attempt to connect to mmWave sensor
  while(!radar.begin()){
    Serial.println("ERROR: mmWave sensor not found");
    delay(1000);
  }

  // Configure mmWave sensor
  radar_ok &= radar.setSensorMode(eExitMode);         // "Exist" mode (presence detection)
  radar_ok &= radar.setDetectionRange(30, 240, 150);  // Min, max, trig (cm)
  radar_ok &= radar.setTrigSensitivity(0);            // 0 (min) to 9 (max)
  radar_ok &= radar.setKeepSensitivity(0);            // 0 (min) to 9 (max)
  radar_ok &= radar.setDelay(0, 4);                   // trigger delay (0.01 s units), keep delay (0.5 s units)
  
  // Make sure we didn't receive any errors
  if (!radar_ok) {
    Serial.println("ERROR: Could not configure mmWave sensor");
    while(1);
  }
  Serial.println("mmWave sensor configured");
}

void loop() {
  static int a = 0;

  // Determine whether the object is moving
  if(radar.motionDetection()){
    Serial.printf("Motion %i\n", a);
    a++;
  }
  delay(100);
}
