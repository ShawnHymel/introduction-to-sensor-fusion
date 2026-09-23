# Introduction to Sensor Fusion

This repository holds the code required to run all the demos from the Introduction to Sensor Fusion video series.

## Installation

### Arduino and Libraries

Download and install the latest version of the [Arduino IDE](https://www.arduino.cc/en/software/).

In the Arduino IDE, we will install the [arduino-pico core by earlephilhower](https://github.com/earlephilhower/arduino-pico). Go to *File > Preferences*. In the *Additional boards manager URLs* list, add the following link:

```
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
```

Click OK and let the boards manager update. In the Arduino *Boards manager*, search for `raspberry pi`. Install the board definition named *Raspberry Pi Pico/RP2040/RP2350*.

Next, go to the *Library manager*. Search for and install the following libraries:

 * Adafruit BNO08x
 * Adafruit LSM6DS
 * Adafruit LIS3MDL

If asked, install any of the dependencies required for each of the libraries.

