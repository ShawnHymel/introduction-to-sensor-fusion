# Introduction to Sensor Fusion

Welcome to the Introduction to Sensor Fusion demo repository!

> **VIDEOS COMING SOON**

## Required Hardware

 * [Adafruit RP2350 Feather](https://www.digikey.com/en/products/detail/adafruit-industries-llc/6130/25981359)
* [Mini PIR sensor](https://www.digikey.com/en/products/detail/adafruit-industries-llc/5578/16719279)
 * [DFRobot mmWave C4001 24 GHz occupancy detector](https://www.digikey.com/en/products/detail/dfrobot/SEN0610/23028644)
 * [NTC thermistor](https://www.digikey.com/en/products/detail/vishay-beyschlag-draloric-bc-components/NTCLE100E3103HT1/2230723)
 * [TMP36 temperature sensor](https://www.digikey.com/en/products/detail/analog-devices-inc/TMP36GT9Z/820404)
 * [MCP9700 temperature sensor](https://www.digikey.com/en/products/detail/microchip-technology/MCP9700A-E-TO/1212508)
 * [TMP117 temperature sensor breakout board](https://www.digikey.com/en/products/detail/adafruit-industries-llc/4821/13571586)
 * [Adafruit BNO085 9 DoF IMU breakout board](https://www.digikey.com/en/products/detail/adafruit-industries-llc/4754/13426653)
 * [Adafruit BMP581 pressure and temperature sensor breakout board](https://www.digikey.com/en/products/detail/adafruit-industries-llc/6407/27545880)
 * 2x [Qwiic/Stemma QT connector cables](https://www.digikey.com/en/products/detail/adafruit-industries-llc/4210/10230021)
 * 10 kΩ resistor
 * 1 kΩ resistor
 * 10 kΩ potentiometer
 * Pushbutton
 * LED
 * Jumper wires
 * Breadboard

## Installation

### Arduino Dependencies

Go to *File > Preferences > Additional boards manager URLs* and add:

```
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
```

Install the following Arduino libraries (accept any pop-ups asking to install additional dependencies):

 * DFRobot C4001
 * Adafruit TMP117
 * Adafruit BNO08x
 * Adafruit BMP5xx

### Python

Download this repository somewhere on your computer. Navigate to this folder and create a virtual environment:

```sh
python -m venv venv
```

Activate the virtual environment with `venv\Scripts\activate` on Windows or `source myenv/bin/activate` on macOS or Linux.

Install the required libraries:

```sh
python -m pip install -r requirements.txt
```

Run JupyterLab:

```sh
jupyter lab
```

## License

All software in this repository, unless otherwise noted, is licensed under the [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0) license.