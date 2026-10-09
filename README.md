# Introduction to Sensor Fusion

This repository holds the code required to run all the demos from the Introduction to Sensor Fusion video series.

> Python version 3.12+ is recommended. These demos were tested on Python 3.12.6.

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

If asked, install any of the dependencies required for each of the libraries.

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

## 3D Web Viewer

This repository comes with a modified version of the [Adafruit WebSerial 3D Model Viewer]() used to show the orientation of a connected board that outputs quaternion or Euler angle information. To use the viewer, head to the following link:

[https://shawnhymel.github.io/introduction-to-sensor-fusion/](https://shawnhymel.github.io/introduction-to-sensor-fusion/)

## License

TODO