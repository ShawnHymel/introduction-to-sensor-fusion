# Introduction to Sensor Fusion


## Installation

### Arduino Dependencies

Go to *File > Preferences > Additional boards manager URLs* and add:

```
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
```

Install the following Arduino libraries (accept any pop-ups asking to install additional dependencies):

 * Adafruit AHTX0
 * Adafruit TMP117
 * DFRobot C4001

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