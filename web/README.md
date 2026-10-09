# Adafruit WebSerial 3D Model Viewer

This is a modified version of the Adafruit WebSerial 3D Model Viewer found at https://github.com/adafruit/Adafruit_WebSerial_3DModelViewer. It has been updated to prevent the hanging issue when fed too many samples, and the model has been changed from a rabbit to an arrow (to show direction from the attached board).

## Host Locally

Navigate into this folder and run a web server. For example:

```sh
python -m http.server 8000
```

Open a browser and navigate to `http://localhost:8000`.