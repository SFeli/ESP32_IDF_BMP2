# BMP2 API

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This example shows how to use the new BOSCH - API 'BMP2' with a BMP280 - Sensor.

![Setup](/BMP2.JPG)


## How to use example

Set the Bord which you are using: e.g. 
Run `idf.py set-target esp32c3' 

### Configure the project

Open the project configuration menu (`idf.py menuconfig`). 

In the `Example Configuration` menu:

* Set the connected PINs in the configuration.
    * Set `I2C_MASTER_SCL`.
    * Set `I2C_MASTER_SDA`.

Optional: If you need, change the other options according to your requirements.

### Build and Flash

Build the project and flash it to the board, then run the monitor tool to view the serial output:

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for all the steps to configure and use the ESP-IDF to build projects.

* [ESP-IDF Getting Started Guide on ESP32](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)
* [ESP-IDF Getting Started Guide on ESP32-S2](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/index.html)
* [ESP-IDF Getting Started Guide on ESP32-C3](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/index.html)

## Example Output
Note that the output, in particular the order of the output, may vary depending on the environment.

Console output if the Sensor is connected and working:
```

