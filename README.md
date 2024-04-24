# INA 226 Test

A very simple driver for the INA 226 current sensor.

### Config
ESP-IDF version 4.4

Use the `menuconfig` to set the `SDA/SCL` pins and that should be everything needed.

### Running
Replace `/dev/ttyACM0` with the correct port:

    idf.py flash monitor -p /dev/ttyACM0