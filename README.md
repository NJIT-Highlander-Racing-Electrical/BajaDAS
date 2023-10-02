# BajaDAS: Data Acquisition System for Highlander Racing

BajaDAS is a data acquisition system tailored for the Highlander Racing team's vehicles. It gathers real-time telemetry data, crucial for performance analytics and diagnostics.

## Features

- **9-DOF Sensor**: Gathers acceleration, magnetic field, gyroscope, and temperature data via the `Adafruit_LSM9DS1` sensor.
- **GPS Integration**: Records GPS data, specifically latitude, longitude, fix status, and satellite count.
- **SD Card Logging**: Logs data to an SD card in CSV format.
- **Serial Logging**: Outputs sensor readings to the serial console for live monitoring.
- **CAN Bus Support**: Architecture in place for CAN bus communication (not fully implemented in the provided code).

## Dependencies

To use BajaDAS, ensure the following libraries are installed:

- `Arduino.h`
- `Wire.h`
- `FS.h`
- `SD.h`
- `SPI.h`
- `Adafruit_LSM9DS1.h`
- `HardwareSerial.h`
- `mcp_can.h`

## Hardware Setup

- **Microcontroller**: Designed for Arduino-compatible boards with multiple hardware serial interfaces and SPI capabilities.
- **SD Card**: MicroSD card slot for data logging. Ensure it's formatted correctly.
- **Sensors**: Adafruit LSM9DS1 9-DOF sensor and a GPS module.

## Quick Start

1. Install the necessary libraries via the Arduino IDE's Library Manager or manually.
2. Connect the LSM9DS1 and GPS modules to the specified pins.
3. Insert a formatted SD card.
4. Upload the `BajaDAS` code to your microcontroller.
5. Use the Serial Monitor at a `115200` baud rate to view data.
6. Post-race, retrieve the SD card and access the `.csv` files for analysis.
