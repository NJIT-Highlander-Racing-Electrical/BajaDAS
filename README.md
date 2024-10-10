# BajaDAS: Data Acquisition System for Highlander Racing

BajaDAS is a data acquisition system tailored for the Highlander Racing team's vehicles. It gathers real-time telemetry data, crucial for performance analytics and diagnostics.

## Important Notes + 2025 Research Topics

* It would be beneficial to add code to offset the "zero" point of the accelerometer/correct the readings. It is not mounted perfectly square on the car relative to ground. This code could also correct gyro readings based off of accelerometer orientation
* In the 2024-2025 DAQ, it would be beneficial to include a voltage divider or some other method of determining battery voltage for a low battery warning
* In the 2024-2025 DAQ, a GPS upgrade should be made to something like the Adafruit Ultimate GPS Breakout - PA1616S (https://www.adafruit.com/product/746) that can poll up to 10Hz

## Features

- **9-DOF Sensor**: Gathers acceleration, magnetic field, gyroscope, and temperature data via the `Adafruit_LSM9DS1` sensor.
- **GPS Integration**: Records GPS data, specifically latitude, longitude, fix status, and satellite count.
- **SD Card Logging**: Logs data to an SD card in CSV format.
- **Serial Logging**: Outputs sensor readings to the serial console for live monitoring.
- **CAN Bus Support**: Receives all incoming vehicle data using a TJA1051T CAN Transceiver.
- **Connection Modularity**: Screw Terminals on DAQ PCB so that additional modules can be connected to the microcontroller in the future
- **Fuel Sensor**: Integrates the fuel sensor and data storage as a part of the DAS

## Hardware Setup

- **Microcontroller**: Designed for Arduino-compatible boards with multiple hardware serial interfaces and SPI capabilities.
- **SD Card**: MicroSD card slot for data logging. Ensure it's formatted correctly.
- **Sensors**: Adafruit LSM9DS1 9-DOF sensor and a GPS module.

## GPS Information

* GPS System that tracks vehicle location (latitude and longitude) as well as reporting other information like time and date

### GPS Design Goals

* CAN-Bus integration with Adafruit Ultimate GPS module reports latitude, longitude, time, and date
* Active antenna mounted to firewall increases reception 
  
## Fuel Sensor

* The fuel sensor is integrated into the DAS for the following reasons:
     * It is physically close to the DAS and a simple circuit with few wires
     * It does not require complex math or data processing
     * A fuel reset switch can be embedded into the DAS enclosure
     * There also needs to be a DAQ recording on/off switch
     * The SD Card on the DAS can also include a file with the last known fuel level for when the vehicle is powered back on 

## Quick Start

1. Install the necessary libraries via the Arduino IDE's Library Manager or manually.
2. Connect the LSM9DS1 and GPS modules to the specified pins.
3. Insert a formatted SD card.
4. Upload the `BajaDAS` code to your microcontroller.
5. Use the Serial Monitor at a `115200` baud rate to view data.
6. Post-race, retrieve the SD card and access the `.csv` files for analysis.
