# BajaDAS: Data Acquisition System for Highlander Racing

BajaDAS is a data acquisition system tailored for the Highlander Racing team's vehicles. It gathers real-time telemetry data, crucial for performance analytics and diagnostics.

## Important Note

* As of 5/20/24, there is an unresolved bug in the main code that halts the program. The program appears to get stuck after GPS setup is ran. Further testing is needed on this
* Additionally, it would be beneficial to add code to offset the "zero" point of the accelerometer/correct the readings. It is not mounted perfectly square on the car relative to ground

## Features

- **9-DOF Sensor**: Gathers acceleration, magnetic field, gyroscope, and temperature data via the `Adafruit_LSM9DS1` sensor.
- **GPS Integration**: Records GPS data, specifically latitude, longitude, fix status, and satellite count.
- **SD Card Logging**: Logs data to an SD card in CSV format.
- **Serial Logging**: Outputs sensor readings to the serial console for live monitoring.
- **CAN Bus Support**: Receives all incoming vehicle data using a TJA1051T CAN Transceiver.
- **Connection Modularity**: Screw Terminals on DAQ PCB so that additional modules can be connected to the microcontroller in the future
- **2WD/4WD State**: Receives input signals from 2WD/4WD switch about what mode the vehicle is in, and forwards this data to the dashboard via CAN-Bus
- **Fuel Sensor**: Integrates the fuel sensor and data storage as a part of the DAS

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

## GPS Information

* GPS System that tracks vehicle location (latitude and longitude) as well as reporting other information like time and date
* As of 5/13/24, the DAS software does not convert the GT-U7's GPS coordinates from NMEA (?) to standard usable coordinates

### GPS Design Goals

* CAN-Bus integration to report latitude, longitude, time, and date
* Should we use GoouuuTech GT-U7 module that we have with active antenna, or buy a new one like an M10Q and use without active antenna? Pros/Cons?
* Voltage regulator
* Custom PCB
* ESP32

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
