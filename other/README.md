# BajaDAS: Data Acquisition System for Highlander Racing

BajaDAS is a data acquisition system tailored for the Highlander Racing team's vehicles. It gathers real-time telemetry data, crucial for performance analytics and diagnostics.

## DAS To-Do

* Add a variable that can easily change the time zone based on GMT (e.g. Marana, AZ is GMT-7 so be able to pass in -7)
* Update CAN Driver AND line of logSD that saves the brake pressure AND line of logSerial (once we have the sensor installed) -- currently it is hard coded to just save zero
* Configure DAS Serial Output to be easily parseable by Serial Studio (for wired connections)
    * This can just be comma separated variables with a newline character at the end of the data string, everything else is parsed and processed inside of SerialStudio
    * The SerialStudio visual application will look very similar to the LoRa dashboard, but because the LoRa dashboard limits the data it sends for bandwith purposes, this DAS dashboard in SerialStudio will be more extensive.
* Ensure "zeroing" accelerometer and correcting the readings for mounting offsets is working properly.
* Ensure numbers are not getting shifted or having other erroneous digits in SD logging.
* Make sure DAS is operating at 10Hz poll rate
* Ensure DAS is ONLY receiving "SD Logging Active" bit from dashbaord
     * Ensure that transition from inactive to active creates a new file
         * If GPS has a fix, update file metadata to include correct time
         * Can also generate the name based off of time/date
         * If GPS does not have a fix, just create a log name (e.g. log3)
     * Ensure that transition from active to inactive saves the data file


## 2025-2026 Changes to Make

* Incorporate this [Adafruit Panel Mount Micro SD Card Extender](https://www.google.com/aclk?sa=l&ai=DChcSEwjj9bOK6cmKAxXBVEcBHVRjHN4YABASGgJxdQ&ae=2&aspm=1&co=1&ase=5&gclid=Cj0KCQiAvbm7BhC5ARIsAFjwNHsn8yJzOILmMZRqH4E_HPufaiggZcexFWYqg4a0y1KrF-u19AfAwZwaAi_wEALw_wcB&sig=AOD64_018L6tCDBYVmc7ekdmUu69rXhYmw&ctype=5&q=&ved=2ahUKEwjsy62K6cmKAxUiFFkFHVE7BFwQww8oAnoECAYQDA&adurl=) for easier use. This will require a waterproof screw on or press fit cover but will make removing the microSD far easier
     * I think we can also just place a microSD card slot for the other end of that panel mount directly to the PCB. It looks like the "reader" we use now just does some voltage regulation and level shifting, but since the ESP32 already uses 3.3V we may be OK without the additional hardware
     * Update the 3D Model and the PCB accordingly for these changes

* Include external pull-up resistors for any input pins on 34, 35, 36, or 39 since these do not have internal pullups

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
