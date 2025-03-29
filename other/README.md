# BajaDAS: Data Acquisition System for Highlander Racing

BajaDAS is a data acquisition system tailored for the Highlander Racing team's vehicles. It gathers real-time telemetry data, crucial for performance analytics and diagnostics.

## Andrew's Winter Break Updates

* I had some issues with the SD card where I would get the message that "The physical drive cannot work." It turned out to be a bad reader, there was a missing capacitor on the board
  
* Pins 34, 35, 36, 39 do not have internal pullups but we have buttons attahed to 34 and 35. Since buying a whole new PCB for such a minor issue is a royal PITA, I soldered two pullup resistors on the underside of the PCB on the ESP32 from 3.3V to 34 and 35

* The Ultimate GPS Module was not communicating properly with the ESP32 over Hardware Serial 2
    * Originally, I thought this was because the module was given 5V on VIN and was not operating at the proper logic level.
    * However, I found that it does work at either voltage due to the following:
        *  Adafruit says the GPS TX pin is 3.3V logic level (seemingly regardless of VIN voltage)
        *  On GPS RX, "You can use use 3.3V or 5V logic, there is a logic level shifter."
    * I was able to get communication working by reading the data directly to serial without adafruit's parsing stuff: Serial2.begin(9600, SERIAL_8N1, 16, 17);
    * It seems that the Adafruit code does not properly initialize the Serial2 port for reading data
    * The AdafruitGPSTestSketch uplaoded in this repo works to receive and parse data and is a good starting point for the full DAS program

## Important Notes + 2025 Research Topics

* It would be beneficial to add code to offset the "zero" point of the accelerometer/correct the readings. It is not mounted perfectly square on the car relative to ground. This code could also correct gyro readings based off of accelerometer orientation

* Occassionaly, numbers will get shifted or have other erroneous digits added to them in the SD logging. Maybe a buffer should be added that saves every value to that so that no values are misprinted to the log.

* Make sure that the DAS this year saves more of the GPS data. Heading, velocity, elevation, etc

* Make sure that the DAS uses the voltage divider onboard for battery level measurement
    * This will have to be calibrated manually by measuring battery voltage

* Make sure that the GPS is reporting new data at a 10Hz rate

* Make sure DAS transmits vehicle velocity to CAN Bus (for wheel speed sensors slip/skid detection)

* Also make it very easy to switch timezone offsets for GPS time (we'll be in Arizona)
* Make it automatically change time zones based on longitude?

* Transfer Data Logging switch from DAS to Dashboard buttons. This makes more sense for the driver to be able to press a button on the dash to enable data logging (e.g. before an acceleration test run) and be able to stop the data logging after the test

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
