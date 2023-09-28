# bajaDAS

A Data Acquisition System (DAS) tailored for Baja vehicles, built for the ESP32 microcontroller. This system leverages the LSM9DS1 sensor to gather 9DOF data, including acceleration, magnetometer, and gyroscope readings. The acquired data is then logged onto an SD card in CSV format.

## Hardware Prerequisites
- ESP32 Microcontroller
- LSM9DS1 sensor module
- SD card module with an SD card inserted

## Key Features
1. **9DOF Data Collection**: Uses the LSM9DS1 sensor to obtain:
   - Acceleration data in X, Y, and Z axes
   - Magnetometer readings in X, Y, and Z axes
   - Gyroscopic data in X, Y, and Z axes
2. **SD Card Logging**: Data is stored on an SD card in CSV format. The system intelligently checks for existing files and increments file numbers to avoid overwriting previous data logs.
3. **Real-time Monitoring**: The system outputs real-time sensor data via the Serial console, facilitating immediate diagnostics.
4. **Error Handling**: Comprehensive error messages are provided through the Serial console in case of LSM9DS1 sensor or SD card initialization failures.
5. **Configurable Sensor Sensitivity**: The system allows for configurable sensitivity and ranges for the accelerometer, magnetometer, and gyroscope.

## Usage
1. **Installation**:
   - Ensure the necessary libraries (`Wire`, `FS`, `SD`, `SPI`, and `Adafruit_LSM9DS1`) are installed on your development environment.
   - Connect the LSM9DS1 sensor and SD card module to the ESP32 via SPI.
   - Upload the `bajaDAS` code to the microcontroller.
2. **Operation**:
   - On powering up, the system initializes the sensor and SD card.
   - Sensor data is read at regular intervals, logged to the SD card, and simultaneously displayed on the Serial console.

## Data Format
The logged data adopts the following format:
`time, ax, ay, az, mx, my, mz, gx, gy, gz`
Where:
- `time`: Timestamp (in seconds since the program started)
- `ax`, `ay`, `az`: Acceleration in X, Y, and Z axes, respectively
- `mx`, `my`, `mz`: Magnetometer readings in X, Y, and Z axes, respectively
- `gx`, `gy`, `gz`: Gyroscopic data in X, Y, and Z axes, respectively

## Future Enhancements
- [ ] **CAN Bus Protocol Support**: To integrate with the CAN bus system for richer datasets.
- [ ] **GPS Integration**: To correlate vehicle dynamics with location data.
- [ ] **Over-the-Air Updates**: Implementing OTA for streamlined system updates.
- [ ] **Advanced Data Analytics**: Enhanced tools for in-depth data interpretation.
