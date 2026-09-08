# BajaDAS: Readme and Quickstart Guide

## PlatformIO 

1. Software Dependencies:
   - `Visual Studio Code` with `PlatformIO IDE` extension
   - `Git` (if editing)
   - `GitHub Desktop App` (recommended for repository organization)
4. Clone the repository and open the project. If PlatformIO IDE is installed, it should recognize the project and automatically initialize the extension.

## Quick Start

1. Insert a formatted `MicroSD` card into the BajaDAS system.
2. Open the project folder in `Visual Studio Code`.
3. Use the `PlatformIO: Upload` button to load the code onto the microcontroller.
4. Post-race, retrieve the SD card and access the `.csv` files for analysis.

## Troubleshooting

- The main config file is `platformio.ini`.
- Avoid hotswapping MicroSD cards. While the code is set up to automatically detect a card when inserted, the code is very buggy. Always power cycle the system after inserting a new card.

# About BajaDAS
This project was started in 2023 as the first part of our whole-vehicle data acquisition (DAQ) system. It serves as the central data logger, with a few onboard sensors. It is not, however, required by or special on the CAN system.

## System Capabilities
- Data logging via MicroSD card (data is stored in a .csv file)
