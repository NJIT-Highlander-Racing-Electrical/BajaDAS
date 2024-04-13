// NJIT Highlander Racing - Electrical Subteam

// BajaDAS 2024
// Shahnawaz Haque & Alexander Huegler


#define TX_GPIO_NUM 22  // Connects to CTX
#define RX_GPIO_NUM 4  // Connects to CRX

#include <Arduino.h>
#include <Wire.h>
#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <HardwareSerial.h>
#include <CAN.h>

// data variables
int cvtPrimaryRPM = 0;
int cvtSecondaryRPM = 0;
int cvtTemp = false;

int gasPedalDepression = 0;
int brakePedalDepression = 0;
int steeringWheelPosition = 0;

int fuelLevel = 0;

bool bmsLowPowerWarning = false;

float gpsLatitude = 0.0;
float gpsLongitude = 0.0;
float gpsTime = 0.0;
float gpsDate = 0.0;

bool dasEnabled = true;
bool engagementState = false;

// time from program start in seconds
float t;

// UTC time of gps observation (hours/minutes/seconds.decimal-seconds)
String timedat = "init";

// chip selects
const int8_t XG_CS = 25;
const int8_t M_CS = 26;

const uint8_t SDCARD_CS = 5;

// drive state switch variables
bool driveState; // true when in 4WD and false in 2WD

// gps initialization variables
String latitude, longitude;
HardwareSerial GPS_Serial(2);
bool hasFix = false;
int sats = 0;

// LSM9DS1 variables
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(XG_CS, M_CS);
sensors_event_t a, m, g, temp;

// last observed drive state 
String lastState;

// log file setup
File logFile;
char logFilePath[13] = "/log.csv"; // up to "/log9999.csv" and the null terminator.

// function declarations here:
void setupSD();
void setupLSM();
void setupGPS();
void setupSwitch();

void readLSM();
String readSwitch();
void readGPS();

void parseGPGGA(String data);

void updateCanbus();
void updateCanbusData();

void logSerial();
void logSD();

void setNextAvailableFilePath();

void setup() {

  // start serial and wait for it to connect
  Serial.begin(115200);
  while(!Serial) {
    delay(1);
  }

  
  CAN.setPins(RX_GPIO_NUM, TX_GPIO_NUM);

  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  } else {
    Serial.println("CAN Initialized");
  }

  // setup connections to various modules
  setupSD();
  setupLSM();
  setupSwitch();
  setupGPS();
}

void loop() {
  delay(6); // limit loop to about 75Hz, ensuring we stay under the bandwidth for serial communication with our logPrint() function
             // amount of functions per loop has changed since this number (13) was picked, so changing it should be possible

  // update all global data variables from CAN-Bus
  //updateCanbus();
  updateCanbusData();

  // read data from modules
  readLSM();
  readGPS();

  // observe drive state
  lastState = readSwitch();

  // log data to microsd card and serial connection
  logSD();
  logSerial();
}

// function definitions here:

void setupLSM()
{
  Serial.print("Initializing LSM9DS1... ");
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1.");
    while (!lsm.begin());
  }
  Serial.println("Found LSM9DS1 9DOF");

  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  // lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  // lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  // lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  // lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

void setupSD()
{
  Serial.print("Initializing SD card... ");
  if (!SD.begin(SDCARD_CS)) {
    Serial.println("Oops... unable to initialize the SD card.");
    while (1);
  }

  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  setNextAvailableFilePath();
  logFile = SD.open(logFilePath, FILE_APPEND);
  if (logFile) {
    Serial.printf("Logging to %s", logFilePath);
    Serial.println();
    if(!logFile.println("UTC, tfs, mode, ax, ay, az, mx, my, mz, gx, gy, gz, lat, lon, fix, sats, 1rpm, 2rpm, gpd, bpd"))
    {
      Serial.println("Logging Failed.");
    }
    logFile.close();
  }
  // if the file didn't open, print an error:
  else {
    Serial.printf("Failed to open %s", logFilePath);
    Serial.println();
  }
}

void setupGPS() {
  Serial.print("Initializing GPS... ");
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); // RX2, TX2
  while(!GPS_Serial) {
    delay(1);
  }
  Serial.println("Found GPS");
}

void setupSwitch() {
  Serial.print("Initializing 2WD/4WD Switch... ");
  
  pinMode(34, INPUT);
  pinMode(35, INPUT);  
  
  if (digitalRead(34) == HIGH && digitalRead(35) == HIGH) {
    driveState == true;
    Serial.println("vehicle is in 4WD mode");
  } else if (digitalRead(34) == HIGH && digitalRead(35) == LOW) {
    driveState == false;
    Serial.println("vehicle is in 2WD mode");
  } else {
    Serial.println("drive state could not be determined");
  }
}

void logSD() {
  logFile = SD.open(logFilePath, FILE_APPEND);
  if (logFile) {
    if(!logFile.printf("%s, %f, %s, %f, %f, %f, %f, %f, %f, %f, %f, %f, %s, %s, %i, %i, %i, %i, %i, %i\n", timedat, 
                       t, lastState, a.acceleration.x, a.acceleration.y, a.acceleration.z,
                       m.magnetic.x, m.magnetic.y, m.magnetic.z,
                       g.gyro.x, g.gyro.y, g.gyro.z,
                       latitude.c_str(), longitude.c_str(), hasFix, sats, cvtPrimaryRPM, cvtSecondaryRPM, gasPedalDepression, brakePedalDepression))
    {
      Serial.println("Logging Failed.");
    }
  }
  else {
    Serial.printf("Failed to open %s", logFilePath);
    Serial.println();
  }
}

void readLSM() {
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  t = millis()/1000.0;
  lsm.getEvent(&a, &m, &g, &temp);
}

void logSerial() { // Write all values to the console with tabs in between them

  // Time data
  Serial.print(timedat);
  Serial.print("\t");
  t = millis()/1000.0;
  Serial.print(t); // Assuming t is program time in seconds. This can be plotted on the x-axis!

  // Drive state
  Serial.print("\t");
  Serial.print(lastState);

  // Accel data
  Serial.print("\t");
  Serial.print(a.acceleration.x);  // Accel X
  Serial.print("\t");
  Serial.print(a.acceleration.y);  // Accel Y
  Serial.print("\t");
  Serial.print(a.acceleration.z);  // Accel Z

  // Magnetic data
  Serial.print("\t");
  Serial.print(m.magnetic.x);      // Mag X
  Serial.print("\t");
  Serial.print(m.magnetic.y);      // Mag Y
  Serial.print("\t");
  Serial.print(m.magnetic.z);      // Mag Z

  // Gyro data
  Serial.print("\t");
  Serial.print(g.gyro.x);          // Gyro X
  Serial.print("\t");
  Serial.print(g.gyro.y);          // Gyro Y
  Serial.print("\t");
  Serial.print(g.gyro.z);          // Gyro Z

  // GPS data
  Serial.print("\t");
  Serial.print(latitude);          // GPS Latitude
  Serial.print("\t");
  Serial.print(longitude);         // GPS Longitude
  Serial.print("\t");
  Serial.print(hasFix);            // GPS Fix
  Serial.print("\t");
  Serial.print(sats);          // GPS Satalites

  // CVT Data
  Serial.print("\t");
  Serial.print(cvtPrimaryRPM);
  Serial.print("\t");
  Serial.print(cvtSecondaryRPM);

  // Pedal Data
  Serial.print("\t");
  Serial.print(gasPedalDepression);
  Serial.print("\t");
  Serial.print(brakePedalDepression);

  Serial.println(); // Finish with a newline
}

void readGPS() {
  while(GPS_Serial.available()) {
    String gpsData = GPS_Serial.readStringUntil('\n');
    Serial.print(gpsData);
    parseGPGGA(gpsData);
  }
}

void setNextAvailableFilePath() {
    int fileNumber = 0;
    bool fileExists = true;

    while (fileExists) {
        // Generate the next file name in the sequence
        snprintf(logFilePath, sizeof(logFilePath), "/log%d.csv", fileNumber);
        
        // Check if the file exists on the SD card
        fileExists = SD.exists(logFilePath);

        // If the file exists, increment the file number and check again
        if (fileExists) {
            fileNumber++;
        }
    }
}

String readSwitch() {
  if ((digitalRead(34) == HIGH) && (digitalRead(35) == HIGH)){
    driveState = true;
  } else if ((digitalRead(34) == HIGH) && (digitalRead(35) == LOW)) {
    driveState = false;
  }

  if (driveState) {
    return "4WD";
  } else if (!driveState) {
    return "2WD";
  } else {
    return "err";
  }
}

void parseGPGGA(String data) {
  if (data.startsWith("$GPGGA")) {
    // Split the data using commas
    int maxFields = 15;  // GPGGA has a maximum of 15 fields
    String fields[maxFields];
    int fieldCount = 0;
    
    int start = 0;
    int pos = data.indexOf(',');
    while (pos != -1 && fieldCount < maxFields) {
      fields[fieldCount] = data.substring(start, pos);
      fieldCount++;
      
      start = pos + 1;
      pos = data.indexOf(',', start);
    }
    if (start < data.length() && fieldCount < maxFields) {
      fields[fieldCount++] = data.substring(start);
    }

    // Extract time information if available
    if (fieldCount > 0 && fields[1].length() > 0) {
      timedat = fields[1];
    } else {
      timedat = "err";
    }

    // Extract latitude and longitude if available
    if (fieldCount > 4 && fields[2].length() > 0 && fields[4].length() > 0) {
      latitude = fields[2];
      longitude = fields[4];
    } else {
      latitude = "";
      longitude = "";
    }

    // Extract fix status and satellite count
    if (fieldCount > 6) {
      int fix = fields[6].toInt();
      hasFix = (fix > 0);

      sats = fields[7].toInt();
    } else {
      hasFix = false;
      sats = 0;
    }
  }
}

void updateCanbus() {
  while (CAN.parsePacket()) {
    int packetSize = CAN.parsePacket();
    int packetId = CAN.packetId();  // Get the packet ID

    Serial.print("Received packet with id 0x");
    Serial.print(packetId, HEX);
    Serial.print(" and length ");
    Serial.println(packetSize);

    switch (packetId) {
        // CVT DATA
      case 0x1F:
        cvtPrimaryRPM = CAN.parseInt();
        Serial.print("Received primary RPM as: ");
        Serial.println(cvtPrimaryRPM);
        break;
      case 0x20:
        cvtSecondaryRPM = CAN.parseInt();
        Serial.print("Received secondary RPM as: ");
        Serial.println(cvtSecondaryRPM);
        break;
      case 0x21:
        cvtTemp = CAN.parseInt();
        Serial.print("Received cvtTemp as: ");
        Serial.println(cvtTemp);
        break;
        // PEDAL DATA
      case 0x29:
        gasPedalDepression = CAN.parseInt();
        Serial.print("Received gasPedalDepression as: ");
        Serial.println(gasPedalDepression);
        break;
      case 0x2A:
        brakePedalDepression = CAN.parseInt();
        Serial.print("Received brakePedalDepression as: ");
        Serial.println(brakePedalDepression);
        break;
      case 0x2B:
        steeringWheelPosition = CAN.parseInt();
        Serial.print("Received steeringWheelPosition as: ");
        Serial.println(steeringWheelPosition);
        break;
        // FUEL SENSOR DATA
      case 0x33:
        fuelLevel = CAN.parseInt();
        Serial.print("Received fuelLevel as: ");
        Serial.println(fuelLevel);
        break;
        // BATTERY DATA
      case 0x3D:
        bmsLowPowerWarning = CAN.parseInt();
        Serial.print("Received bmsLowPowerWarning as: ");
        Serial.println(bmsLowPowerWarning);
        break;
      default:
        Serial.println("Unknown packet ID");
        while (CAN.available()) {
          CAN.read();  // Discard the data
        }
        break;
    }
  }
}

void updateCanbusData() {
  while (CAN.parsePacket()) {
    int packetSize = CAN.parsePacket();
    int packetId = CAN.packetId();  // Get the packet ID

    // Serial.print("Received packet with id 0x");
    // Serial.print(packetId, HEX);
    // Serial.print(" and length ");
    // Serial.println(packetSize);

    switch (packetId) {
        // CVT DATA
      case 0x1F:
        cvtPrimaryRPM = CAN.parseInt();
        break;
      case 0x20:
        cvtSecondaryRPM = CAN.parseInt();
        break;
      case 0x21:
        cvtTemp = CAN.parseInt();
        break;
        // PEDAL DATA
      case 0x29:
        gasPedalDepression = CAN.parseInt();
        break;
      case 0x2A:
        brakePedalDepression = CAN.parseInt();
        break;
      case 0x2B:
        steeringWheelPosition = CAN.parseInt();
        break;
        // FUEL SENSOR DATA
      case 0x33:
        fuelLevel = CAN.parseInt();
        break;
        // BATTERY DATA
      case 0x3D:
        bmsLowPowerWarning = CAN.parseInt();
        break;
      default:
        while (CAN.available()) {
          CAN.read();  // Discard the data
        }
        break;
    }
  }
}