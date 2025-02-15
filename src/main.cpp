// NJIT Highlander Racing - Electrical Subteam

// BajaDAS 2025
// Alexander Huegler


#define TX_GPIO_NUM 25  // Connects to CTX
#define RX_GPIO_NUM 26  // Connects to CRX

#include <Arduino.h>
#include <math.h>
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

int wheelSpeed1 = 0; // there's no wheelSpeed2...

int fuelLevel = 0;

bool bmsLowPowerWarning = false;

float gpsLatitude = 0.0;
float gpsLongitude = 0.0;

String hourString = "";
String minuteString = "";
String secondString = "";

int timeHour = 0;
int timeMinute = 0;
int timeSecond = 0;

String dateString = "";
String dayString = "";
String yearString = "";

int dateMonth = 0;
int dateDay = 0;
int dateYear = 0;

// MicroSD logging on/off
bool sdLoggingEnabled = true;
// const int LoggingModeSwitch = 13;
bool dasError = false;

// time from program start in seconds
float t;

// UTC time of gps observation (hours/minutes/seconds.decimal-seconds)
String timedat = "init";

// chip selects
const int8_t XG_CS = 21;
const int8_t M_CS = 22;

const uint8_t SDCARD_CS = 5;

// gps initialization variables
String latitude, longitude;
String latitudeDecimal, longitudeDecimal;
HardwareSerial GPS_Serial(2);
bool hasFix = false;
int sats = 0;

// LSM9DS1 variables
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(XG_CS, M_CS);
sensors_event_t a, m, g, temp;

// LSM9DS1 accelerations when static and level (without vectors) - measured values
const double initialAccelerationX = 1.87; // should be zero
const double initialAccelerationY = 9.16; // should be 9.81 (gravity)
const double initialAccelerationZ = 3.34; // should be zero

const double gravity = 9.81;

// LSM9DS1 offset angles in radians
double offsetAngleX;
double offsetAngleY;
double offsetAngleZ;

// last observed drive state 
String lastState;
String lastDasState;

// log file setup
File logFile;
char logFilePath[13] = "/log.csv"; // up to "/log9999.csv" and the null terminator.

// function declarations here:
void setupSD();
void setupLSM();
void setupLSMOffsets(double x, double y, double z);
void setupGPS();
void setupLoggingMode();
void readLSM();
String readLoggingMode();
void readGPS();

void parseGPGGA(String data);
String convertToDecimalDegrees(const String& coordinate, bool isLatitude);

void updateCanbus();
void updateCanbusData();
void sendCanbus();

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
  setupLSMOffsets(initialAccelerationX, initialAccelerationY, initialAccelerationZ);
  pinMode(13, INPUT_PULLUP); 
  readLoggingMode();
  setupGPS();
  Serial.println ("Finishing Setup");
}

void loop() {
  delay(0); // limit loop to about 75Hz, ensuring we stay under the bandwidth for serial communication with our logPrint() function
             // amount of functions per loop has changed since this number (13) was picked, so changing it should be possible


  // update all global data variables from CAN-Bus

  // updateCanbusData();

  // sendCanbus();

  // read data from modules

  readLSM();
  readGPS();

  // observe drive state
  lastDasState = readLoggingMode();

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

void setupLSMOffsets(double x, double y, double z) {

  offsetAngleX = asin(gravity / x);
  offsetAngleY = acos(initialAccelerationY / y);
  offsetAngleZ = asin(gravity / z);

}

void setupSD()
{
  Serial.print("Initializing SD card... ");
  if (!SD.begin(SDCARD_CS)) {
    Serial.println("Oops... unable to initialize the SD card.");
    dasError = true;
    while (!SD.begin(SDCARD_CS));
  }

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE){
    Serial.println("No SD card attached");
    dasError = true;
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
    dasError = false;
    Serial.printf("Logging to %s", logFilePath);
    Serial.println();
    if(!logFile.println("Hour, Minute, Second, tfs, mode, daqmode, ax, ay, az, gx, gy, gz, lat, lon, fix, sats, 1rpm, 2rpm, temp, gpd, bpd, wheelspeed1"))
    {
      Serial.println("Logging Failed.");
    }
    logFile.close();
  }
  // if the file didn't open, print an error:
  else {
    Serial.printf("Failed to open %s", logFilePath);
    Serial.println();
    dasError = true;
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

/* void setupLoggingMode() {
  Serial.println("Reading DAS switch");
  pinMode(13, INPUT_PULLUP);
  if (digitalRead(13) == HIGH) {
    sdLoggingEnabled = false;
    Serial.println("DAS RECORDING DISABLED");
  }
  else {
    sdLoggingEnabled = true;
    Serial.println("DAS RECORDING ENABLED");
  }
} */

void logSD() {
  if (sdLoggingEnabled) {
    logFile = SD.open(logFilePath, FILE_APPEND);
    if (logFile) {
      if(!logFile.printf("%s, %s, %s, %f, %s, %s, %f, %f, %f, %f, %f, %f, %s, %s, %i, %i, %i, %i, %i, %i, %i, %i\n", hourString, minuteString, secondString, 
                        t, lastState, lastDasState, (a.acceleration.x * sin(offsetAngleX)), (a.acceleration.y * cos(offsetAngleY)), (a.acceleration.z * sin(offsetAngleZ)),
                        g.gyro.x, g.gyro.y, g.gyro.z,
                        latitudeDecimal.c_str(), longitudeDecimal.c_str(), hasFix, sats, cvtPrimaryRPM, cvtSecondaryRPM, cvtTemp, gasPedalDepression, brakePedalDepression, wheelSpeed1))
      {
        Serial.println("Logging Failed.");
      }
    }
    else {
      Serial.printf("Failed to open %s", logFilePath);
      Serial.println();
    }
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
  Serial.print(hourString);
  Serial.print(":");
  Serial.print(minuteString);
  Serial.print(":");
  Serial.print(secondString);
  Serial.print("\t");
  t = millis()/1000.0;
  Serial.print(t); // Assuming t is program time in seconds. This can be plotted on the x-axis!

  // Accel data
  Serial.print("\t");
  Serial.print(a.acceleration.x);  // Accel X
  Serial.print("\t");
  Serial.print(a.acceleration.y);  // Accel Y
  Serial.print("\t");
  Serial.print(a.acceleration.z);  // Accel Z

  // Gyro data
  Serial.print("\t");
  Serial.print(g.gyro.x);          // Gyro X
  Serial.print("\t");
  Serial.print(g.gyro.y);          // Gyro Y
  Serial.print("\t");
  Serial.print(g.gyro.z);          // Gyro Z

  // GPS data
  Serial.print("\t");
  Serial.print(latitudeDecimal);          // GPS Latitude
  Serial.print("\t");
  Serial.print(longitudeDecimal);         // GPS Longitude
  Serial.print("\t");
  Serial.print(hasFix);            // GPS Fix
  Serial.print("\t");
  Serial.print(sats);          // GPS Satalites

  // CVT Data
  Serial.print("\t");
  Serial.print(cvtPrimaryRPM);
  Serial.print("\t");
  Serial.print(cvtSecondaryRPM);
  Serial.print("\t");
  Serial.print(cvtTemp);

  // Pedal Data
  Serial.print("\t");
  Serial.print(gasPedalDepression);
  Serial.print("\t");
  Serial.print(brakePedalDepression);

  //Wheel Speed Data
  Serial.print("\t");
  Serial.print(wheelSpeed1);

  Serial.println(); // Finish with a newline
}

void readGPS() {
  while(GPS_Serial.available()) {
    String gpsData = GPS_Serial.readStringUntil('\n');
    //Serial.print(gpsData);
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

String readLoggingMode() {
 if (digitalRead(13) == HIGH) {
    sdLoggingEnabled = false;
  }
  else {
    sdLoggingEnabled = true;
  }

  if (sdLoggingEnabled) {
    return "SD LOG ON";
  }
  else {
    return "SD LOG OFF";
  }
}

//Function to convert string NMEA data from GPS into usable decimal degrees format
String convertToDecimalDegrees(const String& coordinate, bool isLatitude) {
    int degreesLength = isLatitude ? 2 : 3; // Determine the number of degrees (DD or DDD)

    // Extract degrees and minutes from the string
    double degrees = coordinate.substring(0, degreesLength).toDouble();
    double minutes = coordinate.substring(degreesLength).toDouble();

    // Convert to decimal degrees
    double decimalDegrees = degrees + (minutes / 60.0);

    // Convert decimal degrees to string
    return String(decimalDegrees, 6); // 6 decimal places
}

// Take the raw gps data and convert
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

    hourString = timedat.substring(0,2); 

    int hours = hourString.toInt(); // Convert the string to an integer
    hours -= 5;
    // Handle cases where the offset causes the hour to go negative
    if (hours < 0) {
        hours += 24; // Wrap around to previous day
    }

    if (hours == 0) {
      timeHour = 12; // Midnight
    } else if (hours > 12) {
      timeHour = hours - 12; // Afternoon/evening
    } else {
      timeHour = hours; // Morning
    }

    hourString = String(timeHour);
     if (timeHour < 10) {
      hourString = "0" + hourString;
    }

    minuteString = timedat.substring(2,4);
    timeMinute = minuteString.toInt();
    secondString = timedat.substring(4,6);
    timeSecond = secondString.toInt();

    // Extract latitude and longitude if available
    if (fieldCount > 4 && fields[2].length() > 0 && fields[4].length() > 0) {
      latitude = fields[2];
      longitude = fields[4];
    } else {
      latitude = "";
      longitude = "";
    }

    // Convert latitude and longitude NMEA strings into decimal degrees strings (for Google Maps, GPS Visualizer, etc)
 if (latitude.length() > 0 && longitude.length() > 0) {
        // Convert NMEA coordinates to decimal degrees strings
        latitudeDecimal = convertToDecimalDegrees(latitude, 1);
        longitudeDecimal = "-" + convertToDecimalDegrees(longitude, 0);
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
        // Wheel speed test sensor
      case 0x96:
        wheelSpeed1 = CAN.parseInt();
        break;
      default:
        while (CAN.available()) {
          CAN.read();  // Discard the data
        }
        break;
    }
  }
}


void sendCanbus() {


   // Serial.println("Sending dasEnabled");

  CAN.beginPacket(0x51);  //sets the ID
  CAN.print(sdLoggingEnabled);  //prints data to CAN Bus just like Serial.print
  CAN.endPacket();

  //  Serial.println("Sent dasEnabled");


  delay(5); // For stability, probably can be reduced

 //   Serial.println("2");
  if (dasError) {
    CAN.beginPacket(0x53);  //sets the ID
    CAN.print(dasError);  //prints data to CAN Bus just like Serial.print
    CAN.endPacket();
    delay(5);
  }

  /*
  //  Serial.println("3");
  CAN.beginPacket(0x52);  //sets the ID
  CAN.print(driveState);  //prints data to CAN Bus just like Serial.print
  CAN.endPacket();
  delay(5);

  CAN.beginPacket(0x54);  //sets the ID
  CAN.print(driveStateUnknown);  //prints data to CAN Bus just like Serial.print
  CAN.endPacket();
  delay(5);
  */

 //   Serial.println("5");
  CAN.beginPacket(0x49);  //sets the ID
  CAN.print(timeHour);  //prints data to CAN Bus just like Serial.print
  CAN.endPacket();
  delay(5);

  CAN.beginPacket(0x4A);  //sets the ID
  CAN.print(timeMinute);  //prints data to CAN Bus just like Serial.print
  CAN.endPacket();
  delay(5);

  CAN.beginPacket(0x4B);  //sets the ID
  CAN.print(timeSecond);  //prints data to CAN Bus just like Serial.print
  CAN.endPacket();
  delay(5);
  
   // Serial.println("end");
}