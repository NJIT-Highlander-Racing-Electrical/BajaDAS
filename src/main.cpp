// NJIT Highlander Racing - Electrical Subteam

// BajaDAS 2025
// Alexander Huegler


#define TX_GPIO_NUM 25  // Connects to CTX
#define RX_GPIO_NUM 26  // Connects to CRX

// Pin 13 - Switch
// Pin 34 - Button 2
// Pin 35 - Button 1

#include <math.h>
#include <string>
#include <iostream>
#include <Arduino.h>
#include <Wire.h>
#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <HardwareSerial.h>
#include <BajaCAN.h>

String hourString = "";
String minuteString = "";
String secondString = "";

String dateString = "";
String dayString = "";
String yearString = "";

// buttons & switches
const int buttonPin1 = 35;
const int buttonPin2 = 34;
const int switchPin = 13;

bool dasError = false;

// battery variables
int batteryPin = 33;
float batVoltageDividerRatio = 0.23255813953;
float batteryVoltage;

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
String gpsStatus;
HardwareSerial GPS_Serial(2);
bool hasFix = false;
int sats = 0;

// LSM9DS1 variables
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(XG_CS, M_CS);
sensors_event_t a, m, g, temp;

// log file setup
File logFile;
char logFilePath[13] = "/log.csv"; // up to "/log9999.csv" and the null terminator.

// function declarations here:
void setupSD();
void setupLSM();
void setupGPS();
// void setupLoggingMode();
void readLSM();
void updateBatteryPercentage();
void readGPS();

void parseGPGGA(String data);
void parseGPRMC(String data);
String convertToDecimalDegrees(const String& coordinate, bool isLatitude);

void logSerial();
void logSD();

void setNextAvailableFilePath();

void setup() {

  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(switchPin, INPUT_PULLUP);

  pinMode(batteryPin, INPUT);

  // start serial and wait for it to connect
  Serial.begin(115200);
  while(!Serial) {
    delay(1);
  }

  // setup connections to various modules
  setupSD();
  setupLSM();
  setupGPS();

  // setup CAN
  setupCAN(DAS);

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
  updateBatteryPercentage();

  // log data to microsd card and serial connection
  logSD();
  // logSerial();
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

void logSD() { // logging switch is being moved to dashboard, logging on/off will come over CAN
  if (true) {
    logFile = SD.open(logFilePath, FILE_APPEND);
    if (logFile) {
      if(!logFile.printf("%s, %s, %s, %f, %s, %s, %f, %f, %f, %f, %f, %f, %s, %s, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", hourString, minuteString, secondString, t, "placeholder", "placeholder", a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z, latitudeDecimal.c_str(), longitudeDecimal.c_str(), hasFix, sats, primaryRPM, secondaryRPM, primaryTemperature, secondaryTemperature, gasPedalPercentage, brakePedalPercentage, frontRightWheelRPM, frontLeftWheelRPM, rearRightWheelRPM, rearLeftWheelRPM))
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

  accelerationX = a.acceleration.x;
  accelerationY = a.acceleration.y;
  accelerationZ = a.acceleration.z;

  gyroscopePitch = g.gyro.x;
  gyroscopeYaw = g.gyro.y;
  gyroscopeRoll = g.gyro.z;
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
  Serial.print(gpsLatitude);              // GPS Latitude
  Serial.print("\t");
  Serial.print(gpsLatitude);              // GPS Longitude
  Serial.print("\t");
  Serial.print(hasFix);                   // GPS Fix
  Serial.print("\t");
  Serial.print(sats);                     // GPS Satellites

  // CVT Data
  Serial.print("\t");
  Serial.print(primaryRPM);
  Serial.print("\t");
  Serial.print(secondaryRPM);
  Serial.print("\t");
  Serial.print(primaryTemperature);
  Serial.print("\t");
  Serial.print(secondaryTemperature);

  // Pedal Data
  Serial.print("\t");
  Serial.print(gasPedalPercentage);
  Serial.print("\t");
  Serial.print(brakePedalPercentage);

  Serial.println(); // Finish with a newline
}

void updateBatteryPercentage() {

  int numSamples = 10;
  
  float voltageReadingList[numSamples];

  // take 10 readings for averaging
  for (int i = 0; i < numSamples; i++) {
   voltageReadingList[i] = float(analogRead(batteryPin)) / 4095 * 3.3 * 1.025 / batVoltageDividerRatio; // multiplied by 1.025 to account for voltage drop
  }

  // sum up those 10 readings
  float readingsSum = 0;
  for (int i = 0; i < numSamples; i++) {
  readingsSum += voltageReadingList[i];
  }

  // calculate average

  batteryVoltage = readingsSum/numSamples;
  
  if (batteryVoltage >= 12.60) batteryPercentage = 100;
  else if (batteryVoltage >= 12.54) batteryPercentage = 95;
  else if (batteryVoltage >= 12.48) batteryPercentage = 90;
  else if (batteryVoltage >= 12.42) batteryPercentage = 85;
  else if (batteryVoltage >= 12.36) batteryPercentage = 80;
  else if (batteryVoltage >= 12.30) batteryPercentage = 75;
  else if (batteryVoltage >= 12.18) batteryPercentage = 70;
  else if (batteryVoltage >= 12.06) batteryPercentage = 65;
  else if (batteryVoltage >= 11.94) batteryPercentage = 60;
  else if (batteryVoltage >= 11.82) batteryPercentage = 55;
  else if (batteryVoltage >= 11.70) batteryPercentage = 50;
  else if (batteryVoltage >= 11.58) batteryPercentage = 45;
  else if (batteryVoltage >= 11.46) batteryPercentage = 40;
  else if (batteryVoltage >= 11.34) batteryPercentage = 35;
  else if (batteryVoltage >= 11.22) batteryPercentage = 30;
  else if (batteryVoltage >= 11.10) batteryPercentage = 25;
  else if (batteryVoltage >= 10.98) batteryPercentage = 20;
  else if (batteryVoltage >= 10.86) batteryPercentage = 15;
  else if (batteryVoltage >= 10.74) batteryPercentage = 10;
  else if (batteryVoltage >= 10.62) batteryPercentage = 5;
  else if (batteryVoltage >= 10.50) batteryPercentage = 0;
  else batteryPercentage = 0;  // Below 10.5V is over-discharged
  Serial.println(batteryVoltage);
  Serial.println(batteryPercentage);
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

// Take the raw gps data and convert - $GPGGA
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
    hours -= 4;
    // Handle cases where the offset causes the hour to go negative
    if (hours < 0) {
      hours += 24; // Wrap around to previous day
    }

    if (hours == 0) {
      gpsTimeHour = 12; // Midnight
    } else if (hours > 12) {
      gpsTimeHour = hours - 12; // Afternoon/evening
    } else {
      gpsTimeHour = hours; // Morning
    }

    hourString = String(gpsTimeHour);
     if (gpsTimeHour < 10) {
      hourString = "0" + hourString;
    }

    minuteString = timedat.substring(2,4);
    gpsTimeMinute = minuteString.toInt();
    secondString = timedat.substring(4,6);
    gpsTimeSecond = secondString.toInt();

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
      gpsLatitude = latitudeDecimal.toFloat();
      longitudeDecimal = "-" + convertToDecimalDegrees(longitude, 0);
      gpsLongitude = longitudeDecimal.toFloat();
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

// Take the raw gps data and convert - $GPRMC
void parseGPRMC(String data) {
  if (data.startsWith("$GPRMC")) {
    // Split the data using commas
    int maxFields = 12;  // GPGGA has a maximum of 15 fields
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
      gpsTimeHour = 12; // Midnight
    } else if (hours > 12) {
      gpsTimeHour = hours - 12; // Afternoon/evening
    } else {
      gpsTimeHour = hours; // Morning
    }

    hourString = String(gpsTimeHour);
     if (gpsTimeHour < 10) {
      hourString = "0" + hourString;
    }

    minuteString = timedat.substring(2,4);
    gpsTimeMinute = minuteString.toInt();
    secondString = timedat.substring(4,6);
    gpsTimeSecond = secondString.toInt();

    // Extract date if available - ddmmyy
    if (fieldCount > 8 && fields[9].length() > 0) {
      gpsDateDay = fields[9].substring(0,1).toInt();
      gpsDateMonth = fields[9].substring(2,3).toInt();
      gpsDateYear = fields[9].substring(4,5).toInt();
    } else {
      gpsDateDay = 99;
      gpsDateMonth = 99;
      gpsDateYear = 99;
    }

    // Extract latitude and longitude if available
    if (fieldCount > 5 && fields[3].length() > 0 && fields[5].length() > 0) {
      latitude = fields[3];
      longitude = fields[5];
    } else {
      latitude = "";
      longitude = "";
    }

    // Convert latitude and longitude NMEA strings into decimal degrees strings (for Google Maps, GPS Visualizer, etc)
    if (latitude.length() > 0 && longitude.length() > 0) {
      // Convert NMEA coordinates to decimal degrees strings
      latitudeDecimal = convertToDecimalDegrees(latitude, 1);
      gpsLatitude = latitudeDecimal.toFloat();
      longitudeDecimal = "-" + convertToDecimalDegrees(longitude, 0);
      gpsLongitude = longitudeDecimal.toFloat();
    }

    // Extract fix status and satellite count
    if (fieldCount > 1) {
      gpsStatus = fields[2]; // A = ok (ie. fix established), V = invalid (ie. not enough satellites)
    } else {
      gpsStatus = "err";
    }
  }
}
