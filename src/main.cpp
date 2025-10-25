// NJIT Highlander Racing - Electrical Subteam

// BajaDAS 2025
// Alexander Huegler

#define TX_GPIO_NUM 25 // Connects to CTX
#define RX_GPIO_NUM 26 // Connects to CRX

bool serialStudioLogging = true; // Set to true if using SerialStudio, false if just using Serial Monitor

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

// GPS Date/Time strings
String hourString = "";
String minuteString = "";
String secondString = "";

String dayString = "";
String monthString = "";
String yearString = "";

const int timeZoneOffset = -4; // -5 for EST, -4 for EDT, -7 for Marana, AZ

// buttons & switches
const int buttonPin1 = 35;
const int buttonPin2 = 34;
const int switchPin = 13;

bool dasError = false;

// battery variables
int batteryPin = 33;
float batVoltageDividerRatio = 0.23255813953;
float batteryVoltage;
unsigned long lastBatteryCheckMillis = 0;
int batteryCheckFrequency = 10000; // Check battery voltage once every 10 seconds

// time from program start in seconds
float time_from_start;

// UTC time of gps observation (hours/minutes/seconds.decimal-seconds)
String timedat = "init";

// Chip selects for LSM9DS1
const int8_t XG_CS = 21;
const int8_t M_CS = 22;

// Chip select pin for MicroSD reader
const uint8_t SDCARD_CS = 5;

// GPS initialization variables
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
File logFile; // Keep file handle open for fast logging
bool fileCreated = false;           // This variable is used to know if we are creating a new file, or just simply logging (in the main loop)
bool filenameIsDescriptive = false; // So we know what type of file we made. We don't want the conditions to change mid-logging and our files to get corrupted
String logFilePathDescriptive = "/yyyy-mm-dd_hh-mm-ss.csv";
char logFilePath[13] = "/log.csv"; // up to "/log9999.csv" and the null terminator.

// Fast SD logging variables
bool logFileIsOpen = false;
unsigned long lastFlush = 0;
const unsigned long FLUSH_INTERVAL = 200; // Flush every 200ms for power-loss protection

// function declarations here:
void setupSD();
void setupLSM();
void setupGPS();
void setupLoggingMode();
void readLSM();
void updateBatteryPercentage();
void readGPS();

void parseGPGGA(String data);
void parseGPRMC(String data);
String convertToDecimalDegrees(const String &coordinate, bool isLatitude);

void logSerial();
void createFileSD();
void openLogFileOnce();
void fastLogSD();

void setNextAvailableFilePath();

void setup()
{

  // Initialize all input pins
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(batteryPin, INPUT);

  // start serial
  Serial.begin(460800);
  delay(1000); // I don't think while(!Serial) works on ESP32, so just delay while the serial initializes

  // setup connections to various modules
  setupGPS();
  setupSD();
  setupLSM();

  // setup CAN
  setupCAN(DAS);

  Serial.println("Finishing Setup");
}

void loop()
{
  delay(1);

  time_from_start = millis() / 1000.0;

  // read data from modules
  readLSM();
  readGPS();
  updateBatteryPercentage();

  // log data to serial connection
  logSerial();

  // log data to SD with keep-file-open approach
  if (sdLoggingActive)
  {
    // Open file once and keep it open
    if (!fileCreated)
    {
      createFileSD();
      openLogFileOnce();
      fileCreated = true;
    }
    
    // Fast logging to already-open file
    if (logFileIsOpen)
    {
      fastLogSD();
    }
  }
  else // If we are here, our logging ended and we should close the file and reset flags
  {
    // Close file when logging stops
    if (logFileIsOpen)
    {
      logFile.close();
      logFileIsOpen = false;
    }
    fileCreated = false;
  }
}

// function definitions here:

void setupLSM()
{
  Serial.print("Initializing LSM9DS1... ");
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1.");
    while (!lsm.begin())
      ;
  }
  Serial.println("Found LSM9DS1 9DOF");

  // 1.) Set the accelerometer range
  // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

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
  if (!SD.begin(SDCARD_CS))
  {
    Serial.println("Oops... unable to initialize the SD card.");
    dasError = true;
    while (!SD.begin(SDCARD_CS))
      ;
  }

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
    dasError = true;
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC)
  {
    Serial.println("MMC");
  }
  else if (cardType == CARD_SD)
  {
    Serial.println("SDSC");
  }
  else if (cardType == CARD_SDHC)
  {
    Serial.println("SDHC");
  }
  else
  {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void setupGPS() {
  Serial.println("Initializing GPS on Serial2 at 9600 bps");
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); // Start at 9600 default
  
  delay(1000); // Give GPS module time to wake up

  // Step 1: Tell GPS to switch to 57600 baud
  GPS_Serial.println("$PMTK251,57600*2C");
  delay(1000); // Wait for GPS to receive and apply change

  // Step 2: Switch Serial to 57600
  GPS_Serial.end();
  delay(100);
  GPS_Serial.begin(57600, SERIAL_8N1, 16, 17);
  Serial.println("Switched to 57600");

  delay(1000); // Wait for GPS to stabilize at new baud

  // Step 3: Send 10Hz update command
  GPS_Serial.println("$PMTK220,100*2F"); // 100 ms = 10 Hz
  delay(100);

  GPS_Serial.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  delay(100);
}

// Modified createFileSD - just creates the file, doesn't keep it open
void createFileSD()
{
  setNextAvailableFilePath();

  String filePath;
  if (gpsDateDay > 0 && gpsDateDay <= 31 && gpsDateYear != 2080)
  {
    filePath = logFilePathDescriptive;
    filenameIsDescriptive = true;
  }
  else
  {
    filePath = String(logFilePath);
    filenameIsDescriptive = false;
  }
  
  // Create file and write header if new
  File tempFile = SD.open(filePath, FILE_APPEND);
  if (tempFile)
  {
    dasError = false;
    Serial.printf("Created log file: %s\n", filePath.c_str());
    
    // Write CSV header only if file is new (empty)
    if (tempFile.size() == 0)
    {
      if (!tempFile.println("Hour, Minute, Second, time_from_start, screenshot_flag, bat_voltage, bat_percent, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, lat, lon, has_fix, num_sats, altitude (ft AGL), heading, velocity (mph), primary_rpm, secondary_rpm, primary_temp, secondary_temp, fl_wheelspeed, fr_wheelspeed, rl_wheelspeed, rr_wheelspeed, fl_wheelstate, fr_wheelstate, rl_wheelstate, rr_wheelstate, fl_wheelpos, fr_wheelpos, rl_wheelpos, rr_wheelpos, front_brake_pressure, rear_brake_pressure, gas_pos, belt_temp"))
      {
        Serial.println("Failed to write header");
      }
    }
    
    tempFile.close();
  }
  else
  {
    Serial.printf("Failed to create file: %s\n", filePath.c_str());
    dasError = true;
  }
}

void openLogFileOnce()
{
  if (filenameIsDescriptive)
  {
    logFile = SD.open(logFilePathDescriptive, FILE_APPEND);
  }
  else
  {
    logFile = SD.open(logFilePath, FILE_APPEND);
  }
  
  if (logFile)
  {
    logFileIsOpen = true;
    Serial.println("Log file opened for fast logging");
  }
  else
  {
    Serial.println("Failed to open log file for logging");
    dasError = true;
  }
}
void fastLogSD()
{
  if (!logFile) return;
  
  // Write data - fast operation (~1ms)
  // Format must match logSerial() output exactly
  if (!logFile.printf("%s,%s,%s,%.3f,%i,%.3f,%i,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%i,%i,%i,%i\n", 
      hourString.c_str(), minuteString.c_str(), secondString.c_str(),  // Time strings
      time_from_start,  // float with 3 decimals
      dataScreenshotFlag,  // int
      batteryVoltage,  // float with 3 decimals
      batteryPercentage,  // int
      a.acceleration.x, a.acceleration.y, a.acceleration.z,  // Accelerometer (3 decimals each)
      g.gyro.x, g.gyro.y, g.gyro.z,  // Gyroscope (3 decimals each)
      gpsLatitude, gpsLongitude,  // GPS position (6 decimals each) - CHANGED from string to float
      hasFix, sats, gpsAltitude, gpsHeading, gpsVelocity,  // GPS data (int)
      primaryRPM, secondaryRPM, primaryTemperature, secondaryTemperature,  // CVT data (int)
      beltTemperature,  // Belt temp (int)
      frontLeftWheelSpeed, frontRightWheelSpeed, rearLeftWheelSpeed, rearRightWheelSpeed,  // Wheel speeds (3 decimals)
      frontLeftDisplacement, frontRightDisplacement, rearLeftDisplacement, rearRightDisplacement,  // Suspension (3 decimals)
      frontBrakePressure, rearBrakePressure, gasPedalPercentage,  // Pedal/brake data (int)
      sdLoggingActive))  // SD logging status (int) - THIS WAS MISSING!
  {
    Serial.println("Fast logging failed");
  }
  
  // Periodic flush for power-loss protection
  unsigned long currentTime = millis();
  if (currentTime - lastFlush >= FLUSH_INTERVAL)
  {
    logFile.flush(); // Force write to SD card
    lastFlush = currentTime;
  }
}

void readLSM()
{
  lsm.read(); /* ask it to read in the data */

  /* Get a new sensor event */
  lsm.getEvent(&a, &m, &g, &temp);

  accelerationX = a.acceleration.x;
  accelerationY = a.acceleration.y;
  accelerationZ = a.acceleration.z;

  gyroscopePitch = g.gyro.x;
  gyroscopeYaw = g.gyro.y;
  gyroscopeRoll = g.gyro.z;
}

void logSerial()
{ // Write all values to the console

  char sepChar; // Separation character, either a comma or a tab

  if (serialStudioLogging)
  {
    sepChar = ','; // Use comma separated variables for serial studio loggin
  }
  else
  {
    sepChar = '\t'; // If we are just using the serial monitor, make it look nice using tabs instead
  }

  // Time data
  Serial.print(hourString);
  Serial.print(sepChar);
  Serial.print(minuteString);
  Serial.print(sepChar);
  Serial.print(secondString);
  Serial.print(sepChar);
  Serial.print(time_from_start, 3); // Assuming time_from_start is program time in seconds. This can be plotted on the x-axis!

  // Data Screenshot Flag Marker
  Serial.print(sepChar);
  Serial.print(dataScreenshotFlag);

  // Battery Data
  Serial.print(sepChar);
  Serial.print(batteryVoltage, 3);
  Serial.print(sepChar);
  Serial.print(batteryPercentage);

  // Accel data
  Serial.print(sepChar);
  Serial.print(a.acceleration.x, 3); // Accel X
  Serial.print(sepChar);  
  Serial.print(a.acceleration.y, 3); // Accel Y
  Serial.print(sepChar);
  Serial.print(a.acceleration.z, 3); // Accel Z

  // Gyro data
  Serial.print(sepChar);
  Serial.print(g.gyro.x, 3); // Gyro X
  Serial.print(sepChar);
  Serial.print(g.gyro.y, 3); // Gyro Y
  Serial.print(sepChar);
  Serial.print(g.gyro.z, 3); // Gyro Z

  // GPS data
  Serial.print(sepChar);
  Serial.print(gpsLatitude, 6); // GPS Latitude (with 6 decimal places)
  Serial.print(sepChar);
  Serial.print(gpsLongitude, 6); // GPS Longitude (with 6 decimal places)
  Serial.print(sepChar);
  Serial.print(hasFix); // GPS Fix
  Serial.print(sepChar);
  Serial.print(sats); // GPS Satellites
  Serial.print(sepChar);
  Serial.print(gpsAltitude); // GPS Altitude
  Serial.print(sepChar);
  Serial.print(gpsHeading); // GPS Heading
  Serial.print(sepChar);
  Serial.print(gpsVelocity); // GPS Velocity

  // CVT Data
  Serial.print(sepChar);
  Serial.print(primaryRPM);
  Serial.print(sepChar);
  Serial.print(secondaryRPM);
  Serial.print(sepChar);
  Serial.print(primaryTemperature);
  Serial.print(sepChar);
  Serial.print(secondaryTemperature);
  Serial.print(sepChar);
  Serial.print(beltTemperature);

  // Wheel RPM Data
  Serial.print(sepChar);
  Serial.print(frontLeftWheelSpeed, 3);
  Serial.print(sepChar);
  Serial.print(frontRightWheelSpeed, 3);
  Serial.print(sepChar);
  Serial.print(rearLeftWheelSpeed, 3);
  Serial.print(sepChar);
  Serial.print(rearRightWheelSpeed, 3);

  // Wheel Displacement Data
  Serial.print(sepChar);
  Serial.print(frontLeftDisplacement, 3);
  Serial.print(sepChar);
  Serial.print(frontRightDisplacement, 3);
  Serial.print(sepChar);
  Serial.print(rearLeftDisplacement, 3);
  Serial.print(sepChar);
  Serial.print(rearRightDisplacement, 3);

  // Pedal Data
  Serial.print(sepChar);
  Serial.print(frontBrakePressure);
  Serial.print(sepChar);
  Serial.print(rearBrakePressure);
  Serial.print(sepChar);
  Serial.print(gasPedalPercentage);

  // SD Logging Active
  Serial.print(sepChar);
  Serial.print(sdLoggingActive);

  Serial.println(); // Finish with a newline
}


void updateBatteryPercentage()
{

  if ((millis() - lastBatteryCheckMillis) > batteryCheckFrequency)
  {

    int numSamples = 10;

    float voltageReadingList[numSamples];

    // take 10 readings for averaging
    for (int i = 0; i < numSamples; i++)
    {
      voltageReadingList[i] = float(analogRead(batteryPin)) / 4095 * 3.3 * 1.025 / batVoltageDividerRatio; // multiplied by 1.025 to account for voltage drop
    }

    // sum up those 10 readings
    float readingsSum = 0;
    for (int i = 0; i < numSamples; i++)
    {
      readingsSum += voltageReadingList[i];
    }

    // calculate average

    batteryVoltage = readingsSum / numSamples;

    if (batteryVoltage >= 12.60)
      batteryPercentage = 100;
    else if (batteryVoltage >= 12.54)
      batteryPercentage = 95;
    else if (batteryVoltage >= 12.48)
      batteryPercentage = 90;
    else if (batteryVoltage >= 12.42)
      batteryPercentage = 85;
    else if (batteryVoltage >= 12.36)
      batteryPercentage = 80;
    else if (batteryVoltage >= 12.30)
      batteryPercentage = 75;
    else if (batteryVoltage >= 12.18)
      batteryPercentage = 70;
    else if (batteryVoltage >= 12.06)
      batteryPercentage = 65;
    else if (batteryVoltage >= 11.94)
      batteryPercentage = 60;
    else if (batteryVoltage >= 11.82)
      batteryPercentage = 55;
    else if (batteryVoltage >= 11.70)
      batteryPercentage = 50;
    else if (batteryVoltage >= 11.58)
      batteryPercentage = 45;
    else if (batteryVoltage >= 11.46)
      batteryPercentage = 40;
    else if (batteryVoltage >= 11.34)
      batteryPercentage = 35;
    else if (batteryVoltage >= 11.22)
      batteryPercentage = 30;
    else if (batteryVoltage >= 11.10)
      batteryPercentage = 25;
    else if (batteryVoltage >= 10.98)
      batteryPercentage = 20;
    else if (batteryVoltage >= 10.86)
      batteryPercentage = 15;
    else if (batteryVoltage >= 10.74)
      batteryPercentage = 10;
    else if (batteryVoltage >= 10.62)
      batteryPercentage = 5;
    else if (batteryVoltage >= 10.50)
      batteryPercentage = 0;
    else
      batteryPercentage = 0; // Below 10.5V is over-discharged

      lastBatteryCheckMillis = millis();

  }
}

void readGPS()
{
  while (GPS_Serial.available())
  {
    String gpsData = GPS_Serial.readStringUntil('\n');  // Read a line of data from the GPS serial port
    gpsData.trim();                                     // Remove any leading or trailing whitespace/newline characters

    if (gpsData.startsWith("$GPGGA"))                   // Check if the line is a GPGGA sentence
    {
      parseGPGGA(gpsData);                              // Parse the GPGGA sentence for position, fix, satellites, altitude, etc.
    }
    else if (gpsData.startsWith("$GPRMC"))              // Check if the line is a GPRMC sentence
    {
      parseGPRMC(gpsData);                              // Parse the GPRMC sentence for speed, heading, date, etc.
    }
  }
}


void setNextAvailableFilePath()
{

  // Check to see if we have a fix for the date. If so, set the log path to the date/time
  if (gpsDateDay > 0 && gpsDateDay <= 31 && gpsDateYear != 2080)
  {
    // If the year is 2080, we appended 2000 to the default 1980 and we should ignore the descriptive file path
    logFilePathDescriptive = "/" + yearString + "-" + monthString + "-" + dayString + "_" + hourString + "-" + minuteString + "-" + secondString + ".csv";
  }

  // If we could not get a date and time, just create a generic log file name (e.g "log4")
  else
  {

    int fileNumber = 0;
    bool fileExists = true;

    while (fileExists)
    {
      // Generate the next file name in the sequence
      snprintf(logFilePath, sizeof(logFilePath), "/log%d.csv", fileNumber);

      // Check if the file exists on the SD card
      fileExists = SD.exists(logFilePath);

      // If the file exists, increment the file number and check again
      if (fileExists)
      {
        fileNumber++;
      }
    }
  }
}

// Function to convert string NMEA data from GPS into usable decimal degrees format
String convertToDecimalDegrees(const String &coordinate, bool isLatitude)
{
  int degreesLength = isLatitude ? 2 : 3; // Determine the number of degrees (DD or DDD)

  // Extract degrees and minutes from the string
  double degrees = coordinate.substring(0, degreesLength).toDouble();
  double minutes = coordinate.substring(degreesLength).toDouble();

  // Convert to decimal degrees
  double decimalDegrees = degrees + (minutes / 60.0);

  // Convert decimal degrees to string
  return String(decimalDegrees, 6); // 6 decimal places
}

// Take the raw gps data and convert - $GPGGA 'Global Positioning System Fix Data'
void parseGPGGA(String data)
{
  if (data.startsWith("$GPGGA"))
  {
    // Split the data using commas
    int maxFields = 15; // GPGGA has a maximum of 15 fields
    String fields[maxFields];
    int fieldCount = 0;

    int start = 0;
    int pos = data.indexOf(',');
    while (pos != -1 && fieldCount < maxFields)
    {
      fields[fieldCount] = data.substring(start, pos);
      fieldCount++;

      start = pos + 1;
      pos = data.indexOf(',', start);
    }
    if (start < data.length() && fieldCount < maxFields)
    {
      fields[fieldCount++] = data.substring(start);
    }

    // Extract time information if available
    if (fieldCount > 0 && fields[1].length() > 0)
    {
      timedat = fields[1];
    }
    else
    {
      timedat = "err";
    }

    hourString = timedat.substring(0, 2);

    int hours = hourString.toInt(); // Convert the string to an integer
    hours += timeZoneOffset;
    // Handle cases where the offset causes the hour to go negative
    if (hours < 0)
    {
      hours += 24; // Wrap around to previous day
    }

    if (hours == 0)
    {
      gpsTimeHour = 12; // Midnight
    }
    else if (hours > 12)
    {
      gpsTimeHour = hours - 12; // Afternoon/evening
    }
    else
    {
      gpsTimeHour = hours; // Morning
    }

    hourString = String(gpsTimeHour);
    if (gpsTimeHour < 10)
    {
      hourString = "0" + hourString;
    }

    minuteString = timedat.substring(2, 4);
    gpsTimeMinute = minuteString.toInt();
    secondString = timedat.substring(4, 6);
    gpsTimeSecond = secondString.toInt();

    // Extract latitude and longitude if available
    if (fieldCount > 4 && fields[2].length() > 0 && fields[4].length() > 0)
    {
      latitude = fields[2];
      longitude = fields[4];
    }
    else
    {
      latitude = "";
      longitude = "";
    }

    // Convert latitude and longitude NMEA strings into decimal degrees strings (for Google Maps, GPS Visualizer, etc)
    if (latitude.length() > 0 && longitude.length() > 0)
    {
      // Convert NMEA coordinates to decimal degrees strings
      latitudeDecimal = convertToDecimalDegrees(latitude, 1);
      gpsLatitude = latitudeDecimal.toFloat();
      longitudeDecimal = "-" + convertToDecimalDegrees(longitude, 0);
      gpsLongitude = longitudeDecimal.toFloat();
    }

    // Extract fix status and satellite count
    if (fieldCount > 6)
    {
      int fix = fields[6].toInt();
      hasFix = (fix > 0);
      sats = fields[7].toInt();
    }
    else
    {
      hasFix = false;
      sats = 0;
    }

    // Extract altitude if available
    if (fieldCount > 9 && fields[9].length() > 0)
    {
      gpsAltitude = fields[9].toFloat(); // Altitude in meters
      gpsAltitude *= 3.28084;            // convert to feet
    }
    else
    {
      gpsAltitude = -1; // Default or error value
    }
  }
}
void parseGPRMC(String data)
{
  if (data.startsWith("$GPRMC"))
  {
    int maxFields = 12;
    String fields[maxFields];
    int fieldCount = 0;

    int start = 0;
    int pos = data.indexOf(',');
    while (pos != -1 && fieldCount < maxFields)
    {
      fields[fieldCount++] = data.substring(start, pos);
      start = pos + 1;
      pos = data.indexOf(',', start);
    }
    if (start < data.length() && fieldCount < maxFields)
    {
      fields[fieldCount++] = data.substring(start);
    }

    // Speed in knots (field 7), convert to MPH
    if (fieldCount > 7 && fields[7].length() > 0)
    {
      float speedKnots = fields[7].toFloat();
      gpsVelocity = speedKnots * 1.15078; // MPH
    }
    else
    {
      gpsVelocity = -1;
    }

    // Heading in degrees (field 8)
    if (fieldCount > 8 && fields[8].length() > 0)
    {
      gpsHeading = fields[8].toFloat();
    }
    else
    {
      gpsHeading = -1;
    }

    // Date: DDMMYY (field 9)
    if (fieldCount > 9 && fields[9].length() == 6)
    {
      dayString = fields[9].substring(0, 2);
      monthString = fields[9].substring(2, 4);
      yearString = "20" + fields[9].substring(4, 6);
      gpsDateDay = dayString.toInt();
      gpsDateMonth = monthString.toInt();
      gpsDateYear = yearString.toInt(); // Convert to 4-digit year
    }
    else
    {
      gpsDateDay = gpsDateMonth = gpsDateYear = -1;
    }
  }
}