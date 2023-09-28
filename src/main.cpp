#include <Arduino.h>
#include <Wire.h>
#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>

float t;

const int8_t XG_CS = 25;
const int8_t M_CS = 26;
const uint8_t SDCARD_CS = 5;
static constexpr double DEGREES_PER_RADIAN =
      (180.0 / 3.141592653589793238463);

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(XG_CS, M_CS);
sensors_event_t a, m, g, temp;

File logFile;
char logFilePath[13] = "/log.csv"; // up to "/log9999.csv" and the null terminator.

// function declarations here:
void setupSD();
void setupLSM();
void readLSM();
void logLSM();
void printLSM();
void setNextAvailableFilePath();

void setup() {
  Serial.begin(115200);

  while(!Serial) {
    delay(1);
  }

  setupLSM();
  setupSD();
}

void loop() {
  delay(10);
  readLSM();
  logLSM();
  printLSM();
}

// function definitions here:

void setupLSM()
{
  Serial.print("Initializing LSM9DS1...");
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1.");
    while (1);
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
  Serial.print("Initializing SD card...");
  if (!SD.begin(SDCARD_CS)) {
    Serial.println("Oops ... unable to initialize the SD card.");
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
    if(!logFile.println("time, ax, ay, az, mx, my, mz, gx, gy, gz"))
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

void logLSM() {
  logFile = SD.open(logFilePath, FILE_APPEND);
  if (logFile) {
    if(!logFile.printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", t, a.acceleration.x, a.acceleration.y, a.acceleration.z,
                                                                       m.magnetic.x, m.magnetic.y, m.magnetic.z,
                                                                       g.gyro.x, g.gyro.y, g.gyro.z))
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

void printLSM() {
  t = millis()/1000.0;
  // Write all values to the console with tabs in between them
  Serial.print(t); // Assuming t is program time in seconds. This can be plotted on the x-axis!

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

  Serial.println(); // Finish with a newline
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
