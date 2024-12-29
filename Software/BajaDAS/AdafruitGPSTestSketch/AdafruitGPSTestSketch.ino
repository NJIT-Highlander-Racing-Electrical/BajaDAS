#include <Adafruit_GPS.h>

// Pin definitions for Serial2
#define RX2 16
#define TX2 17

// Use Serial2 as the GPS serial port
#define GPSSerial Serial2

// Initialize the GPS object with the Serial2 port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
#define GPSECHO true

uint32_t timer = millis();

void setup() {
  // Start the Serial Monitor
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic parsing test!");

  // Initialize Serial2 with custom RX and TX pins
  GPSSerial.begin(9600, SERIAL_8N1, RX2, TX2);

  // Initialize the GPS with the baud rate (no extra arguments here)
  GPS.begin(9600);

  // Configure GPS settings
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Turn on RMC and GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);              // Request antenna status

  delay(1000);

  // Ask for firmware version (optional)
  GPSSerial.println(PMTK_Q_RELEASE);
}

void loop() {
  // Read data from the GPS
  char c = GPS.read();
  if (GPSECHO && c) {
    Serial.write(c); // Echo raw GPS data to Serial Monitor
  }

  // Check if a complete NMEA sentence has been received
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return; // If parsing fails, wait for the next sentence
    }
  }

  // Print GPS data every 2 seconds
  if (millis() - timer > 2000) {
    timer = millis(); // Reset timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) Serial.print('0');
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) Serial.print('0');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) Serial.print('0');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);

    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);

    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
}
