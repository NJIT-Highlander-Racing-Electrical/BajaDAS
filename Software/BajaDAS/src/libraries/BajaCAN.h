/*********************************************************************************
*   
*   BajaCAN.h  -- Version 1.0.1
* 
*   The goal of this BajaCAN header/driver is to enable all subsystems throughout
*   the vehicle to use the same variables, data types, and functions. That way,
*   any changes made to the CAN bus system can easily be applied to each subsystem
*   by simply updating the version of this file.
*
*   This driver is based on the arduino-CAN library by sandeepmistry. Since all of
*   the built-in functions to this library work well, we can simply call its
*   functions in this program.
*
*   This driver serves several functions:
*

*     *** CAN Setup/Initialization ***
*
*     This CAN driver declares all of the variables that will be used in
*     CAN transmissions. That way, there is no confusion as to what data packet
*     a variable name refers to. In addition, declaring the variables in the CAN
*     header file allows for each variable to have the same data type. If one
*     subsystem were to use a float for an RPM value while another uses an integer,
*     there is a chance we could run into issues.
*
*     Before using CAN, several initialization steps must be taken. Since this must 
*     happen during the setup() of the main code, there is a function named
*     setupCAN() that can be called in the setup() portion to execute all CAN setup.
*     By default, arduino-CAN uses GPIO_5 for CAN-TX and GPIO-4 for CAN_RX. We will
*     most likely not use these defaults as GPIO_5 is also used for SPI Chip Select.
*     Ideally, all subsystems will use the same pair of GPIO for CAN that do not
*     have any special functions. However, if setup does differ between subsystems,
*     setupCAN() has optional arguments for baud rate, GPIO, send frequency, etc.
*     The only argument that must be passed into setupCAN is a subsystem name
*     (e.g. DASHBOARD) which is used to determine which CAN messages should be
*     transmitted from each subsystem.
*     
*     A pinout for the ESP32's we use can be found here:
*     https://lastminuteengineers.com/wp-content/uploads/iot/ESP32-Pinout.png
*
*     While most pins can technically be used for CAN, some should be avoided.
*     - GPIO0, GPIO2, GPIO5, GPIO12, and GPIO15 are strapping pins used during boot 
*     - GPIO34, GPIO35, GPIO36, and GPIO39 are input-only pins
*     - GPIO5, GPIO18, GPIO19, and GPIO23 are used for SPI
*     - GPIO21 and GPIO22 are used for I2C
*     - GPIO1 and GPIO3 are used for Serial communication with a PC
*     
*     Also in CAN setup, we must configure the data rate to be used. This can be
*     50Kbps, 80Kbps, 100Kbps, etc. We will generally use 500Kbps as it provides a
*     sufficiently high data rate which also being slow enough to make signal issues
*     less common. Note that newer ESP32 microcontrollers have an issue in their CAN
*     hardware that causes them to run at half of the provided data rate. Therefore,
*     most of our subsytems should be configured at 1000Kbps.
*
*
*
*     *** CAN Receiving ***
*
*     This part of the driver is responsible for parsing incoming packets, getting the
*     packet ID, and then sorting the receiving data into the proper variable. For
*     simplicity purposes, each subsystem sorts all valid data transmissions it
*     receives, even if that data packet isn't pertient to its function. Hardware
*     limitations should not be an issue, as CAN has its own dedicated core for
*     processing on each subsystem. The ESP32 also has plenty of memory to store all of
*     the data packets we use in CAN transmission. 
*
*     When the program needs to work with a variable, such as updating a display or
*     saving data to an SD card, it can simply pass the variable knowing the CAN
*     driver has updated it with the newest value.
*
*
*
*     *** CAN Sending ***
*
*     This driver categorizes each variable based off of which subsystem should be
*     sending it. By passing through a subsystem name in setupCAN(), the subsybstem is
*     essentially telling the CAN driver, "Hey, I'm the CVT." Then, the driver would know
*     to only send CAN packets that the CVT should be updating with new data. Something
*     like the dashboard should never be reporting an RPM value to the rest of the vehicle
*     since it's not obtaining that data. This makes writing the main code easier, as CAN
*     data can just be sent on a fixed interval without any intervention by the main code.
*
*
*
*     *** Roadmap Ideas ***
*       - Implement a flag that is set when a data value is updated, and reset when
*         the main core reads that new data value. This could be practical in
*         situations where we need to be sure that the data we're using is recent.
*         There could even be a flag received from the main code that the CAN driver
*         uses to know when it has data to send out
*
************************************************************************************/

// Include sandeepmistry's arduino-CAN library
#include "arduino-CAN/src/CAN.h"

// Definitions for all CAN setup parameters
#define CAN_BAUD_RATE 1000E3
#define CAN_TX_GPIO 25
#define CAN_RX_GPIO 26

// Number of milliseconds to wait between transmissions
int canSendInterval = 100;
// Definition to log the last time that a CAN message was sent
int lastCanSendTime = 0;

// Task to run on second core (dual-core processing)
TaskHandle_t CAN_Task;

// Defines a list of acceptable subsystem names that can be used in setupCAN(<subsystem>)
enum Subsystem {
  CVT,
  DASHBOARD,
  DAS,
  WHEEL_SPEED,
  PEDALS,
  SUSPENSION,
  POWER,
  FUEL,
  BASE_STATION
};

// Declare the variable for the subsystem this driver should use
// This will be used to modify the data that is sent from this
Subsystem currentSubsystem;


// Definitions for all CAN IDs (in hex form) here:

// CVT Tachometer CAN IDs
const int primaryRPM_ID = 0x01;
const int secondaryRPM_ID = 0x02;
const int cvtTemperature_ID = 0x03;

// Wheel Speed Sensors CAN IDs
const int frontLeftWheelSpeed_ID = 0x0B;
const int frontRightWheelSpeed_ID = 0x0C;
const int rearLeftWheelSpeed_ID = 0x0D;
const int rearRightWheelSpeed_ID = 0x0E;

// Steering and Pedal Sensors CAN IDs
const int gasPedalPercentage_ID = 0x15;
const int brakePedalPercentage_ID = 0x16;
const int steeringAngle_ID = 0x17;

// Suspension Displacement CAN IDs
const int frontLeftDisplacement_ID = 0x1F;
const int frontRightDisplacement_ID = 0x20;
const int rearLeftDisplacement_ID = 0x21;
const int rearRightDisplacement_ID = 0x22;

// DAS (Data Acquisition System) CAN IDs
const int accelerationX_ID = 0x29;
const int accelerationY_ID = 0x2A;
const int accelerationZ_ID = 0x2B;
const int gyroscopeRoll_ID = 0x2C;
const int gyroscopePitch_ID = 0x2D;
const int gyroscopeYaw_ID = 0x2E;
const int gpsLatitude_ID = 0x2F;
const int gpsLongitude_ID = 0x30;
const int gpsTimeHour_ID = 0x31;
const int gpsTimeMinute_ID = 0x32;
const int gpsTimeSecond_ID = 0x33;
const int gpsDateMonth_ID = 0x34;
const int gpsDateDay_ID = 0x35;
const int gpsDateYear_ID = 0x36;
const int gpsAltitude_ID = 0x37;
const int gpsHeading_ID = 0x38;
const int gpsVelocity_ID = 0x39;
const int sdLoggingActive_ID = 0x3A;

// Power CAN IDs
const int batteryPercentage_ID = 0x47;

// Fuel Sensor CAN IDs
const int fuelPercentage_ID = 0x4C;



// Declarations for all variables to be used here:

// CVT Tachometer
int primaryRPM;
int secondaryRPM;
int cvtTemperature;

// Wheel Speed Sensors CAN
int frontLeftWheelSpeed;
int frontRightWheelSpeed;
int rearLeftWheelSpeed;
int rearRightWheelSpeed;

// Pedal Sensors CAN
int gasPedalPercentage;
int brakePedalPercentage;

// Suspension Displacement CAN
int frontLeftDisplacement;
int frontRightDisplacement;
int rearLeftDisplacement;
int rearRightDisplacement;

// DAS (Data Acquisition System) CAN
float accelerationX;
float accelerationY;
float accelerationZ;
float gyroscopeRoll;
float gyroscopePitch;
float gyroscopeYaw;
float gpsLatitude;
float gpsLongitude;
int gpsTimeHour;
int gpsTimeMinute;
int gpsTimeSecond;
int gpsDateMonth;
int gpsDateDay;
int gpsDateYear;
int gpsAltitude;
int gpsHeading;
float gpsVelocity;
bool sdLoggingActive;

// Power CAN
int batteryPercentage;

// Fuel Sensor CAN IDs
int fuelPercentage;

// Declaraction for CAN_Task_Code second core program
void CAN_Task_Code(void* pvParameters); 


// This setupCAN() function should be called in void setup() of the main program
void setupCAN(Subsystem name, int sendInterval = canSendInterval, int rxGpio = CAN_RX_GPIO, int txGpio = CAN_TX_GPIO, int baudRate = CAN_BAUD_RATE) {

  // Assign currentSubsystem based on passed through name
  currentSubsystem = name;

  // If user specified a different sendInterval in setupCAN(), assign that
  // Otherwise, it will just remain as default canSendInterval value
  canSendInterval = sendInterval;

  // Reconfigure pins used for CAN from defaults
  CAN.setPins(rxGpio, txGpio);

  if (!CAN.begin(baudRate)) {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  } else {
    Serial.println("CAN Initialized");
  }

  //create a task that will be executed in the CAN_Task_Code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    CAN_Task_Code, /* Task function. */
    "CAN_Task",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &CAN_Task,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */

  // Delay for stability; may not be necessary but only executes once
  delay(500);

}



// CAN_Task executes on secondary core of ESP32 and its sole function is CAN
// All other processing is done on primary core
void CAN_Task_Code(void* pvParameters) {
  Serial.print("CAN_Task running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {

    // Check if a packet has been received
    // Returns the packet size in bytes or 0 if no packet received
    int packetSize = CAN.parsePacket();
    int packetId;

    if ((packetSize || CAN.packetId() != -1) && (packetSize != 0)) {
      // received a packet
      packetId = CAN.packetId();  // Get the packet ID
    }

    // Sort data packet to correct variable based on ID
    switch (packetId) {

      // Primary RPM Case
      case primaryRPM_ID:
        primaryRPM = CAN.parseInt();
        break;

      // Secondary RPM Case
      case secondaryRPM_ID:
        secondaryRPM = CAN.parseInt();
        break;

      // CVT Temperature Case
      case cvtTemperature_ID:
        cvtTemperature = CAN.parseInt();
        break;
    }


    if ((millis() - lastCanSendTime) > canSendInterval) {

      switch (currentSubsystem) {

        case CVT:

          CAN.beginPacket(primaryRPM_ID);  // sets the ID for the transmission
          CAN.print(primaryRPM);           // prints data to CAN Bus just like Serial.print
          CAN.endPacket();                 // ends the sequence of sending a packet

          CAN.beginPacket(secondaryRPM_ID);
          CAN.print(secondaryRPM);
          CAN.endPacket();

          CAN.beginPacket(cvtTemperature_ID);
          CAN.print(cvtTemperature);
          CAN.endPacket();

          break;

        case DASHBOARD:
          // ...
          // ...
          // ...
          break;

        case DAS:
          // ...
          // ...
          // ...
          break;

          // ...
          // ...
          // ...
      }
    }
  }

}
