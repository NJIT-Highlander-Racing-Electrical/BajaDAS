#include "src/libraries/BajaCAN.h" // https://arduino.github.io/arduino-cli/0.35/sketch-specification/#src-subfolder

void setup() {

  // Initialize CAN Functionality
  setupCAN(DAS);

  // Open a Serial port for debugging
  Serial.begin(115200);

}

void loop() {

  // Main Loop Here

}

