#include <Arduino.h>
#include "hardware.h"
#include "serial_interface.h"


void setup() {

  Serial.begin(115200);

}



void loop() {

  handle_serial_requests();

  stepper_spin();

}