#include <Arduino.h>
#include <Stepper.h>

#define coil_1_A 9
#define coil_1_B 10
#define coil_2_A 11
#define coil_2_B 12

int stepsPerRevolution = 96;
Stepper myStepper(stepsPerRevolution, coil_1_A, coil_1_B, coil_2_A, coil_2_B);

void setup() {

  myStepper.setSpeed(500);
}

void loop() {

  myStepper.step(stepsPerRevolution);
  //delay(1000);
}