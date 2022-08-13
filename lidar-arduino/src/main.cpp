#include <Arduino.h>
#include <Stepper.h>

#define hal_sensor 6

#define coil_1_A 8
#define coil_1_B 9
#define coil_2_A 10
#define coil_2_B 11

#define coil1_enable 7
#define coil2_enable 12

// big gear = 112 teeths
// small gear = 18 teeths

int steps_per_revolution = 96;
int max_revs_per_minute = 60;
double gear_ratio = 112.0 / 18.0;
Stepper myStepper(steps_per_revolution, coil_1_A, coil_1_B, coil_2_A, coil_2_B);


void set_mirror_speed(double vel_mirror) {
  double stepper_speed = (vel_mirror/3.14159)*gear_ratio*60;
  myStepper.setSpeed(stepper_speed);
}

void startCoils() {
  digitalWrite(coil1_enable, HIGH);
  digitalWrite(coil2_enable, HIGH);
}

void stopCoils() {
  digitalWrite(coil1_enable, LOW);
  digitalWrite(coil2_enable, LOW);
}

void accelerateToSpeed(double mirror_speed) {
  for(double i=0.1; i<mirror_speed; i+=0.02) {
    set_mirror_speed(i);
    myStepper.step(1);
  }
  set_mirror_speed(mirror_speed);
  myStepper.step(1);
}

void goToZero(){

  while(digitalRead(hal_sensor) == LOW){
    myStepper.step(1);
  }
  while(digitalRead(hal_sensor) == HIGH){
    myStepper.step(1);
  }
}

void setup() {

  Serial.begin(9600);

  pinMode(hal_sensor, INPUT_PULLUP);
  pinMode(coil1_enable, OUTPUT);
  pinMode(coil2_enable, OUTPUT);

  startCoils();

  accelerateToSpeed(2*3.14);
  
  goToZero();

  // delay(500);

  // stopCoils();

}

void loop() {
  
  // set_mirror_speed(3.14);

  // startCoils();
  myStepper.step(1000);
  // stopCoils();

  // delay(1000);

}