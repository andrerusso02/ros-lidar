#include <Arduino.h>
#include "hardware.h"
#include "serial_interface.h"
#include <util/atomic.h>

bool running = false;

volatile int cnt_steps = 0; // nb steps since sensor irq
volatile unsigned long timeout_wait_isr = 0; // time after which we consider the motor is blocked

int steps_per_revolution = 96;
int max_revs_per_minute = 60;
double gear_ratio = 112.0 / 18.0;
int steps_offset_zero = 160;
Stepper stepper(steps_per_revolution, coil_1_A, coil_1_B, coil_2_A, coil_2_B);


void set_mirror_speed(double vel_mirror) {
  double stepper_speed = (vel_mirror/3.14159)*gear_ratio*60;
  stepper.setSpeed(stepper_speed);
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
    stepper.step(1);
  }
  set_mirror_speed(mirror_speed);
  stepper.step(1);
}

int goToZero(){

  unsigned long timeout =  millis() + (3.14159/mirror_velocity)*1000 + 30;

  while(digitalRead(hall_sensor) == LOW){
    stepper.step(1);
    if(millis()>timeout){
      return -1;
    }
  }
  while(digitalRead(hall_sensor) == HIGH){
    stepper.step(1);
    if(millis()>timeout){
      return -1;
    }
  }

  stepper.step(steps_offset_zero);
  cnt_steps = steps_offset_zero;
  return 0;
}

void hall_sensor_isr() {
  cnt_steps = 0;
  timeout_wait_isr = millis() + (3.14159/mirror_velocity)*1000 + 30;
}

int start() {
    pinMode(hall_sensor, INPUT_PULLUP);
    pinMode(coil1_enable, OUTPUT);
    pinMode(coil2_enable, OUTPUT);

    startCoils();
    accelerateToSpeed(mirror_velocity);
    int res  = goToZero();
    attachInterrupt(digitalPinToInterrupt(hall_sensor), hall_sensor_isr, FALLING); // low when in front of sensor
    running = true;
    return res;
}

void stop() {
  running = false;
  detachInterrupt(digitalPinToInterrupt(hall_sensor));
  stopCoils();
}

void restart_if_timeout_reached() {

  if(millis()>timeout_wait_isr){
    Serial.write(ERROR_MOTOR);
    running = false;
    detachInterrupt(digitalPinToInterrupt(hall_sensor));
    accelerateToSpeed(mirror_velocity);
    goToZero();
    attachInterrupt(digitalPinToInterrupt(hall_sensor), hall_sensor_isr, FALLING);
    running = true;
  }
}

void stop_if_timeout_reached() {
  unsigned long t;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    t = timeout_wait_isr;
  }
  if(millis()>t){
    stop();
    Serial.write(ERROR_MOTOR);
  }
}

void stepper_spin() {
  if(running){
    stepper.step(1);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      cnt_steps++;
    }
    stop_if_timeout_reached();
    // send flag if new revolution starts
    if(cnt_steps == steps_offset_zero){
      Serial.write(REV_COMPLETED_FLAG);
    }
  }
}


