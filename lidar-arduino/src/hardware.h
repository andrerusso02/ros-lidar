#include <Stepper.h>

#define coil_1_A 8
#define coil_1_B 9
#define coil_2_A 10
#define coil_2_B 11

#define coil1_enable 7
#define coil2_enable 12

#define hall_sensor 3

// big gear = 112 teeths
// small gear = 18 teeths

extern bool running;
extern int steps_per_motor_revolution;
extern double gear_ratio;
extern int steps_offset_zero;
extern double mirror_velocity;

int start();
void stop();
void stepper_spin();
void set_mirror_speed(double vel_mirror);
