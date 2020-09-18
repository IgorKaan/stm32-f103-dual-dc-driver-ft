#ifndef PID_H_
#define PID_H_

#include "stdint.h"
#include "motor_lib.h"

float calculate_pwm_first( float target_speed, float current_speed);
float calculate_pwm_second( float target_speed, float current_speed);

#endif
