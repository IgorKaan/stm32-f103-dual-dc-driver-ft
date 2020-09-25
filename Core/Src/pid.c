#include "pid.h"
#include "motor_lib.h"
#include "stm32f1xx_hal.h"

float _dt = 1;
float _max = 500;
float _min = -500;
float _Kp = 0.7;
float _Kd = 1.0;
float _pre_error_first = 0;
float _pre_error_second = 0;

//float calculate_pwm(encoder_speed_data * a,can_RX_data * b) {
//	float error = target_speed - current_speed;
//	float Pout = _Kp * error;
//	float derivative = (error - _pre_error) / _dt;
//	float Dout = _Kd * derivative;
//	float output = Pout + Dout;
//    if( output > _max )
//        output = _max;
//    else if( output < _min )
//        output = _min;
//    _pre_error = error;
//    return output;
//}

float calculate_pwm_first( float target_speed, float current_speed) {
	float error = target_speed - current_speed;
	float Pout = _Kp * error;
	float derivative = (error - _pre_error_first) / _dt;
	float Dout = _Kd * derivative;
	float output = Pout + Dout;
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;
    _pre_error_first = error;
    return output;
}

float calculate_pwm_second( float target_speed, float current_speed) {
	float error = target_speed - current_speed;
	float Pout = _Kp * error;
	float derivative = (error - _pre_error_second) / _dt;
	float Dout = _Kd * derivative;
	float output = Pout + Dout;
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;
    _pre_error_second = error;
    return output;
}


