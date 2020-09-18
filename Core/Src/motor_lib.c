#include "motor_lib.h"
#include "stdint.h"
#include "pid.h"
#include "stm32f1xx_hal.h"

extern can_TX_side can_tx_side;

uint16_t MAX_PWM = 3600;
uint16_t dc_driver_pwm_first;
uint16_t dc_driver_pwm_second;
float _Max = 3600;
float _Min = 0;
float u_pwm_first = 0;
float u_pwm_second = 0;
float u_pwm_right = 0;
float u_pwm_left = 0;

enum States
{
	State_Start,
	State_CW,
	State_CCW,
	State_Stop
};

uint8_t status_first = State_CCW;
uint8_t status_second = State_CCW;

void control_first_wheel(encoder_speed_data * enc_data, can_RX_data * can_rx_data) {
	if ((can_rx_data->first_wheel_rx_side == 1)&&(status_first == State_CCW)) {
	    rotate_first_ccw(can_rx_data->first_wheel_rx_speed, enc_data->encoder_speed_first);
	    status_first = State_CCW;
	}
	else if ((can_rx_data->first_wheel_rx_side == 0)&&(status_first == State_CCW)) {
				//while (encoder_speed != 0) {
					//while (TIM2->CCR4 > 50) {
		reduce_speed_first_ccw(enc_data->encoder_speed_first);
				//}
		if (enc_data->encoder_speed_first <= 90) {
			stop_movement_first();
			status_first = State_CW;
		}
	}
	else if ((can_rx_data->first_wheel_rx_side == 0)&&(status_first == State_CW)) {
		rotate_first_cw(can_rx_data->first_wheel_rx_speed, enc_data->encoder_speed_first);
		status_first = State_CW;
	}
	else if ((can_rx_data->first_wheel_rx_side == 1)&&(status_first == State_CW)) {
				//while (encoder_speed != 0) {
				//while (TIM2->CCR4 > 50) {
		reduce_speed_first_cw(enc_data->encoder_speed_first);
				//}
		if (enc_data->encoder_speed_first <= 90) {
			stop_movement_first();
			status_first = State_CCW;
		}
	}
	else if (can_rx_data->first_wheel_rx_speed == 0) {
		stop_movement_first();
	}
}

void control_second_wheel(encoder_speed_data * enc_data, can_RX_data * can_rx_data) {
    if ((can_rx_data->second_wheel_rx_side == 1)&&(status_second == State_CCW)) {
    	rotate_second_ccw(can_rx_data->second_wheel_rx_speed, enc_data->encoder_speed_second);
    	status_second = State_CCW;
	}
	else if ((can_rx_data->second_wheel_rx_side == 0)&&(status_second == State_CCW)) {
			//while (encoder_speed != 0) {
				//while (TIM2->CCR4 > 50) {
		reduce_speed_second_ccw(enc_data->encoder_speed_second);
			//}
		if (enc_data->encoder_speed_second <= 90) {
			stop_movement_second();
			status_second = State_CW;
		}
	}
	else if ((can_rx_data->second_wheel_rx_side == 0)&&(status_second == State_CW)) {
		rotate_second_cw(can_rx_data->second_wheel_rx_speed, enc_data->encoder_speed_second);
		status_second = State_CW;
	}
	else if ((can_rx_data->second_wheel_rx_side == 1)&&(status_second == State_CW)) {
			//while (encoder_speed != 0) {
			//while (TIM2->CCR4 > 50) {
		reduce_speed_second_cw(enc_data->encoder_speed_second);
			//}
		if (enc_data->encoder_speed_second <= 90) {
			stop_movement_second();
			status_second = State_CCW;
		}
	}
	else if (can_rx_data->second_wheel_rx_speed == 0) {
		stop_movement_second();
	}
}

void rotate_first_cw(uint8_t target_speed, float encoder_speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	can_tx_side.first_motor_can_tx_side = 2;
	float inc = calculate_pwm_first((float)(target_speed), (float)encoder_speed);
	u_pwm_first += inc;
	if( u_pwm_first > _Max )
		u_pwm_first = _Max;
	else if( u_pwm_first < _Min )
		u_pwm_first = _Min;
	dc_driver_pwm_first = u_pwm_first;
    TIM2 -> CCR1 = (uint16_t)dc_driver_pwm_first;
    TIM1 -> CCR3 = MAX_PWM;
}

void rotate_first_ccw(uint8_t target_speed, float encoder_speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    can_tx_side.first_motor_can_tx_side = 1;
	float inc = calculate_pwm_first((float)(target_speed), (float)encoder_speed);
	u_pwm_first += inc;
	if( u_pwm_first > _Max )
		u_pwm_first = _Max;
	else if( u_pwm_first < _Min )
		u_pwm_first = _Min;
	dc_driver_pwm_first = u_pwm_first;
    TIM2 -> CCR1 = MAX_PWM;
    TIM1 -> CCR3 = (uint16_t)dc_driver_pwm_first;
}

void rotate_second_cw(uint8_t target_speed, float encoder_speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    can_tx_side.second_motor_can_tx_side = 2;
	float inc = calculate_pwm_second((float)(target_speed), (float)encoder_speed);
	u_pwm_second += inc;
	if( u_pwm_second > _Max )
		u_pwm_second = _Max;
	else if( u_pwm_second < _Min )
		u_pwm_second = _Min;
	dc_driver_pwm_second = u_pwm_second;
    TIM1 -> CCR2 = (uint16_t)dc_driver_pwm_second;
    TIM1 -> CCR1 = MAX_PWM;
}

void rotate_second_ccw(uint8_t target_speed, float encoder_speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    can_tx_side.second_motor_can_tx_side = 1;
	float inc = calculate_pwm_second((float)(target_speed), (float)encoder_speed);
	u_pwm_second += inc;
	if( u_pwm_second > _Max )
		u_pwm_second = _Max;
	else if( u_pwm_second < _Min )
		u_pwm_second = _Min;
	dc_driver_pwm_second = u_pwm_second;
    TIM1 -> CCR2 = MAX_PWM;
    TIM1 -> CCR1 = (uint16_t)dc_driver_pwm_second;
}

void reduce_speed_first_cw(float encoder_speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	can_tx_side.first_motor_can_tx_side = 2;
	float inc = calculate_pwm_first(0, (float)encoder_speed);
	u_pwm_first += inc;
	if( u_pwm_first > _Max )
		u_pwm_first = _Max;
	else if( u_pwm_first < _Min )
		u_pwm_first = _Min;
	dc_driver_pwm_first = u_pwm_first;
    TIM2 -> CCR1 = (uint16_t)dc_driver_pwm_first;
    TIM1 -> CCR3 = MAX_PWM;
}

void reduce_speed_first_ccw(float encoder_speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    can_tx_side.first_motor_can_tx_side = 1;
	float inc = calculate_pwm_first(0, (float)encoder_speed);
	u_pwm_first += inc;
	if( u_pwm_first > _Max )
		u_pwm_first = _Max;
	else if( u_pwm_first < _Min )
		u_pwm_first = _Min;
	dc_driver_pwm_first = u_pwm_first;
    TIM2 -> CCR1 = MAX_PWM;
    TIM1 -> CCR3 = (uint16_t)dc_driver_pwm_first;
}

void reduce_speed_second_cw(float encoder_speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    can_tx_side.second_motor_can_tx_side = 2;
	float inc = calculate_pwm_second(0, (float)encoder_speed);
	u_pwm_second += inc;
	if( u_pwm_second > _Max )
		u_pwm_second = _Max;
	else if( u_pwm_second < _Min )
		u_pwm_second = _Min;
	dc_driver_pwm_second = u_pwm_second;
    TIM1 -> CCR2 = (uint16_t)dc_driver_pwm_second;
    TIM1 -> CCR1 = MAX_PWM;
}

void reduce_speed_second_ccw(float encoder_speed) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    can_tx_side.second_motor_can_tx_side = 1;
	float inc = calculate_pwm_second(0, (float)encoder_speed);
	u_pwm_second += inc;
	if( u_pwm_second > _Max )
		u_pwm_second = _Max;
	else if( u_pwm_second < _Min )
		u_pwm_second = _Min;
	dc_driver_pwm_second = u_pwm_second;
    TIM1 -> CCR2 = MAX_PWM;
    TIM1 -> CCR1 = (uint16_t)dc_driver_pwm_second;
}

void stop_movement_first(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    TIM2 -> CCR1 = 0;
    TIM1 -> CCR3 = 0;
	u_pwm_first = 0;
}

void stop_movement_second(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    TIM1 -> CCR2 = 0;
    TIM1 -> CCR1 = 0;
	u_pwm_second = 0;
}
