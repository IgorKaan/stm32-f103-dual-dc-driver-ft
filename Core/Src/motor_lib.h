#ifndef MOTOR_LIB_H_
#define MOTOR_LIB_H_

#include "stdint.h"
#include "pid.h"

extern struct encoder_speed_data enc_data;

typedef struct _can_TX_side {
	uint8_t first_motor_can_tx_side;
	uint8_t second_motor_can_tx_side;
} can_TX_side;

typedef struct _can_RX_data {
	  uint8_t first_wheel_rx_side;
	  uint8_t first_wheel_rx_speed;
	  uint8_t second_wheel_rx_side;
	  uint8_t second_wheel_rx_speed;
} can_RX_data;

typedef struct _encoder_speed_data {
	float encoder_speed_first;
	float encoder_speed_second;
	uint8_t encoder_speed_first_can;
	uint8_t encoder_speed_second_can;
	volatile uint16_t encoder_tick_a;
	volatile uint16_t encoder_tick_b;
	volatile uint16_t encoder_tick_c;
	volatile uint16_t encoder_tick_d;
} encoder_speed_data;


void control_first_wheel(encoder_speed_data *,can_RX_data *);
void control_second_wheel(encoder_speed_data *,can_RX_data *);
void rotate_first_cw(uint8_t target_speed, float encoder_speed);
void rotate_first_ccw(uint8_t target_speed, float encoder_speed);
void rotate_second_cw(uint8_t target_speed, float encoder_speed);
void rotate_second_ccw(uint8_t target_speed, float encoder_speed);
void reduce_speed_first_cw(float encoder_speed);
void reduce_speed_first_ccw(float encoder_speed);
void reduce_speed_second_cw(float encoder_speed);
void reduce_speed_second_ccw(float encoder_speed);
void stop_movement_first(void);
void stop_movement_second(void);

#endif
