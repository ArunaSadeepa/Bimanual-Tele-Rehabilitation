#ifndef INC_MOTOR_ENCODER_H_
#define INC_MOTOR_ENCODER_H_
#include "stdint.h"
#include <main.h>

extern float dt;

typedef struct {
	float velocity; //   ticks per seconds
	float position; // ticks position
	float last_counter_value;
	int16_t count;
	uint8_t first_time; // flag for first time
} encoder_instance;

void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim);
void restart_encoder(encoder_instance *encoder_value);
void init_encoder(encoder_instance *encoder_value);

#endif

