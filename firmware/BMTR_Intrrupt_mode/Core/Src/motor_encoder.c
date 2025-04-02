#include "motor_encoder.h"
#include "stdio.h"

// Initialize encoder parameters
void init_encoder(encoder_instance *encoder_value) {
	encoder_value->velocity = 0;
	encoder_value->position = 0;
	encoder_value->last_counter_value = 0;
}

// Update encoder values based on timer and quadrature counting
void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim) {

	uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim); // Get quadrature count

	encoder_value->count = (int16_t) temp_counter;
	encoder_value->velocity = (float) encoder_value->count * (float) 250.0 // ticks per sec (1000ms)  ,(250 = 1000/4)
			/ (float) dt;
	encoder_value->position += (float) encoder_value->count / (float) 4.0; //  Update position based on quadrature factor
	__HAL_TIM_SET_COUNTER(htim, 0); // reset the counter

}

void reset_encoder(encoder_instance *encoder_value) {
	encoder_value->velocity = 0;
	encoder_value->position = 0;
	encoder_value->last_counter_value = 0;
}

