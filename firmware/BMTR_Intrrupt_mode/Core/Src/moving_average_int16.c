
#include "moving_average_int16.h"
#include "stdio.h"
#include "math.h"

void reset_average_filter(mov_aver_instance_int16 *instance) {

	instance->counter = 0;
	instance->sum = 0;
	instance->out = 0;
	for (int i = 0; i < MOVING_AVERAGE_LENGTH; i++) {
		instance->buffer[i] = 0;
	}
}

void apply_average_filter(mov_aver_instance_int16 *instance, float input,
		float *out) {

	instance->sum += input - instance->buffer[instance->counter];
	instance->buffer[instance->counter] = input;
	instance->counter++;
	if (instance->counter == MOVING_AVERAGE_LENGTH) {
		instance->counter = 0;
	}
	instance->out = instance->sum / MOVING_AVERAGE_LENGTH;
	if (fabs(instance->out) <0.005){
		instance->out =0;
	}
	*out = instance->out;

}
