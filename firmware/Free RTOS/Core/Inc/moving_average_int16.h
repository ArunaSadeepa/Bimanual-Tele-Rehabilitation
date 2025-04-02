#ifndef INC_MOVING_AVERAGE_INT16_H_
#define INC_MOVING_AVERAGE_INT16_H_

#include "stdio.h"
#include <stdint.h>

#define MOVING_AVERAGE_LENGTH 50

typedef struct{
	float buffer[MOVING_AVERAGE_LENGTH];
	uint16_t counter;
	float out;
	float sum;
}mov_aver_instance_int16;

void reset_average_filter(mov_aver_instance_int16* instance);
void apply_average_filter(mov_aver_instance_int16* instance , float input, float *out);

#endif 
