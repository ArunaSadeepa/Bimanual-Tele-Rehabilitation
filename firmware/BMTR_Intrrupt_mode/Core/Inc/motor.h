#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "motor_encoder.h"
#include "stdio.h"

typedef struct
{
float p_gain;
float i_gain;
float d_gain;
float last_error;
float error_integral;
float output;


}pid_instance_int16;

void set_pid_gain(pid_instance_int16 *pid_instance, float p, float i, float d);
void apply_pid(pid_instance_int16 *pid_instance, float input_error, float dt);

#endif
