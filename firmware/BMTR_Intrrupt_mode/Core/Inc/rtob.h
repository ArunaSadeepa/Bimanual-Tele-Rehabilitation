#ifndef INC_RTOB_H_
#define INC_RTOB_H_

#include "stdio.h"
#include "stdint.h"
#include <main.h>

extern float dt;

typedef struct {
	float Kt; // torque constant
	float Jm; // inertia
	float G_ext; // filter gain
	float temp;
	float T_ext; // estimated external torque
	float F; // static friction coefficient
	float B; // dynamic friction coefficient

} rtob_instance;

void set_rtob(rtob_instance *rtob, float k, float j, float g, float f, float b);

void update_rtob(rtob_instance *rtob, float Ia_ref, float velocity);

#endif
