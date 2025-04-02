#ifndef INC_DOB_H_
#define INC_DOB_H_

#include "stdio.h"
#include "stdint.h"
#include <main.h>

extern float dt;

typedef struct {
	float Kt; // torque constant
	float Jm; // inertia
	float G_dis; // filter gain
	float temp;
	float T_dis; // estimated torque

} dob_instance;

void set_dob(dob_instance *dob, float k, float j, float g);

void update_dob(dob_instance *dob, float Ia_ref, float velocity);

#endif
