#include <dob.h>

void set_dob(dob_instance *dob, float k, float j, float g) {

	dob->Kt = k;
	dob->Jm = j;
	dob->G_dis = g;
	dob->temp = 0;

}

void update_dob(dob_instance *dob, float Ia_ref, float velocity) {

	float T_in = Ia_ref * dob->Kt + dob->Jm * dob->G_dis * velocity;
	dob->temp = dob->temp + (T_in - dob->temp) * dob->G_dis * dt/1000.0; // /dt

	dob->T_dis = dob->temp - dob->Jm * dob->G_dis * velocity;

}
