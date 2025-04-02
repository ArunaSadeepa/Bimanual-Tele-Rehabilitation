#include <motor.h>

#define INTEGRAL_GAIN_MAX 2000000
//#define PID_MAX 2047
#define PID_MAX 5000

// Set PID gains
void set_pid_gain(pid_instance_int16 *pid_instance, float p, float i, float d) {
	pid_instance->p_gain = p;
	pid_instance->i_gain = i;
	pid_instance->d_gain = d;

}

void reset_pid(pid_instance_int16 *pid_instance, int16_t input_error,
		uint16_t dt) {
	pid_instance->p_gain = 0;
	pid_instance->i_gain = 0;
	pid_instance->d_gain = 0;
	pid_instance->error_integral = 0;
}

void apply_pid(pid_instance_int16 *pid_instance, float input_error,
		float dt) {
	pid_instance->error_integral += input_error * dt;

	// Limit integral gain
	if (pid_instance->error_integral > INTEGRAL_GAIN_MAX) {
		pid_instance->error_integral = INTEGRAL_GAIN_MAX;
	}

	if (pid_instance->error_integral < -INTEGRAL_GAIN_MAX) {
		pid_instance->error_integral = -INTEGRAL_GAIN_MAX;
	}

	// Calculate PID output
	pid_instance->output = pid_instance->p_gain * input_error
			+ pid_instance->i_gain * (pid_instance->error_integral)
	+ pid_instance->d_gain * (input_error - pid_instance->last_error)/dt;

	// Limit PID output
	if (pid_instance->output >= PID_MAX) {
		pid_instance->output = PID_MAX;
	}

	if (pid_instance->output <= -PID_MAX) {
		pid_instance->output = -PID_MAX;
	}
	pid_instance->last_error = input_error;
}

