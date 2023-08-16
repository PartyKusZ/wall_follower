#include "pid.h"

void pid_init(pid_t *pid){
	pid->kp = 0.0;
	pid->ki = 0.0;
	pid->kd = 0.0;
	pid->error_sum = 0.0;
	pid->error_prev = 0.0;
	pid->windup_limit = 0.0;
}

void pid_set_params(pid_t *pid, float kp, float ki, float kd, float windup_limit) {

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->windup_limit = windup_limit;
}


uint8_t pid_calculate(pid_t *pid, uint16_t setpoint, uint16_t process_val){

	int16_t error;
	float p_term;
	float i_term;
	float d_term;

	error = setpoint - process_val;
	pid->error_sum += error;

	p_term = (float)(pid->kp * error);
	i_term = (float)(pid->ki * pid->error_sum);
	d_term = (float)(pid->kd * (error - pid->error_prev));


	if(i_term >= pid->windup_limit){
		i_term = pid->windup_limit;
	}else if(i_term <= -pid->windup_limit){
		i_term = -pid->windup_limit;
	}

	pid->error_prev = error;

	return (uint8_t)(p_term + i_term + d_term);

}
