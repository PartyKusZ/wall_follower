
#ifndef INC_PID_H_
#define INC_PID_H_
#include "main.h"

typedef struct pid_t {

	float kp;
	float ki;
	float kd;

	float error_sum;
	float error_prev;

	float windup_limit;

} pid_t;

void pid_init(pid_t *pid);
void pid_set_params(pid_t *pid, float kp, float ki, float kd, float windup_limit);
int8_t pid_calculate(pid_t *pid, uint16_t setpoint, uint16_t process_val);



#endif /* INC_PID_H_ */
