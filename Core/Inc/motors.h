#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "main.h"

enum motors_t {LEFT_MOTOR = 0, RIGHT_MOTOR = 1, GO_FORWARD = 2, GO_BACKWARDS = 3} typedef motors_t;

void motors_init(TIM_HandleTypeDef *timer_1, TIM_HandleTypeDef *timer_2);

void motors_set_speed(motors_t motor, int16_t speed);

void motors_set_direction(motors_t motor, motors_t direction);

uint32_t motors_get_speed(motors_t motor);

#endif /* INC_MOTORS_H_ */
