#include "motors.h"

TIM_HandleTypeDef *timer_left_motor;
TIM_HandleTypeDef *timer_right_motor;


void motors_init(TIM_HandleTypeDef *timer_1, TIM_HandleTypeDef *timer_2){
	timer_left_motor = timer_1;
	timer_right_motor = timer_2;
	HAL_TIM_Base_Start_IT(timer_left_motor);
	HAL_TIM_Base_Start_IT(timer_right_motor);
	HAL_TIM_PWM_Start(timer_left_motor, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(timer_right_motor, TIM_CHANNEL_1);
	__HAL_TIM_SetCompare(timer_left_motor, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(timer_right_motor, TIM_CHANNEL_1, 0);
}


void motors_set_speed(motors_t motor, uint8_t speed){
	if(speed < 0){
		speed = 0;
	}else if(speed > 100){
		speed = 100;
	}

	if(motor == LEFT_MOTOR){
		__HAL_TIM_SetCompare(timer_left_motor, TIM_CHANNEL_1, speed);
	}else if(motor == RIGHT_MOTOR){
		__HAL_TIM_SetCompare(timer_right_motor, TIM_CHANNEL_1, speed);
	}
}

uint32_t motors_get_speed(motors_t motor){
	if(motor == LEFT_MOTOR){
		return TIM16->CCR1;
	}else if(motor == RIGHT_MOTOR){
		return TIM17->CCR1;
	}
	return 0;
}



