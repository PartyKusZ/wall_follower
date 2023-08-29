
#include "robot_controller.h"

void robot_controller_init(){
	motors_set_speed(LEFT_MOTOR, DEFAULT_SPEED);
	motors_set_speed(RIGHT_MOTOR, DEFAULT_SPEED);
	motors_set_direction(LEFT_MOTOR, GO_FORWARD);
	motors_set_direction(RIGHT_MOTOR, GO_FORWARD);
}

void robot_controller(int16_t pid_out, uint16_t front_distance){

	if(pid_out > 0){
		motors_set_speed(RIGHT_MOTOR, DEFAULT_SPEED + pid_out);
		motors_set_speed(LEFT_MOTOR, DEFAULT_SPEED - pid_out);
	}else{
		motors_set_speed(LEFT_MOTOR, DEFAULT_SPEED - pid_out);
		motors_set_speed(RIGHT_MOTOR, DEFAULT_SPEED + pid_out);

	}
	if(front_distance < SAFETY_DISTANCE){
		motors_set_speed(LEFT_MOTOR, 0);
	    motors_set_speed(RIGHT_MOTOR, 0);
	}
}

void robot_controller_parallelism(float angle){
	if(angle > RIGHT_ANGLE + ANGLE_EPSILON){
		motors_set_direction(LEFT_MOTOR, GO_BACKWARDS);
		motors_set_direction(RIGHT_MOTOR, GO_FORWARD);
		motors_set_speed(LEFT_MOTOR, 20);
		motors_set_speed(RIGHT_MOTOR, 20);
	}else if(angle < RIGHT_ANGLE - ANGLE_EPSILON){
		motors_set_direction(LEFT_MOTOR, GO_FORWARD);
		motors_set_direction(RIGHT_MOTOR, GO_BACKWARDS);
		motors_set_speed(LEFT_MOTOR, 20);
		motors_set_speed(RIGHT_MOTOR, 20);
	}else{
		motors_set_speed(LEFT_MOTOR, 0);
		motors_set_speed(RIGHT_MOTOR, 0);
	}
}

