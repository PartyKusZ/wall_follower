#include <motors_controller.h>

void motors_controller_move(uint8_t pid_output, uint16_t front_distance_sensor, uint16_t first_wall_sensor, uint16_t second_wall_sensor){

	if(pid_output >= 0){
		motors_set_speed(LEFT_MOTOR, DEFAULT_SPEED - pid_output);

	}else{
		motors_set_speed(RIGHT_MOTOR, DEFAULT_SPEED + pid_output);
	}

}
