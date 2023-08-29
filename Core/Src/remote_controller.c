#include "remote_controller.h"

void remote_controller_init(remote_controller_t *controller, UART_HandleTypeDef *uart){
	controller->uart = uart;
	controller->one_byte = 0;
	controller->buf_counter = 0;
    memset(controller->data,0,BUF_SIZE);
	controller->kp = 0.0f;
	controller->ki = 0.0f;
	controller->kd = 0.0f;
	controller->dist_from_wall = 0;
	controller->free_drive = 0;
	controller->robot_state = 0;
	controller->pwm_l = 0;
	controller->pwm_r = 0;
	controller->speed = 0;
}
void remote_controller_set_interrupt(remote_controller_t *controller){
	controller->interrupt = 1;
}
uint8_t remote_controller_is_data_ready(remote_controller_t *controller){
	return controller->interrupt;
}
void remote_controller_celar_interrupt(remote_controller_t *controller){
	controller->interrupt = 0;
}

uint16_t str_to_num(uint8_t *data){
	uint8_t str_to_num[10];
	uint8_t j = 0;

	for(uint8_t i = 3; i != '\n'; ++i,++j ){
		str_to_num[j] = data[i];
	}
	str_to_num[++j] = '\0';


	return (atoi((char *)str_to_num));
}

float str_to_num_f(uint8_t *data){
	uint8_t str_to_num[10];
	uint8_t j = 0;

	for(uint8_t i = 3; i != '\n'; ++i,++j ){
		str_to_num[j] = data[i];
	}
	str_to_num[++j] = '\0';


	return (atoff((char *)str_to_num));
}

void remote_controller_parser(remote_controller_t *controller){

	if(!strncmp((char *) controller->data,"kp",2)){
		controller->kp = str_to_num_f(controller->data);
	}else if(!strncmp((char *) controller->data,"ki",2)){
		controller->ki = str_to_num_f(controller->data);
	}else if(!strncmp((char *) controller->data,"kd",2)){
		controller->kd = str_to_num_f(controller->data);
	}else if(!strncmp((char *) controller->data,"dw",2)){ //dist from wall
		controller->dist_from_wall = str_to_num(controller->data);
	}else if(!strncmp((char *) controller->data,"fd",2)){ //free drive
		controller->free_drive = str_to_num(controller->data);
	}else if(!strncmp((char *) controller->data,"rs",2)){ //robot state
		controller->robot_state = str_to_num(controller->data);
	}else if(!strncmp((char *) controller->data,"pl",2)){ //pwm l
		controller->pwm_l = str_to_num(controller->data);
	}else if(!strncmp((char *) controller->data,"pr",2)){ //pwm r
		controller->pwm_r = str_to_num(controller->data);
	}else if(!strncmp((char *) controller->data,"sp",2)){ //pwm r
		controller->speed = str_to_num(controller->data); //speed
	}

}
