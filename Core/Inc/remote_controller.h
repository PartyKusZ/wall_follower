
#ifndef INC_remote_controller_t_H_
#define INC_remote_controller_t_H_

#include "main.h"
#include <string.h>
#include <stdlib.h>
#define BUF_SIZE 10
typedef struct remote_controller_t{
	UART_HandleTypeDef *uart;
	uint8_t one_byte;
	uint8_t buf_counter;
	uint8_t data[BUF_SIZE];

	float kp;
	float ki;
	float kd;
	uint16_t dist_from_wall;
	uint8_t free_drive;
	uint8_t robot_state; // program on/off
	uint8_t pwm_l;
	uint8_t pwm_r;
	uint8_t interrupt;

}remote_controller_t;

void remote_controller_init(remote_controller_t *controller, UART_HandleTypeDef *uart);
uint16_t str_to_num(uint8_t *data);
void remote_controller_parser(remote_controller_t *controller);
void remote_controller_set_interrupt(remote_controller_t *controller);
void remote_controller_celar_interrupt(remote_controller_t *controller);
uint8_t remote_controller_is_data_ready(remote_controller_t *controller);


#endif /* INC_remote_controller_t_H_ */
