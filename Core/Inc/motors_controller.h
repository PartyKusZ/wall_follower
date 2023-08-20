#ifndef INC_MOTORS_CONTROLLER_H_
#define INC_MOTORS_CONTROLLER_H_

#include "main.h"
#include "motors.h"
#include "distance_data_processing.h"

#define DEFAULT_SPEED 100




void motors_controller_move(uint8_t pid_output, uint16_t front_distance_sensor, uint16_t first_wall_sensor, uint16_t second_wall_sensor);




#endif /* INC_MOTORS_CONTROLLER_H_ */
