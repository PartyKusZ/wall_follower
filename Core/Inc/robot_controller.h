

#ifndef INC_ROBOT_CONTROLLER_H_
#define INC_ROBOT_CONTROLLER_H_

#include "main.h"
#include "motors.h"

#define DEFAULT_SPEED 50
#define SAFETY_DISTANCE 100
#define ANGLE_EPSILON 0.085
#define RIGHT_ANGLE 1.57

#endif /* INC_ROBOT_CONTROLLER_H_ */
void robot_controller_init();
void robot_controller(int16_t pid_out, uint16_t front_distance);
void robot_controller_parallelism(float angle);

