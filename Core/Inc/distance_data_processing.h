#ifndef INC_DISTANCE_DATA_PROCESSING_H_
#define INC_DISTANCE_DATA_PROCESSING_H_
#include <math.h>
#include <stdlib.h>
#include "distance_sensos.h"

#define X1 12.7

float distance_data_processing_get_distance(distance_sensors_t *sensors);
float distance_data_processing_get_angle(distance_sensors_t *sensors);



#endif /* INC_DISTANCE_DATA_PROCESSING_H_ */
