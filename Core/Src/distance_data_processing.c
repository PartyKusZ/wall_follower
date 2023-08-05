#include "distance_data_processing.h"

float distance_data_processing_get_distance(distance_sensors_t *sensors){
	uint16_t dist_0 = distance_sensors_get_distance(sensors, 0);
	uint16_t dist_1 = distance_sensors_get_distance(sensors, 1);
	return abs(((dist_0 - dist_1) * X1) - (2 * X1 * dist_0)) / sqrt(pow(dist_0 - dist_1, 2) + 4 * pow(X1,2));


}
float distance_data_processing_get_angle(distance_sensors_t *sensors){
	uint16_t dist_0 = distance_sensors_get_distance(sensors, 0);
	uint16_t dist_1 = distance_sensors_get_distance(sensors, 1);
	float distance = distance_data_processing_get_distance(sensors);
	return acos(distance * (dist_0 - dist_1) / (X1 * (dist_0 + dist_1)));
}
