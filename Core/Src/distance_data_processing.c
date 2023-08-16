#include "distance_data_processing.h"

float distance_data_processing_get_distance(uint16_t dist_0, uint16_t dist_1){

	return abs(((dist_0 - dist_1) * X1) - (2 * X1 * dist_0)) / sqrt(pow(dist_0 - dist_1, 2) + 4 * pow(X1,2));


}
float distance_data_processing_get_angle(uint16_t dist_0, uint16_t dist_1){

	float distance = distance_data_processing_get_distance(dist_0, dist_1);
	return acos(distance * (dist_0 - dist_1) / (X1 * (dist_0 + dist_1)));
}
