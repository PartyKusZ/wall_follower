#include "filter_moving_averange.h"
uint16_t filter_moving_averange(uint16_t *data, uint8_t size){
	uint32_t sum = 0;

	for(uint8_t i = 0; i < size; ++i){
		sum += data[i];
	}

	return sum / size;

}
