#ifndef INC_CIRCULAR_BUFFER_H_
#define INC_CIRCULAR_BUFFER_H_

#include "main.h"
#define BUFFER_SIZE 5

typedef struct circular_buffer_t{

	uint16_t buffer[BUFFER_SIZE];
	uint8_t head;

}circular_buffer_t;


void circular_buffer_init(circular_buffer_t *buffer);
void circular_buffer_push(circular_buffer_t *buffer, uint16_t new_data);
uint16_t* circular_buffer_get_data(circular_buffer_t *buffer);

#endif /* INC_CIRCULAR_BUFFER_H_ */
