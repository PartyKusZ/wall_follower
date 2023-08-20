#include "circular_buffer.h"

void circular_buffer_init(circular_buffer_t *buffer){

	for (uint8_t i = 0; i < BUFFER_SIZE; ++i) {
		buffer->buffer[i] = 0;
	}
	buffer->head = 0;
}

void circular_buffer_push(circular_buffer_t *buffer, uint16_t new_data){
	buffer->buffer[buffer->head] = new_data;
	buffer->head = (buffer->head + 1) % BUFFER_SIZE;
}

uint16_t* circular_buffer_get_data(circular_buffer_t *buffer){
	return buffer->buffer;
}
