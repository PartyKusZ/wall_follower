#ifndef INC_DISTANCE_SENSOS_H_
#define INC_DISTANCE_SENSOS_H_

#include "main.h"
#include "vl53l0x_api.h"
#define ADDR_SENSOR_0 0x54
#define ADDR_SENSOR_1 0x56
#define ADDR_SENSOR_2 0x58

struct distance_sensors_t{


	VL53L0X_RangingMeasurementData_t ranging_data[3];

	VL53L0X_Dev_t  vl53l0x[3];
	VL53L0X_DEV sensors[3];

	volatile uint8_t data_ready[3];


    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

}typedef distance_sensors_t;

void distance_sensors_init(distance_sensors_t *distance_sensors, I2C_HandleTypeDef *hi2c);

uint8_t distance_sensors_is_data_ready(distance_sensors_t *distance_sensors, uint8_t num_of_sensor);

void distance_sensors_cleer_interrupt(distance_sensors_t *distance_sensors, uint8_t num_of_sensor);
void distance_sensors_set_interrupt(distance_sensors_t *distance_sensors, uint8_t num_of_sensor);
uint16_t distance_sensors_get_distance(distance_sensors_t *distance_sensors, uint8_t num_of_sensor);



#endif /* INC_DISTANCE_SENSOS_H_ */
