#include "distance_sensos.h"

const uint8_t addresses[3] = {ADDR_SENSOR_0, ADDR_SENSOR_1, ADDR_SENSOR_2};
GPIO_TypeDef *port[3] = {TOF_XSHUT_0_GPIO_Port, TOF_XSHUT_1_GPIO_Port, TOF_XSHUT_2_GPIO_Port};
const uint16_t pin[3] = {TOF_XSHUT_0_Pin, TOF_XSHUT_1_Pin, TOF_XSHUT_2_Pin} ;

void distance_sensors_init(distance_sensors_t *distance_sensors, I2C_HandleTypeDef *hi2c){

	for(uint8_t i = 0; i < 3; ++i){
		distance_sensors->sensors[i] = &distance_sensors->vl53l0x[i];
		distance_sensors->data_ready[i] = 0;

	}

	for(uint8_t i = 0; i < 3; ++i){
		distance_sensors->sensors[i]->I2cHandle = hi2c;
		distance_sensors->sensors[i]->I2cDevAddr = 0x52;
	}

	for(uint8_t i = 0; i < 3; ++i){
		HAL_GPIO_WritePin(port[i], pin[i], GPIO_PIN_RESET);
		HAL_Delay(20);
	}

	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
 	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
 	HAL_NVIC_DisableIRQ(EXTI1_IRQn);

 	for(uint8_t i = 0; i < 3; ++i){
 		HAL_GPIO_WritePin(port[i], pin[i], GPIO_PIN_SET);
 		HAL_Delay(20);
 		VL53L0X_WaitDeviceBooted(distance_sensors->sensors[i]);
 		VL53L0X_DataInit(distance_sensors->sensors[i]);
 		VL53L0X_SetDeviceAddress(distance_sensors->sensors[i],addresses[i]);
 		distance_sensors->sensors[i]->I2cDevAddr = addresses[i];
 		VL53L0X_WaitDeviceBooted(distance_sensors->sensors[i]);
 		VL53L0X_StaticInit(distance_sensors->sensors[i]);
		VL53L0X_PerformRefCalibration(distance_sensors->sensors[i], &distance_sensors->VhvSettings, &distance_sensors->PhaseCal);
		VL53L0X_PerformRefSpadManagement(distance_sensors->sensors[i], &distance_sensors->refSpadCount, &distance_sensors->isApertureSpads);
		VL53L0X_SetDeviceMode(distance_sensors->sensors[i], VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

		VL53L0X_SetLimitCheckEnable(distance_sensors->sensors[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
		VL53L0X_SetLimitCheckEnable(distance_sensors->sensors[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
		VL53L0X_SetLimitCheckValue(distance_sensors->sensors[i], VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
		VL53L0X_SetLimitCheckValue(distance_sensors->sensors[i], VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
		VL53L0X_SetMeasurementTimingBudgetMicroSeconds(distance_sensors->sensors[i], 33000);
		VL53L0X_SetVcselPulsePeriod(distance_sensors->sensors[i], VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
		VL53L0X_SetVcselPulsePeriod(distance_sensors->sensors[i], VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

		VL53L0X_StartMeasurement(distance_sensors->sensors[i]);
 	}



	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

uint8_t distance_sensors_is_data_ready(distance_sensors_t *distance_sensors, uint8_t num_of_sensor){
	return distance_sensors->data_ready[num_of_sensor];
}

void distance_sensors_cleer_interrupt(distance_sensors_t *distance_sensors, uint8_t num_of_sensor){
	distance_sensors->data_ready[num_of_sensor] = 0;
}

uint16_t distance_sensors_get_distance(distance_sensors_t *distance_sensors, uint8_t num_of_sensor){
	VL53L0X_GetRangingMeasurementData(distance_sensors->sensors[num_of_sensor], &distance_sensors->ranging_data[num_of_sensor]);
	VL53L0X_ClearInterruptMask(distance_sensors->sensors[num_of_sensor], VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
	return distance_sensors->ranging_data[num_of_sensor].RangeMilliMeter;
}

