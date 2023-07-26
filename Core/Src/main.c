/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vl53l0x_api.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t Message[64];
uint8_t MessageLen;

VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_Dev_t  vl53l0x_c; // center module
VL53L0X_DEV    Dev = &vl53l0x_c;

volatile uint8_t TofDataRead;


VL53L0X_RangingMeasurementData_t RangingData_2;
VL53L0X_Dev_t  vl53l0x_c_2; // center module
VL53L0X_DEV    Dev_2 = &vl53l0x_c_2;

volatile uint8_t TofDataRead_2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	    uint32_t refSpadCount;
	    uint8_t isApertureSpads;
	    uint8_t VhvSettings;
	    uint8_t PhaseCal;


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


  MessageLen = sprintf((char*)Message, "msalamon.pl VL53L0X Continuous mode\n\r");
   HAL_UART_Transmit(&huart2, Message, MessageLen, 100);


   Dev->I2cHandle = &hi2c1;
   Dev->I2cDevAddr = 0x52;

   Dev_2->I2cHandle = &hi2c1;
   Dev_2->I2cDevAddr = 0x52;
   HAL_GPIO_WritePin(TOF_XSHUT_GPIO_Port, TOF_XSHUT_Pin, GPIO_PIN_RESET); // Disable XSHUT
   HAL_Delay(20);
   HAL_GPIO_WritePin(TOF_XSHUT_2_GPIO_Port, TOF_XSHUT_2_Pin, GPIO_PIN_RESET); // Disable XSHUT
   HAL_Delay(20);
   HAL_GPIO_WritePin(TOF_XSHUT_GPIO_Port, TOF_XSHUT_Pin, GPIO_PIN_SET);
   HAL_Delay(20);




   //
   // VL53L0X init for Single Measurement
   //



   	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
 	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

 	VL53L0X_WaitDeviceBooted(Dev);
 	VL53L0X_DataInit( Dev );

 	MessageLen = sprintf((char*)Message,"Addr change 1: %i \n\r\n\r", VL53L0X_SetDeviceAddress(Dev, 0x54));
   HAL_UART_Transmit(&huart2, Message, MessageLen, 100);
   Dev->I2cDevAddr = 0x54;

   	VL53L0X_WaitDeviceBooted(Dev);
   	//VL53L0X_DataInit(Dev);
   	VL53L0X_StaticInit(Dev);
   	VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
   	VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

    VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
   	 VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
   	 VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
   	 VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
   	 VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
    VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
   	 VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

   	 VL53L0X_StartMeasurement(Dev);


   	 HAL_GPIO_WritePin(TOF_XSHUT_2_GPIO_Port, TOF_XSHUT_2_Pin, GPIO_PIN_SET);
   	   HAL_Delay(20);

   	VL53L0X_WaitDeviceBooted(Dev_2);
	VL53L0X_DataInit(Dev_2);
   	MessageLen = sprintf((char*)Message,"Addr change 2: %i \n\r\n\r", VL53L0X_SetDeviceAddress(Dev_2, 0x56));
   	   HAL_UART_Transmit(&huart2, Message, MessageLen, 100);
   	   Dev_2->I2cDevAddr = 0x56;
    	VL53L0X_WaitDeviceBooted(Dev_2);

   	   	VL53L0X_StaticInit(Dev_2);
   	   	VL53L0X_PerformRefCalibration(Dev_2, &VhvSettings, &PhaseCal);
   	    VL53L0X_PerformRefSpadManagement(Dev_2, &refSpadCount, &isApertureSpads);
   	   	VL53L0X_SetDeviceMode(Dev_2, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

   	    VL53L0X_SetLimitCheckEnable(Dev_2, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
   	   	 VL53L0X_SetLimitCheckEnable(Dev_2, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
   	   	 VL53L0X_SetLimitCheckValue(Dev_2, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
   	   	 VL53L0X_SetLimitCheckValue(Dev_2, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
   	   	 VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev_2, 33000);
   	    VL53L0X_SetVcselPulsePeriod(Dev_2, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
   	   	 VL53L0X_SetVcselPulsePeriod(Dev_2, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

   	   	 VL53L0X_StartMeasurement(Dev_2);
   	  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
   	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(TofDataRead == 1)
	  {
		  VL53L0X_GetRangingMeasurementData(Dev, &RangingData);
		  		VL53L0X_ClearInterruptMask(Dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
		MessageLen = sprintf((char*)Message, "Measured distance: %i\n\r", RangingData.RangeMilliMeter);
		HAL_UART_Transmit(&huart2, Message, MessageLen, 100);
		TofDataRead = 0;
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	  }

	  if(TofDataRead_2 == 1)
	 	  {
	 		  VL53L0X_GetRangingMeasurementData(Dev_2, &RangingData_2);
	 		  		VL53L0X_ClearInterruptMask(Dev_2, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
	 		MessageLen = sprintf((char*)Message, "Measured distance_2: %i\n\r", RangingData_2.RangeMilliMeter);
	 		HAL_UART_Transmit(&huart2, Message, MessageLen, 100);
	 		TofDataRead_2 = 0;
	 		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	 	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TOF_XSHUT_2_Pin|TOF_XSHUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TOF_INT_Pin */
  GPIO_InitStruct.Pin = TOF_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOF_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TOF_XSHUT_2_Pin TOF_XSHUT_Pin */
  GPIO_InitStruct.Pin = TOF_XSHUT_2_Pin|TOF_XSHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TOF_INT_2_Pin */
  GPIO_InitStruct.Pin = TOF_INT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOF_INT_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == TOF_INT_Pin)
	{
//		VL53L0X_GetRangingMeasurementData(Dev, &RangingData);
//		VL53L0X_ClearInterruptMask(Dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
		TofDataRead = 1;

	}

	if(GPIO_Pin == TOF_INT_2_Pin)
		{
	//		VL53L0X_GetRangingMeasurementData(Dev, &RangingData);
	//		VL53L0X_ClearInterruptMask(Dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
			TofDataRead_2 = 1;

		}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
