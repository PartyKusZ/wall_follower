################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL530X/core/src/vl53l0x_api.c \
../Drivers/VL530X/core/src/vl53l0x_api_calibration.c \
../Drivers/VL530X/core/src/vl53l0x_api_core.c \
../Drivers/VL530X/core/src/vl53l0x_api_ranging.c \
../Drivers/VL530X/core/src/vl53l0x_api_strings.c 

OBJS += \
./Drivers/VL530X/core/src/vl53l0x_api.o \
./Drivers/VL530X/core/src/vl53l0x_api_calibration.o \
./Drivers/VL530X/core/src/vl53l0x_api_core.o \
./Drivers/VL530X/core/src/vl53l0x_api_ranging.o \
./Drivers/VL530X/core/src/vl53l0x_api_strings.o 

C_DEPS += \
./Drivers/VL530X/core/src/vl53l0x_api.d \
./Drivers/VL530X/core/src/vl53l0x_api_calibration.d \
./Drivers/VL530X/core/src/vl53l0x_api_core.d \
./Drivers/VL530X/core/src/vl53l0x_api_ranging.d \
./Drivers/VL530X/core/src/vl53l0x_api_strings.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL530X/core/src/vl53l0x_api.o: ../Drivers/VL530X/core/src/vl53l0x_api.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"/home/kubus/STM32CubeIDE/workspace_1.4.0/VL53L0X_tests/Drivers/VL530X/platform/inc" -I"/home/kubus/STM32CubeIDE/workspace_1.4.0/VL53L0X_tests/Drivers/VL530X/core/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/VL530X/core/src/vl53l0x_api.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/VL530X/core/src/vl53l0x_api_calibration.o: ../Drivers/VL530X/core/src/vl53l0x_api_calibration.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"/home/kubus/STM32CubeIDE/workspace_1.4.0/VL53L0X_tests/Drivers/VL530X/platform/inc" -I"/home/kubus/STM32CubeIDE/workspace_1.4.0/VL53L0X_tests/Drivers/VL530X/core/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/VL530X/core/src/vl53l0x_api_calibration.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/VL530X/core/src/vl53l0x_api_core.o: ../Drivers/VL530X/core/src/vl53l0x_api_core.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"/home/kubus/STM32CubeIDE/workspace_1.4.0/VL53L0X_tests/Drivers/VL530X/platform/inc" -I"/home/kubus/STM32CubeIDE/workspace_1.4.0/VL53L0X_tests/Drivers/VL530X/core/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/VL530X/core/src/vl53l0x_api_core.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/VL530X/core/src/vl53l0x_api_ranging.o: ../Drivers/VL530X/core/src/vl53l0x_api_ranging.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"/home/kubus/STM32CubeIDE/workspace_1.4.0/VL53L0X_tests/Drivers/VL530X/platform/inc" -I"/home/kubus/STM32CubeIDE/workspace_1.4.0/VL53L0X_tests/Drivers/VL530X/core/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/VL530X/core/src/vl53l0x_api_ranging.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/VL530X/core/src/vl53l0x_api_strings.o: ../Drivers/VL530X/core/src/vl53l0x_api_strings.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"/home/kubus/STM32CubeIDE/workspace_1.4.0/VL53L0X_tests/Drivers/VL530X/platform/inc" -I"/home/kubus/STM32CubeIDE/workspace_1.4.0/VL53L0X_tests/Drivers/VL530X/core/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/VL530X/core/src/vl53l0x_api_strings.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

