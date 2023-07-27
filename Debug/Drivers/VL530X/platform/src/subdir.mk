################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL530X/platform/src/vl53l0x_platform.c \
../Drivers/VL530X/platform/src/vl53l0x_platform_log.c 

OBJS += \
./Drivers/VL530X/platform/src/vl53l0x_platform.o \
./Drivers/VL530X/platform/src/vl53l0x_platform_log.o 

C_DEPS += \
./Drivers/VL530X/platform/src/vl53l0x_platform.d \
./Drivers/VL530X/platform/src/vl53l0x_platform_log.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL530X/platform/src/vl53l0x_platform.o: ../Drivers/VL530X/platform/src/vl53l0x_platform.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"/home/kubus/STM32CubeIDE/workspace_1.4.0/VL53L0X_tests/Drivers/VL530X/platform/inc" -I"/home/kubus/STM32CubeIDE/workspace_1.4.0/VL53L0X_tests/Drivers/VL530X/core/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/VL530X/platform/src/vl53l0x_platform.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/VL530X/platform/src/vl53l0x_platform_log.o: ../Drivers/VL530X/platform/src/vl53l0x_platform_log.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F303x8 -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"/home/kubus/STM32CubeIDE/workspace_1.4.0/VL53L0X_tests/Drivers/VL530X/platform/inc" -I"/home/kubus/STM32CubeIDE/workspace_1.4.0/VL53L0X_tests/Drivers/VL530X/core/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/VL530X/platform/src/vl53l0x_platform_log.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

