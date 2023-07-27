################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/VL530X/platform/src/subdir.mk
-include Drivers/VL530X/core/src/subdir.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
wall_follower.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
wall_follower.list \

OBJCOPY_BIN += \
wall_follower.bin \


# All Target
all: wall_follower.elf secondary-outputs

# Tool invocations
wall_follower.elf: $(OBJS) $(USER_OBJS) /home/kubus/STM32CubeIDE/workspace_1.4.0/VL53L0X_tests/STM32F303K8TX_FLASH.ld
	arm-none-eabi-gcc -o "wall_follower.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/kubus/STM32CubeIDE/workspace_1.4.0/VL53L0X_tests/STM32F303K8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="wall_follower.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

wall_follower.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "wall_follower.list"
	@echo 'Finished building: $@'
	@echo ' '

wall_follower.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "wall_follower.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
