################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/App/hardware_chips/h-bridge/drv8833.c 

OBJS += \
./Core/App/hardware_chips/h-bridge/drv8833.o 

C_DEPS += \
./Core/App/hardware_chips/h-bridge/drv8833.d 


# Each subdirectory must supply rules for building sources it contributes
Core/App/hardware_chips/h-bridge/%.o Core/App/hardware_chips/h-bridge/%.su Core/App/hardware_chips/h-bridge/%.cyclo: ../Core/App/hardware_chips/h-bridge/%.c Core/App/hardware_chips/h-bridge/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Core/App -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-App-2f-hardware_chips-2f-h-2d-bridge

clean-Core-2f-App-2f-hardware_chips-2f-h-2d-bridge:
	-$(RM) ./Core/App/hardware_chips/h-bridge/drv8833.cyclo ./Core/App/hardware_chips/h-bridge/drv8833.d ./Core/App/hardware_chips/h-bridge/drv8833.o ./Core/App/hardware_chips/h-bridge/drv8833.su

.PHONY: clean-Core-2f-App-2f-hardware_chips-2f-h-2d-bridge

