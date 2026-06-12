################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/App/hardware_chips/gps/gps.c \
../Core/App/hardware_chips/gps/gps_task.c 

OBJS += \
./Core/App/hardware_chips/gps/gps.o \
./Core/App/hardware_chips/gps/gps_task.o 

C_DEPS += \
./Core/App/hardware_chips/gps/gps.d \
./Core/App/hardware_chips/gps/gps_task.d 


# Each subdirectory must supply rules for building sources it contributes
Core/App/hardware_chips/gps/%.o Core/App/hardware_chips/gps/%.su Core/App/hardware_chips/gps/%.cyclo: ../Core/App/hardware_chips/gps/%.c Core/App/hardware_chips/gps/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Core/App -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-App-2f-hardware_chips-2f-gps

clean-Core-2f-App-2f-hardware_chips-2f-gps:
	-$(RM) ./Core/App/hardware_chips/gps/gps.cyclo ./Core/App/hardware_chips/gps/gps.d ./Core/App/hardware_chips/gps/gps.o ./Core/App/hardware_chips/gps/gps.su ./Core/App/hardware_chips/gps/gps_task.cyclo ./Core/App/hardware_chips/gps/gps_task.d ./Core/App/hardware_chips/gps/gps_task.o ./Core/App/hardware_chips/gps/gps_task.su

.PHONY: clean-Core-2f-App-2f-hardware_chips-2f-gps

