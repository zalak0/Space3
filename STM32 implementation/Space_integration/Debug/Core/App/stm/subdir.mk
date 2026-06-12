################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/App/stm/stm.c 

OBJS += \
./Core/App/stm/stm.o 

C_DEPS += \
./Core/App/stm/stm.d 


# Each subdirectory must supply rules for building sources it contributes
Core/App/stm/%.o Core/App/stm/%.su Core/App/stm/%.cyclo: ../Core/App/stm/%.c Core/App/stm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Core/App -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-App-2f-stm

clean-Core-2f-App-2f-stm:
	-$(RM) ./Core/App/stm/stm.cyclo ./Core/App/stm/stm.d ./Core/App/stm/stm.o ./Core/App/stm/stm.su

.PHONY: clean-Core-2f-App-2f-stm

