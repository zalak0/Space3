################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/App/eps/eps.c 

OBJS += \
./Core/App/eps/eps.o 

C_DEPS += \
./Core/App/eps/eps.d 


# Each subdirectory must supply rules for building sources it contributes
Core/App/eps/%.o Core/App/eps/%.su Core/App/eps/%.cyclo: ../Core/App/eps/%.c Core/App/eps/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Core/App -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-App-2f-eps

clean-Core-2f-App-2f-eps:
	-$(RM) ./Core/App/eps/eps.cyclo ./Core/App/eps/eps.d ./Core/App/eps/eps.o ./Core/App/eps/eps.su

.PHONY: clean-Core-2f-App-2f-eps

