################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/App/common/goose_config.c 

OBJS += \
./Core/App/common/goose_config.o 

C_DEPS += \
./Core/App/common/goose_config.d 


# Each subdirectory must supply rules for building sources it contributes
Core/App/common/%.o Core/App/common/%.su Core/App/common/%.cyclo: ../Core/App/common/%.c Core/App/common/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Core/App -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-App-2f-common

clean-Core-2f-App-2f-common:
	-$(RM) ./Core/App/common/goose_config.cyclo ./Core/App/common/goose_config.d ./Core/App/common/goose_config.o ./Core/App/common/goose_config.su

.PHONY: clean-Core-2f-App-2f-common

