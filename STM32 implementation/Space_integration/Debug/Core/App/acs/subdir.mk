################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/App/acs/acs_task.c \
../Core/App/acs/acs_test.c \
../Core/App/acs/aplqr.c \
../Core/App/acs/bdot.c 

OBJS += \
./Core/App/acs/acs_task.o \
./Core/App/acs/acs_test.o \
./Core/App/acs/aplqr.o \
./Core/App/acs/bdot.o 

C_DEPS += \
./Core/App/acs/acs_task.d \
./Core/App/acs/acs_test.d \
./Core/App/acs/aplqr.d \
./Core/App/acs/bdot.d 


# Each subdirectory must supply rules for building sources it contributes
Core/App/acs/%.o Core/App/acs/%.su Core/App/acs/%.cyclo: ../Core/App/acs/%.c Core/App/acs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Core/App -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-App-2f-acs

clean-Core-2f-App-2f-acs:
	-$(RM) ./Core/App/acs/acs_task.cyclo ./Core/App/acs/acs_task.d ./Core/App/acs/acs_task.o ./Core/App/acs/acs_task.su ./Core/App/acs/acs_test.cyclo ./Core/App/acs/acs_test.d ./Core/App/acs/acs_test.o ./Core/App/acs/acs_test.su ./Core/App/acs/aplqr.cyclo ./Core/App/acs/aplqr.d ./Core/App/acs/aplqr.o ./Core/App/acs/aplqr.su ./Core/App/acs/bdot.cyclo ./Core/App/acs/bdot.d ./Core/App/acs/bdot.o ./Core/App/acs/bdot.su

.PHONY: clean-Core-2f-App-2f-acs

