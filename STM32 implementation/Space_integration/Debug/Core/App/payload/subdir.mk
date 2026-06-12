################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/App/payload/burn.c \
../Core/App/payload/langmuir.c \
../Core/App/payload/payload.c 

OBJS += \
./Core/App/payload/burn.o \
./Core/App/payload/langmuir.o \
./Core/App/payload/payload.o 

C_DEPS += \
./Core/App/payload/burn.d \
./Core/App/payload/langmuir.d \
./Core/App/payload/payload.d 


# Each subdirectory must supply rules for building sources it contributes
Core/App/payload/%.o Core/App/payload/%.su Core/App/payload/%.cyclo: ../Core/App/payload/%.c Core/App/payload/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Core/App -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-App-2f-payload

clean-Core-2f-App-2f-payload:
	-$(RM) ./Core/App/payload/burn.cyclo ./Core/App/payload/burn.d ./Core/App/payload/burn.o ./Core/App/payload/burn.su ./Core/App/payload/langmuir.cyclo ./Core/App/payload/langmuir.d ./Core/App/payload/langmuir.o ./Core/App/payload/langmuir.su ./Core/App/payload/payload.cyclo ./Core/App/payload/payload.d ./Core/App/payload/payload.o ./Core/App/payload/payload.su

.PHONY: clean-Core-2f-App-2f-payload

