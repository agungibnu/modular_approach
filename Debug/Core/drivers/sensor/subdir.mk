################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/drivers/sensor/bmp280.c \
../Core/drivers/sensor/sensor.c 

OBJS += \
./Core/drivers/sensor/bmp280.o \
./Core/drivers/sensor/sensor.o 

C_DEPS += \
./Core/drivers/sensor/bmp280.d \
./Core/drivers/sensor/sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/drivers/sensor/%.o Core/drivers/sensor/%.su Core/drivers/sensor/%.cyclo: ../Core/drivers/sensor/%.c Core/drivers/sensor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Agung Ibnu/Documents/Stm32_Development/optimize_training/Core/drivers/sensor" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-drivers-2f-sensor

clean-Core-2f-drivers-2f-sensor:
	-$(RM) ./Core/drivers/sensor/bmp280.cyclo ./Core/drivers/sensor/bmp280.d ./Core/drivers/sensor/bmp280.o ./Core/drivers/sensor/bmp280.su ./Core/drivers/sensor/sensor.cyclo ./Core/drivers/sensor/sensor.d ./Core/drivers/sensor/sensor.o ./Core/drivers/sensor/sensor.su

.PHONY: clean-Core-2f-drivers-2f-sensor

