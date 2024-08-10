################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/MPUXX50.c \
../Core/Lib/i2c.c 

OBJS += \
./Core/Lib/MPUXX50.o \
./Core/Lib/i2c.o 

C_DEPS += \
./Core/Lib/MPUXX50.d \
./Core/Lib/i2c.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/%.o Core/Lib/%.su Core/Lib/%.cyclo: ../Core/Lib/%.c Core/Lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I"C:/ENCODER/Core/Lib" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib

clean-Core-2f-Lib:
	-$(RM) ./Core/Lib/MPUXX50.cyclo ./Core/Lib/MPUXX50.d ./Core/Lib/MPUXX50.o ./Core/Lib/MPUXX50.su ./Core/Lib/i2c.cyclo ./Core/Lib/i2c.d ./Core/Lib/i2c.o ./Core/Lib/i2c.su

.PHONY: clean-Core-2f-Lib

