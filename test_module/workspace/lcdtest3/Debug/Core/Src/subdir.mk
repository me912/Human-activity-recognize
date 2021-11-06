################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fonts.c \
../Core/Src/kalman.c \
../Core/Src/main.c \
../Core/Src/max30100.c \
../Core/Src/ssd1306.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/test.c 

OBJS += \
./Core/Src/fonts.o \
./Core/Src/kalman.o \
./Core/Src/main.o \
./Core/Src/max30100.o \
./Core/Src/ssd1306.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/test.o 

C_DEPS += \
./Core/Src/fonts.d \
./Core/Src/kalman.d \
./Core/Src/main.d \
./Core/Src/max30100.d \
./Core/Src/ssd1306.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/test.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32F411xE -I"C:/Users/LENOVO-PC/Documents/a/project1/test/workspace/lcdtest3/Core/Inc" -I"C:/Users/LENOVO-PC/Documents/a/project1/test/workspace/lcdtest3/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/LENOVO-PC/Documents/a/project1/test/workspace/lcdtest3/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/LENOVO-PC/Documents/a/project1/test/workspace/lcdtest3/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/LENOVO-PC/Documents/a/project1/test/workspace/lcdtest3/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


