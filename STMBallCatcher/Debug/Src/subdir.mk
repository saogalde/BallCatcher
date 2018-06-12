################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/ballCatcher.c \
../Src/lcd_i2c.c \
../Src/main.c \
../Src/stepperControl.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/ballCatcher.o \
./Src/lcd_i2c.o \
./Src/main.o \
./Src/stepperControl.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/ballCatcher.d \
./Src/lcd_i2c.d \
./Src/main.d \
./Src/stepperControl.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F446xx -I"/Users/saogalde/eclipse-workspace/BallCatcher/STMBallCatcher/Inc" -I"/Users/saogalde/eclipse-workspace/BallCatcher/STMBallCatcher/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/Users/saogalde/eclipse-workspace/BallCatcher/STMBallCatcher/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/Users/saogalde/eclipse-workspace/BallCatcher/STMBallCatcher/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/Users/saogalde/eclipse-workspace/BallCatcher/STMBallCatcher/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


