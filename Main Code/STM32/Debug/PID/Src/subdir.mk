################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../PID/Src/pid.c 

OBJS += \
./PID/Src/pid.o 

C_DEPS += \
./PID/Src/pid.d 


# Each subdirectory must supply rules for building sources it contributes
PID/Src/%.o PID/Src/%.su: ../PID/Src/%.c PID/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/Yu_Hao/Desktop/Y3S1/MDP/MDP22-23/Main Code/STM32/PID/Inc" -I"C:/Users/Yu_Hao/Desktop/Y3S1/MDP/MDP22-23/Main Code/STM32/PeripheralDriver/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-PID-2f-Src

clean-PID-2f-Src:
	-$(RM) ./PID/Src/pid.d ./PID/Src/pid.o ./PID/Src/pid.su

.PHONY: clean-PID-2f-Src

