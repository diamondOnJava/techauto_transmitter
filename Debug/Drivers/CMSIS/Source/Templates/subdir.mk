################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/Source/Templates/system_stm32g0xx.c 

OBJS += \
./Drivers/CMSIS/Source/Templates/system_stm32g0xx.o 

C_DEPS += \
./Drivers/CMSIS/Source/Templates/system_stm32g0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/Source/Templates/%.o Drivers/CMSIS/Source/Templates/%.su Drivers/CMSIS/Source/Templates/%.cyclo: ../Drivers/CMSIS/Source/Templates/%.c Drivers/CMSIS/Source/Templates/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G031xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-Source-2f-Templates

clean-Drivers-2f-CMSIS-2f-Source-2f-Templates:
	-$(RM) ./Drivers/CMSIS/Source/Templates/system_stm32g0xx.cyclo ./Drivers/CMSIS/Source/Templates/system_stm32g0xx.d ./Drivers/CMSIS/Source/Templates/system_stm32g0xx.o ./Drivers/CMSIS/Source/Templates/system_stm32g0xx.su

.PHONY: clean-Drivers-2f-CMSIS-2f-Source-2f-Templates

