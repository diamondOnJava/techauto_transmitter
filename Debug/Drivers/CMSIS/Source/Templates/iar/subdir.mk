################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Drivers/CMSIS/Source/Templates/iar/startup_stm32g030xx.s \
../Drivers/CMSIS/Source/Templates/iar/startup_stm32g031xx.s \
../Drivers/CMSIS/Source/Templates/iar/startup_stm32g041xx.s \
../Drivers/CMSIS/Source/Templates/iar/startup_stm32g050xx.s \
../Drivers/CMSIS/Source/Templates/iar/startup_stm32g051xx.s \
../Drivers/CMSIS/Source/Templates/iar/startup_stm32g061xx.s \
../Drivers/CMSIS/Source/Templates/iar/startup_stm32g070xx.s \
../Drivers/CMSIS/Source/Templates/iar/startup_stm32g071xx.s \
../Drivers/CMSIS/Source/Templates/iar/startup_stm32g081xx.s \
../Drivers/CMSIS/Source/Templates/iar/startup_stm32g0b0xx.s \
../Drivers/CMSIS/Source/Templates/iar/startup_stm32g0b1xx.s \
../Drivers/CMSIS/Source/Templates/iar/startup_stm32g0c1xx.s 

OBJS += \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g030xx.o \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g031xx.o \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g041xx.o \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g050xx.o \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g051xx.o \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g061xx.o \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g070xx.o \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g071xx.o \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g081xx.o \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g0b0xx.o \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g0b1xx.o \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g0c1xx.o 

S_DEPS += \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g030xx.d \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g031xx.d \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g041xx.d \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g050xx.d \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g051xx.d \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g061xx.d \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g070xx.d \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g071xx.d \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g081xx.d \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g0b0xx.d \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g0b1xx.d \
./Drivers/CMSIS/Source/Templates/iar/startup_stm32g0c1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/Source/Templates/iar/%.o: ../Drivers/CMSIS/Source/Templates/iar/%.s Drivers/CMSIS/Source/Templates/iar/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m0plus -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-Drivers-2f-CMSIS-2f-Source-2f-Templates-2f-iar

clean-Drivers-2f-CMSIS-2f-Source-2f-Templates-2f-iar:
	-$(RM) ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g030xx.d ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g030xx.o ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g031xx.d ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g031xx.o ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g041xx.d ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g041xx.o ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g050xx.d ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g050xx.o ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g051xx.d ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g051xx.o ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g061xx.d ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g061xx.o ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g070xx.d ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g070xx.o ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g071xx.d ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g071xx.o ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g081xx.d ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g081xx.o ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g0b0xx.d ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g0b0xx.o ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g0b1xx.d ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g0b1xx.o ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g0c1xx.d ./Drivers/CMSIS/Source/Templates/iar/startup_stm32g0c1xx.o

.PHONY: clean-Drivers-2f-CMSIS-2f-Source-2f-Templates-2f-iar

