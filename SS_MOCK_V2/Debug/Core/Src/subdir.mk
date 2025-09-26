################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/LTR-329.c \
../Core/Src/MCP9808.c \
../Core/Src/bno055.c \
../Core/Src/main.c \
../Core/Src/motordrivers.c \
../Core/Src/motorencoder.c \
../Core/Src/pid.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c 

OBJS += \
./Core/Src/LTR-329.o \
./Core/Src/MCP9808.o \
./Core/Src/bno055.o \
./Core/Src/main.o \
./Core/Src/motordrivers.o \
./Core/Src/motorencoder.o \
./Core/Src/pid.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o 

C_DEPS += \
./Core/Src/LTR-329.d \
./Core/Src/MCP9808.d \
./Core/Src/bno055.d \
./Core/Src/main.d \
./Core/Src/motordrivers.d \
./Core/Src/motorencoder.d \
./Core/Src/pid.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/LTR-329.cyclo ./Core/Src/LTR-329.d ./Core/Src/LTR-329.o ./Core/Src/LTR-329.su ./Core/Src/MCP9808.cyclo ./Core/Src/MCP9808.d ./Core/Src/MCP9808.o ./Core/Src/MCP9808.su ./Core/Src/bno055.cyclo ./Core/Src/bno055.d ./Core/Src/bno055.o ./Core/Src/bno055.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motordrivers.cyclo ./Core/Src/motordrivers.d ./Core/Src/motordrivers.o ./Core/Src/motordrivers.su ./Core/Src/motorencoder.cyclo ./Core/Src/motorencoder.d ./Core/Src/motorencoder.o ./Core/Src/motorencoder.su ./Core/Src/pid.cyclo ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/pid.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su

.PHONY: clean-Core-2f-Src

