################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ACS712.c \
../Core/Src/FLASH_SECTOR_F4.c \
../Core/Src/PID.c \
../Core/Src/PayloadSequence.c \
../Core/Src/it.c \
../Core/Src/main.c \
../Core/Src/msp.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/ACS712.o \
./Core/Src/FLASH_SECTOR_F4.o \
./Core/Src/PID.o \
./Core/Src/PayloadSequence.o \
./Core/Src/it.o \
./Core/Src/main.o \
./Core/Src/msp.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/ACS712.d \
./Core/Src/FLASH_SECTOR_F4.d \
./Core/Src/PID.d \
./Core/Src/PayloadSequence.d \
./Core/Src/it.d \
./Core/Src/main.d \
./Core/Src/msp.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ACS712.d ./Core/Src/ACS712.o ./Core/Src/ACS712.su ./Core/Src/FLASH_SECTOR_F4.d ./Core/Src/FLASH_SECTOR_F4.o ./Core/Src/FLASH_SECTOR_F4.su ./Core/Src/PID.d ./Core/Src/PID.o ./Core/Src/PID.su ./Core/Src/PayloadSequence.d ./Core/Src/PayloadSequence.o ./Core/Src/PayloadSequence.su ./Core/Src/it.d ./Core/Src/it.o ./Core/Src/it.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/msp.d ./Core/Src/msp.o ./Core/Src/msp.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

