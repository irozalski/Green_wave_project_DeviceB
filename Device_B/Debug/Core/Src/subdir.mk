################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ir_receiver.c \
../Core/Src/ir_sender.c \
../Core/Src/main.c \
../Core/Src/nRF24.c \
../Core/Src/random_message.c \
../Core/Src/ring_buffer.c \
../Core/Src/rsa_driver.c \
../Core/Src/rsa_keys.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/ir_receiver.o \
./Core/Src/ir_sender.o \
./Core/Src/main.o \
./Core/Src/nRF24.o \
./Core/Src/random_message.o \
./Core/Src/ring_buffer.o \
./Core/Src/rsa_driver.o \
./Core/Src/rsa_keys.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/ir_receiver.d \
./Core/Src/ir_sender.d \
./Core/Src/main.d \
./Core/Src/nRF24.d \
./Core/Src/random_message.d \
./Core/Src/ring_buffer.d \
./Core/Src/rsa_driver.d \
./Core/Src/rsa_keys.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Igor/Desktop/STM32CubeExpansion_Crypto_V3.1.0/Fw_Crypto/STM32F4/Middlewares/ST/STM32_Cryptographic/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ir_receiver.d ./Core/Src/ir_receiver.o ./Core/Src/ir_receiver.su ./Core/Src/ir_sender.d ./Core/Src/ir_sender.o ./Core/Src/ir_sender.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/nRF24.d ./Core/Src/nRF24.o ./Core/Src/nRF24.su ./Core/Src/random_message.d ./Core/Src/random_message.o ./Core/Src/random_message.su ./Core/Src/ring_buffer.d ./Core/Src/ring_buffer.o ./Core/Src/ring_buffer.su ./Core/Src/rsa_driver.d ./Core/Src/rsa_driver.o ./Core/Src/rsa_driver.su ./Core/Src/rsa_keys.d ./Core/Src/rsa_keys.o ./Core/Src/rsa_keys.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

