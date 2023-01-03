################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/AERS.c \
../Core/Src/AHRS.c \
../Core/Src/Baro.c \
../Core/Src/dma.c \
../Core/Src/filters.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/ibus.c \
../Core/Src/imu.c \
../Core/Src/main.c \
../Core/Src/opticalflow.c \
../Core/Src/spi.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/AERS.o \
./Core/Src/AHRS.o \
./Core/Src/Baro.o \
./Core/Src/dma.o \
./Core/Src/filters.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/ibus.o \
./Core/Src/imu.o \
./Core/Src/main.o \
./Core/Src/opticalflow.o \
./Core/Src/spi.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/AERS.d \
./Core/Src/AHRS.d \
./Core/Src/Baro.d \
./Core/Src/dma.d \
./Core/Src/filters.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/ibus.d \
./Core/Src/imu.d \
./Core/Src/main.d \
./Core/Src/opticalflow.d \
./Core/Src/spi.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -c -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

