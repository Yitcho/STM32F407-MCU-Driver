################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/ds1307.c \
../BSP/i2c-lcd.c 

OBJS += \
./BSP/ds1307.o \
./BSP/i2c-lcd.o 

C_DEPS += \
./BSP/ds1307.d \
./BSP/i2c-lcd.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/%.o BSP/%.su BSP/%.cyclo: ../BSP/%.c BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/Mahdi Kaffel/Desktop/MCU1/stm32f4xx_drivers/BSP" -I"C:/Users/Mahdi Kaffel/Desktop/MCU1/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BSP

clean-BSP:
	-$(RM) ./BSP/ds1307.cyclo ./BSP/ds1307.d ./BSP/ds1307.o ./BSP/ds1307.su ./BSP/i2c-lcd.cyclo ./BSP/i2c-lcd.d ./BSP/i2c-lcd.o ./BSP/i2c-lcd.su

.PHONY: clean-BSP

