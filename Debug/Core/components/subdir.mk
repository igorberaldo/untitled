################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/components/bmx160.c \
../Core/components/bt_assobio.c \
../Core/components/nrf52832_api.c 

C_DEPS += \
./Core/components/bmx160.d \
./Core/components/bt_assobio.d \
./Core/components/nrf52832_api.d 

OBJS += \
./Core/components/bmx160.o \
./Core/components/bt_assobio.o \
./Core/components/nrf52832_api.o 


# Each subdirectory must supply rules for building sources it contributes
Core/components/%.o Core/components/%.su Core/components/%.cyclo: ../Core/components/%.c Core/components/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L072xx -c -I../Core/Inc -I"/Users/igorberaldo/dev/core_stm32/Core/components" -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-components

clean-Core-2f-components:
	-$(RM) ./Core/components/bmx160.cyclo ./Core/components/bmx160.d ./Core/components/bmx160.o ./Core/components/bmx160.su ./Core/components/bt_assobio.cyclo ./Core/components/bt_assobio.d ./Core/components/bt_assobio.o ./Core/components/bt_assobio.su ./Core/components/nrf52832_api.cyclo ./Core/components/nrf52832_api.d ./Core/components/nrf52832_api.o ./Core/components/nrf52832_api.su

.PHONY: clean-Core-2f-components

