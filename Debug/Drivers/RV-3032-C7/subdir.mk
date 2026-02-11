################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/RV-3032-C7/rtc_rv3032.c 

OBJS += \
./Drivers/RV-3032-C7/rtc_rv3032.o 

C_DEPS += \
./Drivers/RV-3032-C7/rtc_rv3032.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/RV-3032-C7/%.o Drivers/RV-3032-C7/%.su Drivers/RV-3032-C7/%.cyclo: ../Drivers/RV-3032-C7/%.c Drivers/RV-3032-C7/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G071xx -c -I../Core/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I../Drivers/RV-3032-C7 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-RV-2d-3032-2d-C7

clean-Drivers-2f-RV-2d-3032-2d-C7:
	-$(RM) ./Drivers/RV-3032-C7/rtc_rv3032.cyclo ./Drivers/RV-3032-C7/rtc_rv3032.d ./Drivers/RV-3032-C7/rtc_rv3032.o ./Drivers/RV-3032-C7/rtc_rv3032.su

.PHONY: clean-Drivers-2f-RV-2d-3032-2d-C7

