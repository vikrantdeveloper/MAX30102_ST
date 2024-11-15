################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Log/erlog.c 

OBJS += \
./Log/erlog.o 

C_DEPS += \
./Log/erlog.d 


# Each subdirectory must supply rules for building sources it contributes
Log/%.o Log/%.su Log/%.cyclo: ../Log/%.c Log/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I"/Users/vikrantthakur/Documents/IITD/course_work/Semester-4/Courses/Advanced Embedded/Assignment/freertos_L7/Log" -I"/Users/vikrantthakur/Documents/IITD/course_work/Semester-4/Courses/Advanced Embedded/Assignment/freertos_L7/MAX30102" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"/Users/vikrantthakur/Documents/IITD/course_work/Semester-4/Courses/Advanced Embedded/Assignment/freertos_L7/Middlewares/Third_Party/FatFs/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Log

clean-Log:
	-$(RM) ./Log/erlog.cyclo ./Log/erlog.d ./Log/erlog.o ./Log/erlog.su

.PHONY: clean-Log

