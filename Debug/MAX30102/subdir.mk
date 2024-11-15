################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MAX30102/filters.c \
../MAX30102/heartRate.c \
../MAX30102/max30102.c \
../MAX30102/spo2.c 

OBJS += \
./MAX30102/filters.o \
./MAX30102/heartRate.o \
./MAX30102/max30102.o \
./MAX30102/spo2.o 

C_DEPS += \
./MAX30102/filters.d \
./MAX30102/heartRate.d \
./MAX30102/max30102.d \
./MAX30102/spo2.d 


# Each subdirectory must supply rules for building sources it contributes
MAX30102/%.o MAX30102/%.su MAX30102/%.cyclo: ../MAX30102/%.c MAX30102/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"/Users/vikrantthakur/Documents/IITD/course_work/Semester-4/Courses/Advanced Embedded/Assignment/L7_sdcard/sdcard_L7/MAX30102" -I"/Users/vikrantthakur/Documents/IITD/course_work/Semester-4/Courses/Advanced Embedded/Assignment/L7_sdcard/sdcard_L7/Log" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MAX30102

clean-MAX30102:
	-$(RM) ./MAX30102/filters.cyclo ./MAX30102/filters.d ./MAX30102/filters.o ./MAX30102/filters.su ./MAX30102/heartRate.cyclo ./MAX30102/heartRate.d ./MAX30102/heartRate.o ./MAX30102/heartRate.su ./MAX30102/max30102.cyclo ./MAX30102/max30102.d ./MAX30102/max30102.o ./MAX30102/max30102.su ./MAX30102/spo2.cyclo ./MAX30102/spo2.d ./MAX30102/spo2.o ./MAX30102/spo2.su

.PHONY: clean-MAX30102

