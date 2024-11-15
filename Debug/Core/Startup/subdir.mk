################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f722zetx.s 

OBJS += \
./Core/Startup/startup_stm32f722zetx.o 

S_DEPS += \
./Core/Startup/startup_stm32f722zetx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -DDEBUG -c -I"/Users/vikrantthakur/Documents/IITD/course_work/Semester-4/Courses/Advanced Embedded/Assignment/freertos_L7/Log" -I"/Users/vikrantthakur/Documents/IITD/course_work/Semester-4/Courses/Advanced Embedded/Assignment/freertos_L7/MAX30102" -I"/Users/vikrantthakur/Documents/IITD/course_work/Semester-4/Courses/Advanced Embedded/Assignment/freertos_L7/FATFS/App" -I"/Users/vikrantthakur/Documents/IITD/course_work/Semester-4/Courses/Advanced Embedded/Assignment/freertos_L7/Middlewares/Third_Party/FatFs/src" -I"/Users/vikrantthakur/Documents/IITD/course_work/Semester-4/Courses/Advanced Embedded/Assignment/freertos_L7/FATFS/Target" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f722zetx.d ./Core/Startup/startup_stm32f722zetx.o

.PHONY: clean-Core-2f-Startup

