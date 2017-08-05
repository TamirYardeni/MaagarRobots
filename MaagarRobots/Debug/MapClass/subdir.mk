################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../MapClass/MapClass.cpp 

OBJS += \
./MapClass/MapClass.o 

CPP_DEPS += \
./MapClass/MapClass.d 


# Each subdirectory must supply rules for building sources it contributes
MapClass/%.o: ../MapClass/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


