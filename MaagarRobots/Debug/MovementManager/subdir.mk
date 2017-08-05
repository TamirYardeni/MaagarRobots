################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../MovementManager/MovementManager.cpp 

OBJS += \
./MovementManager/MovementManager.o 

CPP_DEPS += \
./MovementManager/MovementManager.d 


# Each subdirectory must supply rules for building sources it contributes
MovementManager/%.o: ../MovementManager/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


