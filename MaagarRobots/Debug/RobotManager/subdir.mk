################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../RobotManager/RobotManager.cpp 

OBJS += \
./RobotManager/RobotManager.o 

CPP_DEPS += \
./RobotManager/RobotManager.d 


# Each subdirectory must supply rules for building sources it contributes
RobotManager/%.o: ../RobotManager/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

