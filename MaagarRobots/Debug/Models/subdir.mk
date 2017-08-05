################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Models/Grid.cpp \
../Models/Map.cpp \
../Models/Node.cpp 

OBJS += \
./Models/Grid.o \
./Models/Map.o \
./Models/Node.o 

CPP_DEPS += \
./Models/Grid.d \
./Models/Map.d \
./Models/Node.d 


# Each subdirectory must supply rules for building sources it contributes
Models/%.o: ../Models/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


