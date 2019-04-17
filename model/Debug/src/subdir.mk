################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/base_rock_sample.cpp \
../src/main.cpp \
../src/rock_sample.cpp \
../src/simulator.cpp 

OBJS += \
./src/base_rock_sample.o \
./src/main.o \
./src/rock_sample.o \
./src/simulator.o 

CPP_DEPS += \
./src/base_rock_sample.d \
./src/main.d \
./src/rock_sample.d \
./src/simulator.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/home/fanj2/eclipse-workspace/model/include" -I/home/fanj2/Documents/despot-API_redesign/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


