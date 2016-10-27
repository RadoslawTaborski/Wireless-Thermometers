################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../RFM12B.c \
../crc8.c \
../dallas_one_wire.c \
../ds18b20.c \
../eeprom.c \
../main.c \
../other.c \
../tactSwitches.c \
../timers.c \
../uart.c 

OBJS += \
./RFM12B.o \
./crc8.o \
./dallas_one_wire.o \
./ds18b20.o \
./eeprom.o \
./main.o \
./other.o \
./tactSwitches.o \
./timers.o \
./uart.o 

C_DEPS += \
./RFM12B.d \
./crc8.d \
./dallas_one_wire.d \
./ds18b20.d \
./eeprom.d \
./main.d \
./other.d \
./tactSwitches.d \
./timers.d \
./uart.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega88pa -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


