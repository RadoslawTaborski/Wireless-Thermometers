################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
termometr_rfm.lss \

FLASH_IMAGE += \
termometr_rfm.hex \

EEPROM_IMAGE += \
termometr_rfm.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: termometr_rfm.elf secondary-outputs

# Tool invocations
termometr_rfm.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,termometr_rfm.map -mmcu=atmega88pa -o "termometr_rfm.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

termometr_rfm.lss: termometr_rfm.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S termometr_rfm.elf  >"termometr_rfm.lss"
	@echo 'Finished building: $@'
	@echo ' '

termometr_rfm.hex: termometr_rfm.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex termometr_rfm.elf  "termometr_rfm.hex"
	@echo 'Finished building: $@'
	@echo ' '

termometr_rfm.eep: termometr_rfm.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex termometr_rfm.elf  "termometr_rfm.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: termometr_rfm.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega88pa termometr_rfm.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) termometr_rfm.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
