/*
 * Eeprom.c
 *
 *  Created on: 27 paü 2016
 *      Author: rados
 */

#include "eeprom.h"

void saveEeprom(unsigned int uiAddress, unsigned char ucData) {
	/* Wait for completion of previous write */
	while (EECR & (1 << EEPE))
		;
	/* Set up address and Data Registers */
	EEAR = uiAddress;
	EEDR = ucData;
	/* Write logical one to EEMPE */
	EECR |= (1 << EEMPE);
	/* Start eeprom write by setting EEPE */
	EECR |= (1 << EEPE);
}

unsigned char readEeprom(unsigned int uiAddress) {
	/* Wait for completion of previous write */
	while (EECR & (1 << EEPE))
		;
	/* Set up address register */
	EEAR = uiAddress;
	/* Start eeprom read by writing EERE */
	EECR |= (1 << EERE);
	/* Return data from Data Register */
	return EEDR;
}

void readStringFromEeprom(char * address, int length) {
	int licz = 0;
	address[length - 1] = '\0';
	for (licz = 0; licz < length - 1; ++licz)
		address[licz] = readEeprom(licz);
}

void writeStringToEeprom(char *address) {
	unsigned int licz = 0;

	for (licz = 0; licz < strlen(address); licz++)
		saveEeprom(licz, address[licz]);
}

uint8_t getHexFromEeprom()
{
	char text[3];
	readStringFromEeprom(text, 3);
	uint8_t number = (uint8_t) strtol(text, NULL, 16);

	return number;
}
