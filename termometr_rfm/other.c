/*
 * other.c
 *
 *  Created on: 24 lip 2016
 *      Author: rados
 */

#include"other.h"

void initializeAddressArray(uint8_t * array, uint8_t size) {
	uint8_t value = 0x01;
	for (int i = 0; i < size; ++i) {
		array[i] = value;
		value += 0x01;
	}
}

char * addSensor(uint8_t *array, uint8_t size) {
	if (size == 0)
		array[size] = 0x01;
	else
		array[size] = array[size - 1] + 0x01;

	char *result;
	result=uint8_tToCharArray(array[size]);

	return result;
}

void EEPROM_zapisz(unsigned int uiAddress, unsigned char ucData) {
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

unsigned char EEPROM_odczyt(unsigned int uiAddress) {
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
		address[licz] = EEPROM_odczyt(licz);
}

void writeStringToEeprom(char *address) {
	unsigned int licz = 0;

	for (licz = 0; licz < strlen(address); licz++)
		EEPROM_zapisz(licz, address[licz]);
}

/*
 * ustawienie pinów jako wyjœcia ze stanem
 * niskim lub wysokim powoduje ¿e nie dzia³aj¹ jak anteny
 * dodatkowo pod³¹cza restet do wewnêtrznego pull-upa
 */
void setAllPins()
{
	DDRB=0x3F;
	DDRC=0x3F;
	DDRD=0xFF;
	PORTB=0x00;
	PORTC=0x40;
	PORTD=0x00;
}

char *uint8_tToCharArray(uint8_t address) {
	char str[4];
	itoa(address, str, 16);

	char *result = malloc(sizeof(char) * 3);
	itoa(address, result, 16);

	if (address < 0x10) {
		char s[3] = "0";
		strcat(s, result);
		strncpy(result, s, 3);
	}

	return result;
}

/*uint8_t ClickedSwitch(uint8_t button){
	if(! (PIND & button))
	{
		Pause(80);
		if(! (PIND & button)) return 1;
	}
	return 0;
}

void Pause(uint8_t ms){

}*/
