/*
 * other.c
 *
 *  Created on: 24 lip 2016
 *      Author: rados
 */

#include"other.h"

uint8_t * createAddressArray(uint8_t size) {
	uint8_t *array=(uint8_t*) malloc(255 * sizeof(uint8_t));
	uint8_t value = 0x01;
	for (int i = 0; i < size; ++i) {
		array[i] = value;
		value += 0x01;
	}
	return array;
}

char * addSensor(uint8_t *array, uint8_t size) {
	if (size == 0)
		array[size] = 0x01;
	else
		array[size] = array[size - 1] + 0x01;

	char *result;
	result=uintToString(array[size]);

	return result;
}

/*
 * ustawienie pin�w jako wyj�cia ze stanem
 * niskim lub wysokim powoduje �e nie dzia�aj� jak anteny
 * dodatkowo pod��cza restet do wewn�trznego pull-upa
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

char *uintToString(uint8_t address) {
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

void initDS18B20()
{
	//wywo�ujemy funkcj� pomiaru temperatury
	dallas_reset(); //reset magistrali 1-wire
	dallas_write_byte(SKIP_ROM_COMMAND); //pomini�cie weryfikacji numeru
	dallas_write_byte(CONVERT_T_COMMAND); //zlecamy konwersj� temperatury
}
