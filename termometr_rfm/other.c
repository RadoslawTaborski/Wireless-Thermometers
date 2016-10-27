/*
 * other.c
 *
 *  Created on: 24 lip 2016
 *      Author: rados
 */

#include"other.h"

uint8_t * createAddressArray(uint8_t size) {
	uint8_t *array = (uint8_t*) malloc(255 * sizeof(uint8_t));
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
	result = uintToString(array[size]);

	return result;
}

/*
 * ustawienie pin�w jako wyj�cia ze stanem
 * niskim lub wysokim powoduje �e nie dzia�aj� jak anteny
 * dodatkowo pod��cza restet do wewn�trznego pull-upa
 */
void setAllPins() {
	DDRB = 0x3F;
	DDRC = 0x3F;
	DDRD = 0xFF;
	PORTB = 0x00;
	PORTC = 0x40;
	PORTD = 0x00;
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

void resetDS18B20() {
	//wywo�ujemy funkcj� pomiaru temperatury
	dallas_reset(); //reset magistrali 1-wire
	dallas_write_byte(SKIP_ROM_COMMAND); //pomini�cie weryfikacji numeru
	dallas_write_byte(CONVERT_T_COMMAND); //zlecamy konwersj� temperatury
}

uint8_t temperatureMeasurment(char * bufor) {
	uint8_t ok = 0;
	uint16_t measure;
	uint8_t subzero;
	uint8_t temp_int;
	uint16_t temp_fract;
	uint8_t buffer[9];

	ok = dallas_reset();				//reset magistrali 1-wire
	if (!ok) {				//je�li nie odpowiedzia� termometr to wy�wietlamy informacj�
		sprintf(bufor, "T.PRESENT");
		resetDS18B20();
		return 0;
	}

	dallas_reset();				//reset magistrali 1-wire
	dallas_write_byte(SKIP_ROM_COMMAND);				//pomini�cie weryfikacji numeru
	dallas_write_byte(READ_SCRATCHPAD_COMMAND);				//zlecamy odczyt danych
	dallas_read_buffer(buffer, 9);				//odczytujemy dane z termometru
	if (buffer[8] != crc8(buffer, 8)) {	//sprawdzamy sum� kontroln� odczytu
		sprintf(bufor, "T.CRC8.ERR");
		resetDS18B20();
		return 0;
	}
	measure = (uint16_t) buffer[0] + (((uint16_t) buffer[1]) << 8);	//��czymy 2 bajty danych o temperaturze
	if (measure & 0x8000) {	//je�li wynik jest ujemny to zapisujemy informacj� o znaku i konwertujemy liczb� kodu U2 na dodatni�
		subzero = 1;
		measure ^= 0xFFFF;
		measure += 1;
	} else {
		subzero = 0;
	}
	//rozdzielamy liczb� na cz�� ca�kowit� i u�amkow�
	temp_int = measure >> 4;
	temp_fract = (measure & 0x000F) * 625;
	sprintf(bufor, "%c%03d.%04d", subzero ? '-' : ' ', temp_int, temp_fract);
	resetDS18B20();

	return 1;
}
