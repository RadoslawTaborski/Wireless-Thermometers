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

void addSensor(uint8_t *array, uint8_t size, char *result) { //TODO: coś tu sie sypie, całkiem możliwe wycieki pamięci :D
	if (size == 0)
		array[size] = 0x01;
	else
		array[size] = array[size - 1] + 0x01;

	uintToString(array[size],result);
}

/*
 * ustawienie pinów jako wyjścia ze stanem
 * niskim lub wysokim powoduje że nie działają jak anteny
 * dodatkowo podłącza restet do wewnętrznego pull-upa
 */
void setAllPins() {
	DDRB = 0x3F;
	DDRC = 0x3F;
	DDRD = 0xFF;
	PORTB = 0x00;
	PORTC = 0x40;
	PORTD = 0x00;
}

void uintToString(uint8_t address,char * result) {
	char str[4];
	itoa(address, str, 16);

	//char *result = malloc(sizeof(char) * 3);
	itoa(address, result, 16);

	if (address < 0x10) {
		char s[3] = "0";
		strcat(s, result);
		strncpy(result, s, 3);
	}
}

void resetDS18B20() {
	//wywołujemy funkcję pomiaru temperatury
	dallas_reset(); //reset magistrali 1-wire
	dallas_write_byte(SKIP_ROM_COMMAND); //pominięcie weryfikacji numeru
	dallas_write_byte(CONVERT_T_COMMAND); //zlecamy konwersję temperatury
}

uint8_t temperatureMeasurment(char * bufor) {
	uint8_t ok = 0;
	uint16_t measure;
	uint8_t subzero;
	uint8_t temp_int;
	uint16_t temp_fract;
	uint8_t buffer[9];

	ok = dallas_reset();				//reset magistrali 1-wire
	if (!ok) {				//jeśli nie odpowiedział termometr to wyświetlamy informację
		sprintf(bufor, "T.PRESENT");
		resetDS18B20();
		return 0;
	}

	dallas_reset();				//reset magistrali 1-wire
	dallas_write_byte(SKIP_ROM_COMMAND);				//pominięcie weryfikacji numeru
	dallas_write_byte(READ_SCRATCHPAD_COMMAND);				//zlecamy odczyt danych
	dallas_read_buffer(buffer, 9);				//odczytujemy dane z termometru
	if (buffer[8] != crc8(buffer, 8)) {	//sprawdzamy sumę kontrolną odczytu
		sprintf(bufor, "T.CRC8.ERR");
		resetDS18B20();
		return 0;
	}
	measure = (uint16_t) buffer[0] + (((uint16_t) buffer[1]) << 8);	//łączymy 2 bajty danych o temperaturze
	if (measure & 0x8000) {	//jeśli wynik jest ujemny to zapisujemy informację o znaku i konwertujemy liczbę kodu U2 na dodatnią
		subzero = 1;
		measure ^= 0xFFFF;
		measure += 1;
	} else {
		subzero = 0;
	}
	//rozdzielamy liczbę na część całkowitą i ułamkową
	temp_int = measure >> 4;
	temp_fract = (measure & 0x000F) * 625;
	sprintf(bufor, "%c%03d.%04d", subzero ? '-' : ' ', temp_int, temp_fract);
	resetDS18B20();

	return 1;
}

void sendRFM12B(uint8_t address, char * bufor) { //TODO: dodaj Rfm_tx_frame_prepare
	Rfm_stop();
	Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor), address);
	Rfm_tx_set((uint8_t*) bufor, strlen(bufor), address);
	Rfm_tx_start();
	while (Rfm_state_check() == RFM_TX)
		;
}

void receiveRFM12B(uint8_t receiverAddress, uint8_t *rx_buf, uint8_t *length) {
	uint8_t ok = 0;
	switch (Rfm_state_check()) {
		case RFM_RXC:
			Rfm_rx_get(rx_buf, length); //tu sprawdzamy poprawnosc crc odebranej ramki
			ok = Rfm_rx_frame_good(rx_buf, length, receiverAddress);
			if (!ok) { //i jezli prawidlowa to czy zawiera zapytanie o temperature (znak T)
				//uartSendString("blad");
				rx_buf[0] = 0;
				(*length) = 0;
			}
			Rfm_rx_prepare();
			break;
		case RFM_RXOVF:
			Rfm_rx_prepare();
			break;
		default:
			break;
	}
}

uint8_t waitForReceive(uint8_t receiverAddress, uint8_t *rx_buf, uint8_t *length, char sign, uint16_t timeout) {
	uint8_t ok = 0;
	Rfm_rx_prepare();
	while (timeout > 1) { //odbieramy dane w znany nam spos�b
		if (Rfm_state_check() == RFM_RXC) {
			Rfm_rx_get(rx_buf, length);
			ok = Rfm_rx_frame_good(rx_buf, length, receiverAddress); //tu kolejna funkcja, kt�ra sprawdza crc odebranych danych, wraz z adresem docelowym
			if (ok) { //wyświetlamy dane
				if (rx_buf[0] == sign) {
					Rfm_stop();
					return 1;
				}
			}
		}
		timeout--;
		//czekamy nieco mi�dzy kolejnym sprawdzeniem, czy dane nie nadesz�y
		pause(1);
	}
	//tu sprawdzamy, czy dane zosta�y odebrane, czy te� nie by�o odpowiedzi i wy�wietlamy wtedy stosowny komunikat
	//uartSendString("nie gra");
	rx_buf[0] = 0;
	(*length) = 0;
	Rfm_stop();
	return 0;
}
