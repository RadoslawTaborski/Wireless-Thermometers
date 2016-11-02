#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "crc8.h"
#include "dallas_one_wire.h"
#include "eeprom.h"
#include "RFM12B.h"
#include "uart.h"
#include "other.h"
#include "tactSwitches.h"
#include "eeprom.h"
#include "timers.h"

#define MASTER
//#define SLAVE
#define TIMEOUT 200; //ustawiamy zmienn� z warto�ci� timeout'u - czasu po kt�rym uznamy, �e urz�dzenie nie odpowiada
#define MASTER_ADDR 0xFF //adres uk�adu master

char bufor_data[32];
char bufor[65];
uint8_t rx_buf[32];
uint8_t ok;
volatile uint16_t timer0;

ISR(TIMER0_COMPA_vect) { //przerwanie co ~1ms
	timer0++;
}

#ifdef MASTER
//--------------------------------------------------------------------------------------------------------
//Kod dla uk�adu MASTER
//--------------------------------------------------------------------------------------------------------
int main(void) {
	//setAllPins();
	initUART(MYUBRR); //inicjalizacja USART
	initTactSwitchAddSensor();
	initTactSwitchResetEeprom();
	initSPI(); //inicjalizacja magistrali SPI
	initRFM(); //inicjalizacja uk�ad RFM12B
	initCtcTimer0(16); //inicjalizacja timera0 w trybie CTC, czas przerwania ~1ms;

	char text[100];
	uint8_t size = getHexFromEeprom();
	uint8_t *sensorsAddresses = createAddressArray(size);
	uint8_t cur_meas = 0; //zmienna informuj�ca o aktualnie wybranym uk�adzie
	uint8_t option = 0;
	Rfm_xmit(SYNC_PATTERN | MASTER_ADDR); //ustawiamy programowalny bajt synchronizacji

	sei();
	//w��czamy przerwania
	uartSendString("\r\n\r\nRFM12B - MASTER\r\n"); //wy�wietlamy powitanie

	while (1) {
		//za ka�dym razem zmieniamy aktywny uk�ad.
		if (clickedSwitch(ADD_SENSOR)) {
			char *pText;
			pText = addSensor(sensorsAddresses, size);
			pause(500);
			sprintf(bufor, "%s", pText);
			option = 1;
		} else if (clickedSwitch(CLEAN)) {
			writeStringToEeprom("00");
			size = getHexFromEeprom();
			cur_meas = 0;

			sprintf(text, "\r\nReset\r\nsize: %d\r\n", size);
			uartSendString(text);
			option = 2;
			pause(200);
		} else if (size > 0) {
			if (cur_meas == size - 1)
				cur_meas = 0;
			else
				cur_meas++;

			//przygotowujemy i wysy�amy zapytanie o temperatur� do jednego z uk�ad�w
			bufor[0] = 'T';
			bufor[1] = 0;
			option = 3;
		}

		//sprintf(text, "\r\nDODAWANIE CZUJNIKA: %s\r\n", bufor);
		//uartSendString(text);
		Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor), 0x00);
		sendRFM12B(0x00, bufor);
		Rfm_rx_prepare();

		uint8_t timeout = TIMEOUT
		;
		//sprawdzam czy warto�� jest wi�ksza od 1 - dzi�ki temu je�li po zako�czeniu p�tli warto��
		//timeout==1 to znaczy, �e uk�ad nie odpowiedzia�
		while (timeout > 1) {
			//odbieramy dane w znany nam spos�b
			uint8_t tmp;
			if (Rfm_state_check() == RFM_RXC) {

				Rfm_rx_get(rx_buf, &tmp);
				Rfm_rx_prepare();
				//tu kolejna funkcja, kt�ra sprawdza crc odebranych danych, wraz z adresem docelowym
				ok = Rfm_rx_frame_good(rx_buf, &tmp, MASTER_ADDR);
				//wy�wietlamy dane
				if (ok) {
					switch (option) {
						case 1:
							size++;
							char* n_size;
							n_size = uintToString(size);
							writeStringToEeprom(n_size);
							sprintf(bufor, "Dodano nowy czujnik: %d: \r\n", cur_meas + 1);
							uartSendString(bufor);
							break;
						case 3:
							sprintf(bufor, "Adres: %d Temperatura: \r\n", cur_meas + 1);
							uartSendString(bufor);
							rx_buf[tmp] = 0;
							uartSendString((char*) rx_buf);
							break;
					}
				}
				//ustawiamy timeout na warto�� 1...
				timeout = 1;
			}
			//...dzi�ki czemu tu zostanie on zmieniony na warto�� 0, kt�rej nie osi�gn��by gdyby p�tla zako�czy�a si�
			//bez wykonania tamtej instrukcji if
			timeout--;
			//czekamy nieco mi�dzy kolejnym sprawdzeniem, czy dane nie nadesz�y
			pause(1);
		}
		//tu sprawdzamy, czy dane zosta�y odebrane, czy te� nie by�o odpowiedzi i wy�wietlamy wtedy stosowny komunikat
		if (timeout == 1) {
			switch (option) {
				case 1:
					sprintf(bufor, "Adres: %d - BLAD DODANIA\r\n", cur_meas + 1);
					break;
				case 3:
					sprintf(bufor, "Adres: %d - BRAK ODPOWIEDZI\r\n", cur_meas + 1);
					break;
			}
			uartSendString(bufor);
		}
		//wy��czamy odbiornik
		Rfm_stop();
		//i czekamy przed kolejnym zapytaniem o temperatur�
		pause(1000);
	}

	return 0;
}
#elif defined  SLAVE
//--------------------------------------------------------------------------------------------------------
//Kod dla uk�adu SLAVE
//--------------------------------------------------------------------------------------------------------

volatile uint8_t tick_flag = 0;

ISR(TIMER1_COMPA_vect) {				//przerwanie co ~750ms
	tick_flag = 1;//ustawia flag� odmierzenia tego czasu
}

int main(void) {
	setAllPins();
	initTactSwitchResetEeprom();
	initSPI();			//inicjalizacja magistrali SPI
	initRFM();//wst�pna konfiguracja uk�adu RFM12B
	initUART(MYUBRR);
	initCtcTimer1(11800);
	initCtcTimer0(16);
	resetDS18B20();

	uint8_t address = getHexFromEeprom();// domy�lnie ka�dy uk�ad powinien miec 0x00 w EEPROM

	Rfm_xmit(SYNC_PATTERN | address);//ustawiamy programowalny bajt synchronizacji
	Rfm_rx_prepare();

	sei();
//w��czamy gobalny system przerwa�.
	uartSendString("\r\n\r\nRFM12B - SLAVE\r\n");//wy�wietlamy powitanie

	while (1) {
		uint8_t tmp;

		if (clickedSwitch(CLEAN)) {
			writeStringToEeprom("00");
			address = 0x00;
			pause(200);
			uartSendString("reset\r\n");
		}

		switch (Rfm_state_check()) {
			case RFM_RXC:
			Rfm_rx_get(rx_buf, &tmp); //tu sprawdzamy poprawno�� crc odebranej ramki
			ok = Rfm_rx_frame_good(rx_buf, &tmp, address);
			if (ok) {
				//i je�li prawid�owa to czy zawiera zapytanie o temperatur� (znak T)
				if (rx_buf[0] == 'T') {
					uartSendString(bufor);
					pause(5); //czekamy nieco - master musi przestawi� si� na odbi�r
					//i nadajemy ostatnio przygotowan� ramk�
					sendRFM12B(MASTER_ADDR, bufor);
				}
				if (rx_buf[0] == 'N') {
					pause(5);					//czekamy nieco - master musi przestawi� si� na odbi�r
					char newAddress[3];
					sprintf(newAddress, "%s", (rx_buf + 1));
					writeStringToEeprom(newAddress);
					address = (uint8_t) strtol(newAddress, NULL, 16);
					sendRFM12B(MASTER_ADDR, bufor);
				}
			}
			Rfm_rx_prepare();
			break;
			case RFM_RXOVF:
			Rfm_rx_prepare();
			break;
			default:
			break;
		}

		cli(); //przerwania wy��czamy na czas komunikacji 1-wire
		if (tick_flag) {				//wykonamy te instrukcje co 750ms
			tick_flag = 0;
			ok = temperatureMeasurment(bufor);
			Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor), MASTER_ADDR);//nast�pnie przygotowujemy ramk� wraz z sum� CRC
			if (!ok) { //TODO: sprawdzic czy potrzebne
				continue;
			}
		}
		sei();
	}

	return 0;
}
#else
int main(void) {
	writeStringToEeprom("00");
	return 0;
}

#endif
