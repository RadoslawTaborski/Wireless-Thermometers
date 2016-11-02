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
#define TIMEOUT 200; //ustawiamy zmienn¹ z wartoœci¹ timeout'u - czasu po którym uznamy, ¿e urz¹dzenie nie odpowiada
#define MASTER_ADDR 0xFF //adres uk³adu master

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
//Kod dla uk³adu MASTER
//--------------------------------------------------------------------------------------------------------
int main(void) {
	//setAllPins();
	initUART(MYUBRR); //inicjalizacja USART
	initTactSwitchAddSensor();
	initTactSwitchResetEeprom();
	initSPI(); //inicjalizacja magistrali SPI
	initRFM(); //inicjalizacja uk³ad RFM12B
	initCtcTimer0(16); //inicjalizacja timera0 w trybie CTC, czas przerwania ~1ms;

	char text[100];
	uint8_t size = getHexFromEeprom();
	uint8_t *sensorsAddresses = createAddressArray(size);
	uint8_t cur_meas = 0; //zmienna informuj¹ca o aktualnie wybranym uk³adzie
	uint8_t option = 0;
	Rfm_xmit(SYNC_PATTERN | MASTER_ADDR); //ustawiamy programowalny bajt synchronizacji

	sei();
	//w³¹czamy przerwania
	uartSendString("\r\n\r\nRFM12B - MASTER\r\n"); //wyœwietlamy powitanie

	while (1) {
		//za ka¿dym razem zmieniamy aktywny uk³ad.
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

			//przygotowujemy i wysy³amy zapytanie o temperaturê do jednego z uk³adów
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
		//sprawdzam czy wartoœæ jest wiêksza od 1 - dziêki temu jeœli po zakoñczeniu pêtli wartoœæ
		//timeout==1 to znaczy, ¿e uk³ad nie odpowiedzia³
		while (timeout > 1) {
			//odbieramy dane w znany nam sposób
			uint8_t tmp;
			if (Rfm_state_check() == RFM_RXC) {

				Rfm_rx_get(rx_buf, &tmp);
				Rfm_rx_prepare();
				//tu kolejna funkcja, która sprawdza crc odebranych danych, wraz z adresem docelowym
				ok = Rfm_rx_frame_good(rx_buf, &tmp, MASTER_ADDR);
				//wyœwietlamy dane
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
				//ustawiamy timeout na wartoœæ 1...
				timeout = 1;
			}
			//...dziêki czemu tu zostanie on zmieniony na wartoœæ 0, której nie osi¹gn¹³by gdyby pêtla zakoñczy³a siê
			//bez wykonania tamtej instrukcji if
			timeout--;
			//czekamy nieco miêdzy kolejnym sprawdzeniem, czy dane nie nadesz³y
			pause(1);
		}
		//tu sprawdzamy, czy dane zosta³y odebrane, czy te¿ nie by³o odpowiedzi i wyœwietlamy wtedy stosowny komunikat
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
		//wy³¹czamy odbiornik
		Rfm_stop();
		//i czekamy przed kolejnym zapytaniem o temperaturê
		pause(1000);
	}

	return 0;
}
#elif defined  SLAVE
//--------------------------------------------------------------------------------------------------------
//Kod dla uk³adu SLAVE
//--------------------------------------------------------------------------------------------------------

volatile uint8_t tick_flag = 0;

ISR(TIMER1_COMPA_vect) {				//przerwanie co ~750ms
	tick_flag = 1;//ustawia flagê odmierzenia tego czasu
}

int main(void) {
	setAllPins();
	initTactSwitchResetEeprom();
	initSPI();			//inicjalizacja magistrali SPI
	initRFM();//wstêpna konfiguracja uk³adu RFM12B
	initUART(MYUBRR);
	initCtcTimer1(11800);
	initCtcTimer0(16);
	resetDS18B20();

	uint8_t address = getHexFromEeprom();// domyœlnie ka¿dy uk³ad powinien miec 0x00 w EEPROM

	Rfm_xmit(SYNC_PATTERN | address);//ustawiamy programowalny bajt synchronizacji
	Rfm_rx_prepare();

	sei();
//w³¹czamy gobalny system przerwañ.
	uartSendString("\r\n\r\nRFM12B - SLAVE\r\n");//wyœwietlamy powitanie

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
			Rfm_rx_get(rx_buf, &tmp); //tu sprawdzamy poprawnoœæ crc odebranej ramki
			ok = Rfm_rx_frame_good(rx_buf, &tmp, address);
			if (ok) {
				//i jeœli prawid³owa to czy zawiera zapytanie o temperaturê (znak T)
				if (rx_buf[0] == 'T') {
					uartSendString(bufor);
					pause(5); //czekamy nieco - master musi przestawiæ siê na odbiór
					//i nadajemy ostatnio przygotowan¹ ramkê
					sendRFM12B(MASTER_ADDR, bufor);
				}
				if (rx_buf[0] == 'N') {
					pause(5);					//czekamy nieco - master musi przestawiæ siê na odbiór
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

		cli(); //przerwania wy³¹czamy na czas komunikacji 1-wire
		if (tick_flag) {				//wykonamy te instrukcje co 750ms
			tick_flag = 0;
			ok = temperatureMeasurment(bufor);
			Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor), MASTER_ADDR);//nastêpnie przygotowujemy ramkê wraz z sum¹ CRC
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
