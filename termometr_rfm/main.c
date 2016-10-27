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

//#define MASTER
#define SLAVE
#define TIMEOUT 200; //ustawiamy zmienn¹ z wartoœci¹ timeout'u - czasu po którym uznamy, ¿e urz¹dzenie nie odpowiada
#define MASTER_ADDR 0xFF //adres uk³adu master

char bufor_data[32];
char bufor[65];
uint8_t rx_buf[32];
uint8_t buffer[9];
uint8_t ok;
uint16_t measure;
uint8_t subzero;
uint8_t temp_int;
uint16_t temp_fract;
volatile uint16_t timer0;

ISR(TIMER0_COMPA_vect) { //przerwanie co ~1ms
	timer0++;
}
ISR(TIMER1_COMPA_vect) {				//przerwanie co ~750ms
	tick_flag = 1;				//ustawia flagê odmierzenia tego czasu
}

#ifdef MASTER
//--------------------------------------------------------------------------------------------------------
//Kod dla uk³adu MASTER
//--------------------------------------------------------------------------------------------------------
int main(void) {
	//setAllPins();
	initUART(MYUBRR);//inicjalizacja USART
	initTactSwitchAddSensor();
	initTactSwitchResetEeprom();
	initSPI();//inicjalizacja magistrali SPI
	initRFM();//inicjalizacja uk³ad RFM12B
	initCtcTimer0(16);//inicjalizacja timera0 w trybie CTC, czas przerwania ~1ms;

	char text[100];
	uint8_t size = getHexFromEeprom();
	uint8_t *sensorsAddresses = createAddressArray(size);
	uint8_t cur_meas = 0;//zmienna informuj¹ca o aktualnie wybranym uk³adzie

	Rfm_xmit(SYNC_PATTERN | MASTER_ADDR);//ustawiamy programowalny bajt synchronizacji

	sei();
	//w³¹czamy przerwania
	uartSendString("\r\n\r\nRFM12B - MASTER\r\n");//wyœwietlamy powitanie

	while (1) {
		//za ka¿dym razem zmieniamy aktywny uk³ad.
		if (clickedSwitch(ADD_SENSOR)) {
			char *pText;
			pText = addSensor(sensorsAddresses, size);
			pause(500);
			sprintf(bufor,"N%s",pText);
			sprintf(text, "\r\nDODAWANIE CZUJNIKA: %s\r\n", bufor);
			uartSendString(text);

			Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor), 0x00);
			Rfm_tx_set((uint8_t*) bufor, strlen(bufor), 0x00);
			Rfm_tx_start();
			while (Rfm_state_check() == RFM_TX);
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
						size++;
						char n_size[3];
						itoa(size, n_size, 16);
						if (size < 0x10) {
							char s[3] = "0";
							strcat(s, n_size);
							strncpy(n_size, s, 3);
						}
						writeStringToEeprom(n_size);
						sprintf(bufor, "%d: DODANO CZUJNIK", cur_meas + 1);
						uartSendString(bufor);
						//i informacjê o zgodnoœci sumy kontrolnej (normalnie danych ze z³ym crc wogóle byœmy nie obs³ugiwali)
					}
					uartSendString("\r\n");
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
				sprintf(bufor, "%d: BLAD DODANIA\r\n", cur_meas + 1);
				uartSendString(bufor);
			}
			//wy³¹czamy odbiornik
			Rfm_stop();
			//i czekamy przed kolejnym zapytaniem o temperaturê
			pause(1000);
		}

		if (clickedSwitch(CLEAN)) {
			writeStringToEeprom("00");
			size = getHexFromEeprom();
			cur_meas = 0;

			sprintf(text, "\r\nReset\r\nsize: %d\r\n", size);
			uartSendString(text);

			pause(200);
		}

		if (size > 0) {
			if (cur_meas == size - 1)
			cur_meas = 0;
			else
			cur_meas++;

			//przygotowujemy i wysy³amy zapytanie o temperaturê do jednego z uk³adów
			bufor[0] = 'T';
			bufor[1] = 0;
			//uart_send_s(" 1\r\n ");
			//to nowa funkcja, która dodaje do danych sumê kontroln¹ CRC
			Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor), sensorsAddresses[cur_meas]);
			//uart_send_s(" 2\r\n ");
			//wysy³anie ju¿ znamy
			Rfm_tx_set((uint8_t*) bufor, strlen(bufor), sensorsAddresses[cur_meas]);
			//uart_send_s(" 3\r\n ");
			Rfm_tx_start();
			//uart_send_s(" 4\r\n ");
			while (Rfm_state_check() == RFM_TX)
			;
			//uart_send_s(" 5\r\n ");
			//po wys³aniu przygotowujemy siê do odczytu
			Rfm_rx_prepare();
			//uart_send_s(" 6\r\n ");

			//ustawiamy zmienn¹ z wartoœci¹ timeout'u - czasu po którym uznamy, ¿e urz¹dzenie nie odpowiada
			uint8_t timeout = 200;

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
						sprintf(bufor, "%d: ", cur_meas + 1);
						uartSendString(bufor);
						rx_buf[tmp] = 0;
						uartSendString((char*) rx_buf);
						//i informacjê o zgodnoœci sumy kontrolnej (normalnie danych ze z³ym crc wogóle byœmy nie obs³ugiwali)
					}
					uartSendString("\r\n");
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
				sprintf(bufor, "%d: BRAK ODPOWIEDZI\r\n", cur_meas + 1); // TODO: chyba niepotrzebne
				uartSendString(bufor);
			}
			//wy³¹czamy odbiornik
			Rfm_stop();
			//i czekamy przed kolejnym zapytaniem o temperaturê
			pause(1000);
		}
	}
	return 0;
}
#elif defined  SLAVE
//--------------------------------------------------------------------------------------------------------
//Kod dla uk³adu SLAVE
//--------------------------------------------------------------------------------------------------------

volatile uint8_t tick_flag = 0;

int main(void) {
	setAllPins();
	initTactSwitchResetEeprom();
	initSPI();			//inicjalizacja magistrali SPI
	initRFM();			//wstêpna konfiguracja uk³adu RFM12B
	initUART(MYUBRR);
	initCtcTimer1(11800);
	initCtcTimer0(16);
	void initDS18B20();

	uint8_t address = getHexFromEeprom();			// domyœlnie ka¿dy uk³ad powinien miec 0x00 w EEPROM

	Rfm_xmit(SYNC_PATTERN | address); //ustawiamy programowalny bajt synchronizacji
	Rfm_rx_prepare();

	sei();
	//w³¹czamy gobalny system przerwañ.
	uartSendString("\r\n\r\nRFM12B - SLAVE\r\n"); //wyœwietlamy powitanie

	while (1) {
		uint8_t tmp;
		//odbieramy dane w znany nam sposób

		if (clickedSwitch(CLEAN)) {
			writeStringToEeprom("00");
			address = 0x00;
			pause(200);
			uartSendString("reset\r\n");
		}

		switch (Rfm_state_check()) {
			case RFM_RXC:
				Rfm_rx_get(rx_buf, &tmp);
				//tu sprawdzamy poprawnoœæ crc odebranej ramki
				ok = Rfm_rx_frame_good(rx_buf, &tmp, address);
				if (ok) {
					//i jeœli prawid³owa to czy zawiera zapytanie o temperaturê (znak T)
					if (rx_buf[0] == 'T') {
						uartSendString(bufor);
						pause(5); //czekamy nieco - master musi przestawiæ siê na odbiór
						//i nadajemy ostatnio przygotowan¹ ramkê
						Rfm_tx_set((uint8_t*) bufor, strlen(bufor), MASTER_ADDR);
						Rfm_tx_start();
						while (Rfm_state_check() == RFM_TX)
							;
					}
					if (rx_buf[0] == 'N') {
						pause(5);					//czekamy nieco - master musi przestawiæ siê na odbiór
						char newAddress[3];
						sprintf(newAddress, "%s", (rx_buf + 1));
						writeStringToEeprom(newAddress);
						address = (uint8_t) strtol(newAddress, NULL, 16);

						Rfm_tx_set((uint8_t*) bufor, strlen(bufor), MASTER_ADDR);
						Rfm_tx_start();
						while (Rfm_state_check() == RFM_TX)
							;
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

		//przerwania wy³¹czamy na czas komunikacji 1-wire
		cli();
		if (tick_flag) {				//wykonamy te instrukcje co 750ms
			tick_flag = 0;
			ok = dallas_reset();				//reset magistrali 1-wire
			if (!ok) {				//jeœli nie odpowiedzia³ ¿aden termometr to wyœwietlamy informacjê
				//w ka¿dej sytuacji przygotowujemy stosown¹ informacjê
				sprintf(bufor, "T.PRESENT");
				//nastêpnie przygotowujemy ramkê wraz z sum¹ CRC
				Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor),
				MASTER_ADDR);

				dallas_reset();
				dallas_write_byte(SKIP_ROM_COMMAND);				//pominiêcie weryfikacji numeru
				dallas_write_byte(CONVERT_T_COMMAND);				//zlecamy konwersjê temperatury
				continue;
			}

			dallas_reset();				//reset magistrali 1-wire
			dallas_write_byte(SKIP_ROM_COMMAND);				//pominiêcie weryfikacji numeru
			dallas_write_byte(READ_SCRATCHPAD_COMMAND);				//zlecamy odczyt danych
			dallas_read_buffer(buffer, 9);				//odczytujemy dane z termometru
			if (buffer[8] != crc8(buffer, 8)) {	//sprawdzamy sumê kontroln¹ odczytu
				sprintf(bufor, "T.CRC8.ERR");

				Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor),
				MASTER_ADDR);

				dallas_reset();
				dallas_write_byte(SKIP_ROM_COMMAND);	//pominiêcie weryfikacji numeru
				dallas_write_byte(CONVERT_T_COMMAND);	//zlecamy konwersjê temperatury
				continue;
			}
			measure = (uint16_t) buffer[0] + (((uint16_t) buffer[1]) << 8);	//³¹czymy 2 bajty danych o temperaturze
			if (measure & 0x8000) {	//jeœli wynik jest ujemny to zapisujemy informacjê o znaku i konwertujemy liczbê kodu U2 na dodatni¹
				subzero = 1;
				measure ^= 0xFFFF;
				measure += 1;
			} else {
				subzero = 0;
			}
			//rozdzielamy liczbê na czêœæ ca³kowit¹ i u³amkow¹
			temp_int = measure >> 4;
			temp_fract = (measure & 0x000F) * 625;
			sprintf(bufor, "%c%03d.%04d", subzero ? '-' : ' ', temp_int, temp_fract);

			Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor), MASTER_ADDR);

			dallas_reset();
			dallas_write_byte(SKIP_ROM_COMMAND);	//pominiêcie weryfikacji numeru
			dallas_write_byte(CONVERT_T_COMMAND);	//zlecamy konwersjê temperatury
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
