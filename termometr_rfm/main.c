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

	Rfm_xmit(SYNC_PATTERN | MASTER_ADDR); //ustawiamy programowalny bajt synchronizacji

	sei();
	//w��czamy przerwania
	uartSendString("\r\n\r\nRFM12B - MASTER\r\n"); //wy�wietlamy powitanie

	while (1) {
		//za ka�dym razem zmieniamy aktywny uk�ad.
		if (!(PIND & (1 << ADD_SENSOR))) {
			char *pText;
			pText = addSensor(sensorsAddresses, size);
			timer0 = 0;
			while (timer0 < 500)
				;
			bufor[0] = 'N';
			bufor[1] = pText[0];
			bufor[2] = pText[1];
			bufor[3] = 0;
			sprintf(text, "\r\nDODAWANIE CZUJNIKA: %s\r\n", bufor);
			uartSendString(text);

			Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor), 0x00);
			Rfm_tx_set((uint8_t*) bufor, strlen(bufor), 0x00);
			Rfm_tx_start();
			while (Rfm_state_check() == RFM_TX)
				;
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
						size++;
						char n_size[3];
						itoa(size, n_size, 16);
						if (size < 0x10) {
							char s[3] = "0";
							strcat(s, n_size);
							strncpy(n_size, s, 3);
							//uart_send_s(result);
							//uart_send_s("\r\n");
						}
						writeStringToEeprom(n_size);
						sprintf(bufor, "%d: DODANO CZUJNIK", cur_meas + 1);
						uartSendString(bufor);
						//i informacj� o zgodno�ci sumy kontrolnej (normalnie danych ze z�ym crc wog�le by�my nie obs�ugiwali)
					}
					uartSendString("\r\n");
					//ustawiamy timeout na warto�� 1...
					timeout = 1;
				}
				//...dzi�ki czemu tu zostanie on zmieniony na warto�� 0, kt�rej nie osi�gn��by gdyby p�tla zako�czy�a si�
				//bez wykonania tamtej instrukcji if
				timeout--;
				//czekamy nieco mi�dzy kolejnym sprawdzeniem, czy dane nie nadesz�y
				timer0 = 0;
				while (timer0 < 1)
					;

			}
			//tu sprawdzamy, czy dane zosta�y odebrane, czy te� nie by�o odpowiedzi i wy�wietlamy wtedy stosowny komunikat
			if (timeout == 1) {
				sprintf(bufor, "%d: BLAD DODANIA\r\n", cur_meas + 1);
				uartSendString(bufor);
			}
			//wy��czamy odbiornik
			Rfm_stop();
			//i czekamy przed kolejnym zapytaniem o temperatur�
			timer0 = 0;
			while (timer0 < 5000)
				;
		}

		if (!(PIND & (1 << CLEAR))) {
			writeStringToEeprom("00");
			size = getHexFromEeprom();
			cur_meas = 0;

			sprintf(text, "\r\nReset\r\nsize: %d\r\n", size);
			uartSendString(text);

			timer0 = 0;
			while (timer0 < 200)
				;
		}

		if (size > 0) {
			if (cur_meas == size - 1)
				cur_meas = 0;
			else
				cur_meas++;

			//przygotowujemy i wysy�amy zapytanie o temperatur� do jednego z uk�ad�w
			bufor[0] = 'T';
			bufor[1] = 0;
			//uart_send_s(" 1\r\n ");
			//to nowa funkcja, kt�ra dodaje do danych sum� kontroln� CRC
			Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor), sensorsAddresses[cur_meas]);
			//uart_send_s(" 2\r\n ");
			//wysy�anie ju� znamy
			Rfm_tx_set((uint8_t*) bufor, strlen(bufor), sensorsAddresses[cur_meas]);
			//uart_send_s(" 3\r\n ");
			Rfm_tx_start();
			//uart_send_s(" 4\r\n ");
			while (Rfm_state_check() == RFM_TX)
				;
			//uart_send_s(" 5\r\n ");
			//po wys�aniu przygotowujemy si� do odczytu
			Rfm_rx_prepare();
			//uart_send_s(" 6\r\n ");

			//ustawiamy zmienn� z warto�ci� timeout'u - czasu po kt�rym uznamy, �e urz�dzenie nie odpowiada
			uint8_t timeout = 200;

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
						sprintf(bufor, "%d: ", cur_meas + 1);
						uartSendString(bufor);
						rx_buf[tmp] = 0;
						uartSendString((char*) rx_buf);
						//i informacj� o zgodno�ci sumy kontrolnej (normalnie danych ze z�ym crc wog�le by�my nie obs�ugiwali)
					}
					uartSendString("\r\n");
					//ustawiamy timeout na warto�� 1...
					timeout = 1;
				}
				//...dzi�ki czemu tu zostanie on zmieniony na warto�� 0, kt�rej nie osi�gn��by gdyby p�tla zako�czy�a si�
				//bez wykonania tamtej instrukcji if
				timeout--;
				//czekamy nieco mi�dzy kolejnym sprawdzeniem, czy dane nie nadesz�y
				timer0 = 0;
				while (timer0 < 1)
					;
				//_delay_us(250);

			}
			//tu sprawdzamy, czy dane zosta�y odebrane, czy te� nie by�o odpowiedzi i wy�wietlamy wtedy stosowny komunikat
			if (timeout == 1) {
				sprintf(bufor, "%d: BRAK ODPOWIEDZI\r\n", cur_meas + 1); // TODO: chyba niepotrzebne
				uartSendString(bufor);
			}
			//wy��czamy odbiornik
			Rfm_stop();
			//i czekamy przed kolejnym zapytaniem o temperatur�
			timer0 = 0;
			while (timer0 < 5000)
				;
			//_delay_ms(500);
		}
	}
	return 0;
}
#elif defined  SLAVE
//--------------------------------------------------------------------------------------------------------
//Kod dla uk�adu SLAVE
//--------------------------------------------------------------------------------------------------------

volatile uint8_t tick_flag = 0;

int main(void) {
	setAllPins();
	initTactSwitchResetEeprom();
	initSPI();			//inicjalizacja magistrali SPI
	initRFM();//wst�pna konfiguracja uk�adu RFM12B
	initUART(MYUBRR);

				  // domy�lnie ka�dy uk�ad powinien miec 0x00 w EEPROM
	uint8_t address=getHexFromEeprom();

	Rfm_xmit(SYNC_PATTERN | address);
				  //ustawiamy programowalny bajt synchronizacji

	TCCR1B |= (1 << WGM12) | (1 << CS10) | (1 << CS12);//timer 1 - tryb ctc - /1024
	OCR1A = 11800;//warto�� dla przerwania co oko�o 755ms
	TIMSK1 |= (1 << OCIE1A);//zezwolenie na przerwanie od ocr1A

	TCCR0A |= (1 << WGM01);
	TCCR0B |= (1 << CS02) | (1 << CS00);
	OCR0A = 16;//warto�c dla przerwania 1ms
	TIMSK0 |= _BV(OCIE0A);

			   //wywo�ujemy funkcj� pomiaru temperatury
	dallas_reset();//reset magistrali 1-wire
	dallas_write_byte(SKIP_ROM_COMMAND);//pomini�cie weryfikacji numeru
	dallas_write_byte(CONVERT_T_COMMAND);//zlecamy konwersj� temperatury

	sei();
	//w��czamy gobalny system przerwa�.

	Rfm_rx_prepare();
	uartSendString("\r\n\r\nRFM12B - SLAVE\r\n");//wy�wietlamy powitanie
	while (1) {
		uint8_t tmp;
		//odbieramy dane w znany nam spos�b

		if (!(PIND & (1 << CLEAR))) {
			writeStringToEeprom("00");
			address=0x00;
			timer0 = 0;
			while (timer0 < 200);
			uartSendString("reset\r\n");
		}

		switch (Rfm_state_check()) {
			case RFM_RXC:
			Rfm_rx_get(rx_buf, &tmp);
			//tu sprawdzamy poprawno�� crc odebranej ramki
			ok = Rfm_rx_frame_good(rx_buf, &tmp, address);
			if (ok) {
				//i je�li prawid�owa to czy zawiera zapytanie o temperatur� (znak T)
				if (rx_buf[0] == 'T') {
					uartSendString(bufor);
					timer0 = 0;
					while (timer0 < 5); //czekamy nieco - master musi przestawi� si� na odbi�r
					//i nadajemy ostatnio przygotowan� ramk�
					Rfm_tx_set((uint8_t*) bufor, strlen(bufor), MASTER_ADDR);
					Rfm_tx_start();
					while (Rfm_state_check() == RFM_TX);
				}
				if (rx_buf[0] == 'N') {
					timer0 = 0;
					while (timer0 < 5);					//czekamy nieco - master musi przestawi� si� na odbi�r
					char newAddress[3];
					newAddress[0] = rx_buf[1];
					newAddress[1] = rx_buf[2];
					newAddress[2] = '\0';
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

		//przerwania wy��czamy na czas komunikacji 1-wire
		cli();
		if (tick_flag) {				//wykonamy te instrukcje co 750ms
			tick_flag = 0;
			ok = dallas_reset();//reset magistrali 1-wire
			if (!ok) {				//je�li nie odpowiedzia� �aden termometr to wy�wietlamy informacj�
				//w ka�dej sytuacji przygotowujemy stosown� informacj�
				sprintf(bufor, "T.PRESENT");
				//nast�pnie przygotowujemy ramk� wraz z sum� CRC
				Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor),
							MASTER_ADDR);

				dallas_reset();
				dallas_write_byte(SKIP_ROM_COMMAND);//pomini�cie weryfikacji numeru
				dallas_write_byte(CONVERT_T_COMMAND);//zlecamy konwersj� temperatury
				continue;
			}

			dallas_reset();				//reset magistrali 1-wire
			dallas_write_byte(SKIP_ROM_COMMAND);//pomini�cie weryfikacji numeru
			dallas_write_byte(READ_SCRATCHPAD_COMMAND);//zlecamy odczyt danych
			dallas_read_buffer(buffer, 9);//odczytujemy dane z termometru
			if (buffer[8] != crc8(buffer, 8)) {	//sprawdzamy sum� kontroln� odczytu
				sprintf(bufor, "T.CRC8.ERR");

				Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor),
							MASTER_ADDR);

				dallas_reset();
				dallas_write_byte(SKIP_ROM_COMMAND);//pomini�cie weryfikacji numeru
				dallas_write_byte(CONVERT_T_COMMAND);//zlecamy konwersj� temperatury
				continue;
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
			sprintf(bufor, "%c%03d.%04d", subzero ? '-' : ' ', temp_int,
						temp_fract);

			Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor), MASTER_ADDR);

			dallas_reset();
			dallas_write_byte(SKIP_ROM_COMMAND);//pomini�cie weryfikacji numeru
			dallas_write_byte(CONVERT_T_COMMAND);//zlecamy konwersj� temperatury
		}
		sei();
	}

	return 0;
}

ISR(TIMER1_COMPA_vect) {				//przerwanie co ~750ms
	tick_flag = 1;//ustawia flag� odmierzenia tego czasu
}
#else
int main(void) {
	writeStringToEeprom("00");
	return 0;
}

#endif
