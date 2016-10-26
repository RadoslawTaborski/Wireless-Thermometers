/////////////////////////////////////////////////////WERSJA3B///////////////////////////////////////////////////////////////////////////

//#include <util/delay.h>
#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "crc8.h"
#include "dallas_one_wire.h"

#include "RFM12B.h"
#include "uart.h"
#include "other.h"

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

//jeœli poni¿sza linia bêdzie zakomentowana to skompilujemy kod dla uk³adu SLAVE,
//natomiast jeœli bêdzie odkomentowana to skompilujemy kod dla uk³adu SLAVE
//#define MASTER
#define SLAVE

//adres uk³adu master
#define MASTER_ADDR 0xFF
//adresy 2 uk³adów slave
//#define SLAVE1_ADDR 0x00
//#define SLAVE2_ADDR 0xFF
//tu definiujemy z jakim adresem zostanie skompilowany kod dla uk³adu slave
#define CUR_SLAVE_ADDR 0x00

#ifdef MASTER
//--------------------------------------------------------------------------------------------------------
//Kod dla uk³adu MASTER
//--------------------------------------------------------------------------------------------------------
int main(void) {
	//setAllPins();
	//DDRD &= ~(1<<ADD_SENSOR);
	PORTD |= (1<<ADD_SENSOR);
	//DDRD &= ~(1<<CLEAR);
	PORTD |= (1<<CLEAR);
	Rfm_spi_init(); //inicjalizacja magistrali SPI

	init_uart(MYUBRR); //inicjalizacja USART

	Rfm_init(); //inicjalizujemy uk³ad RFM12B

	Rfm_xmit(SYNC_PATTERN | MASTER_ADDR);
	//ustawiamy programowalny bajt synchronizacji

	TCCR0A |= (1 << WGM01);
	TCCR0B |= (1 << CS02) | (1 << CS00);
	OCR0A = 16; //wartoœc dla przerwania 1ms
	TIMSK0 |= _BV(OCIE0A);

	char p_size[3];
	readStringFromEeprom(p_size, 3);
	uint8_t size = (uint8_t) strtol(p_size, NULL, 16);
	//uint8_t size = atoi(p_size);
	uint8_t *slave_addr = (uint8_t*) malloc(255 * sizeof(uint8_t));
	initializeAddressArray(slave_addr, (uint8_t) size);

	for (int i = 0; i < size; ++i) {
		char b[5];
		itoa(slave_addr[i], b, 16);
		//uart_send_s(b);
		//uart_send_s("tu\r\n");
	}

	//uint8_t add=0, clear=0;

	uint8_t cur_meas = 0; //i zmienn¹ informuj¹c¹ o aktualnie wybranym uk³adzie
	sei();	//w³¹czamy przerwania do obs³ugi uart

	uart_send_s("\r\n\r\nRFM12B - MASTER\r\n"); //wyœwietlamy powitanie
	while (1) {
		//za ka¿dym razem zmieniamy aktywny uk³ad.
		if (!(PIND & (1 << ADD_SENSOR))) {
			//uart_send_s("tu1\r\n");
			char *result;
			result = addSensor(slave_addr, size);
			timer0 = 0;
			while (timer0 < 500);
			//_delay_ms(500);
			//char bbb[3];
			//itoa(size,bbb,16);
			//uart_send_s(bbb);
			//uart_send_s("\r\n");

			//uart_send_s("\r\n");
			//uart_send_s(result);
			bufor[0] = 'N';
			bufor[1] = result[0];
			bufor[2] = result[1];
			bufor[3] = 0;
			uart_send_s("\r\nDODAWANIE CZUJNIKA: ");
			uart_send_s(bufor);
			uart_send_s("\r\n");

			Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor), 0x00);
			Rfm_tx_set((uint8_t*) bufor, strlen(bufor), 0x00);
			Rfm_tx_start();
			while (Rfm_state_check() == RFM_TX);
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
						sprintf(bufor, "%d: ", cur_meas + 1);
						uart_send_s(bufor);
						uart_send_s("DODANO CZUJNIK");
						//i informacjê o zgodnoœci sumy kontrolnej (normalnie danych ze z³ym crc wogóle byœmy nie obs³ugiwali)
					}
					uart_send_s("\r\n");
					//ustawiamy timeout na wartoœæ 1...
					timeout = 1;
				}
				//...dziêki czemu tu zostanie on zmieniony na wartoœæ 0, której nie osi¹gn¹³by gdyby pêtla zakoñczy³a siê
				//bez wykonania tamtej instrukcji if
				timeout--;
				//czekamy nieco miêdzy kolejnym sprawdzeniem, czy dane nie nadesz³y
				timer0 = 0;
				while (timer0 < 1);
				//_delay_us(250);

			}
			//tu sprawdzamy, czy dane zosta³y odebrane, czy te¿ nie by³o odpowiedzi i wyœwietlamy wtedy stosowny komunikat
			if (timeout == 1) {
				sprintf(bufor, "%d: BLAD DODANIA\r\n", cur_meas + 1);
				uart_send_s(bufor);
			}
			//wy³¹czamy odbiornik
			Rfm_stop();
			//i czekamy przed kolejnym zapytaniem o temperaturê
			timer0 = 0;
			while (timer0 < 5000);
			//_delay_ms(500);
			/*uart_send_s("\r\n");
			 readStringFromEeprom(p_size,3);
			 uart_send_s("size: ");
			 uart_send_s(p_size);
			 uart_send_s("\r\n");*/
		}

		if (!(PIND & (1 << CLEAR))) {
			writeStringToEeprom("00");
			readStringFromEeprom(p_size, 3);
			size = atoi(p_size);
			cur_meas = 0;
			uart_send_s("\r\nCLEAR\r\n");
			uart_send_s("size: ");
			uart_send_s(p_size);
			uart_send_s("\r\n");
			timer0 = 0;
			while (timer0 < 200);
			//_delay_ms(200);
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
			Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor),
					slave_addr[cur_meas]);
			//uart_send_s(" 2\r\n ");
			//wysy³anie ju¿ znamy
			Rfm_tx_set((uint8_t*) bufor, strlen(bufor), slave_addr[cur_meas]);
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
						uart_send_s(bufor);
						rx_buf[tmp] = 0;
						uart_send_s((char*) rx_buf);
						//i informacjê o zgodnoœci sumy kontrolnej (normalnie danych ze z³ym crc wogóle byœmy nie obs³ugiwali)
					}
					uart_send_s("\r\n");
					//ustawiamy timeout na wartoœæ 1...
					timeout = 1;
				}
				//...dziêki czemu tu zostanie on zmieniony na wartoœæ 0, której nie osi¹gn¹³by gdyby pêtla zakoñczy³a siê
				//bez wykonania tamtej instrukcji if
				timeout--;
				//czekamy nieco miêdzy kolejnym sprawdzeniem, czy dane nie nadesz³y
				timer0 = 0;
				while (timer0 < 1);
				//_delay_us(250);

			}
			//tu sprawdzamy, czy dane zosta³y odebrane, czy te¿ nie by³o odpowiedzi i wyœwietlamy wtedy stosowny komunikat
			if (timeout == 1) {
				sprintf(bufor, "%d: BRAK ODPOWIEDZI\r\n", cur_meas + 1); // TODO: chyba niepotrzebne
				uart_send_s(bufor);
			}
			//wy³¹czamy odbiornik
			Rfm_stop();
			//i czekamy przed kolejnym zapytaniem o temperaturê
			timer0 = 0;
			while (timer0 < 5000);
			//_delay_ms(500);
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
	DDRD &= ~(1<<CLEAR);
	PORTD |= (1<<CLEAR);

	Rfm_spi_init();			//inicjalizacja magistrali SPI

	Rfm_init();//wstêpna konfiguracja uk³adu RFM12B
	init_uart(MYUBRR);

	// domyœlnie ka¿dy uk³ad powinien miec 0x00 w EEPROM
	char t_address[3];
	readStringFromEeprom(t_address, 3);
	uint8_t address = (uint8_t) strtol(t_address, NULL, 16);

	Rfm_xmit(SYNC_PATTERN | address);
	//ustawiamy programowalny bajt synchronizacji

	TCCR1B |= (1 << WGM12) | (1 << CS10) | (1 << CS12);//timer 1 - tryb ctc - /1024
	OCR1A = 11800;//wartoœæ dla przerwania co oko³o 755ms
	TIMSK1 |= (1 << OCIE1A);//zezwolenie na przerwanie od ocr1A

	TCCR0A |= (1 << WGM01);
	TCCR0B |= (1 << CS02) | (1 << CS00);
	OCR0A = 16; //wartoœc dla przerwania 1ms
	TIMSK0 |= _BV(OCIE0A);

	//wywo³ujemy funkcjê pomiaru temperatury
	dallas_reset();//reset magistrali 1-wire
	dallas_write_byte(SKIP_ROM_COMMAND);//pominiêcie weryfikacji numeru
	dallas_write_byte(CONVERT_T_COMMAND);//zlecamy konwersjê temperatury

	sei();
	//w³¹czamy gobalny system przerwañ.

	Rfm_rx_prepare();
	uart_send_s("\r\n\r\nRFM12B - SLAVE\r\n"); //wyœwietlamy powitanie
	while (1) {
		uint8_t tmp;
		//odbieramy dane w znany nam sposób

		if (!(PIND & (1 << CLEAR))) {
			writeStringToEeprom("00");
			address=0x00;
			timer0 = 0;
			while (timer0 < 200);
			uart_send_s("reset\r\n");
		}

		switch (Rfm_state_check()) {
			case RFM_RXC:
			Rfm_rx_get(rx_buf, &tmp);
			//tu sprawdzamy poprawnoœæ crc odebranej ramki
			ok = Rfm_rx_frame_good(rx_buf, &tmp, address);
			if (ok) {
				//i jeœli prawid³owa to czy zawiera zapytanie o temperaturê (znak T)
				if (rx_buf[0] == 'T') {
					uart_send_s(bufor);
					timer0 = 0;
					while (timer0 < 5);//czekamy nieco - master musi przestawiæ siê na odbiór
					//i nadajemy ostatnio przygotowan¹ ramkê
					Rfm_tx_set((uint8_t*) bufor, strlen(bufor), MASTER_ADDR);
					Rfm_tx_start();
					while (Rfm_state_check() == RFM_TX);
				}
				if (rx_buf[0] == 'N') {
					timer0 = 0;
					while (timer0 < 5);//czekamy nieco - master musi przestawiæ siê na odbiór
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

		//przerwania wy³¹czamy na czas komunikacji 1-wire
		cli();
		if (tick_flag) {				//wykonamy te instrukcje co 750ms
			tick_flag = 0;
			ok = dallas_reset();//reset magistrali 1-wire
			if (!ok) {//jeœli nie odpowiedzia³ ¿aden termometr to wyœwietlamy informacjê
				//w ka¿dej sytuacji przygotowujemy stosown¹ informacjê
				sprintf(bufor, "T.PRESENT");
				//nastêpnie przygotowujemy ramkê wraz z sum¹ CRC
				Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor),
						MASTER_ADDR);

				dallas_reset();
				dallas_write_byte(SKIP_ROM_COMMAND);//pominiêcie weryfikacji numeru
				dallas_write_byte(CONVERT_T_COMMAND);//zlecamy konwersjê temperatury
				continue;
			}

			dallas_reset();				//reset magistrali 1-wire
			dallas_write_byte(SKIP_ROM_COMMAND);//pominiêcie weryfikacji numeru
			dallas_write_byte(READ_SCRATCHPAD_COMMAND);//zlecamy odczyt danych
			dallas_read_buffer(buffer, 9);//odczytujemy dane z termometru
			if (buffer[8] != crc8(buffer, 8)) {	//sprawdzamy sumê kontroln¹ odczytu
				sprintf(bufor, "T.CRC8.ERR");

				Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor),
						MASTER_ADDR);

				dallas_reset();
				dallas_write_byte(SKIP_ROM_COMMAND);//pominiêcie weryfikacji numeru
				dallas_write_byte(CONVERT_T_COMMAND);//zlecamy konwersjê temperatury
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
			sprintf(bufor, "%c%03d.%04d", subzero ? '-' : ' ', temp_int,
					temp_fract);

			Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor), MASTER_ADDR);

			dallas_reset();
			dallas_write_byte(SKIP_ROM_COMMAND);//pominiêcie weryfikacji numeru
			dallas_write_byte(CONVERT_T_COMMAND);//zlecamy konwersjê temperatury
		}
		sei();
	}

	return 0;
}

ISR(TIMER1_COMPA_vect) {				//przerwanie co ~750ms
	tick_flag = 1;//ustawia flagê odmierzenia tego czasu
}
#else
int main(void) {
	writeStringToEeprom("00");
	return 0;
}

#endif
