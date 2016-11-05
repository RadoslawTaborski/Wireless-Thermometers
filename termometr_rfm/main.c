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

#define MASTER_ADDR 0xFF //adres układu master

//char bufor_data[32];
char bufor[65];
uint8_t rx_buf[32];
uint8_t ok;
volatile uint16_t timer0;

ISR(TIMER0_COMPA_vect) { //przerwanie co ~1ms
	timer0++;
}

#ifdef MASTER
//--------------------------------------------------------------------------------------------------------
//Kod dla układu MASTER
//--------------------------------------------------------------------------------------------------------
int main(void) {
	//setAllPins();
	initUART(MYUBRR);//inicjalizacja USART
	initTactSwitchAddSensor();
	initTactSwitchResetEeprom();
	initSPI();//inicjalizacja magistrali SPI
	initRFM();//inicjalizacja układ RFM12B
	initCtcTimer0(16);//inicjalizacja timera0 w trybie CTC, czas przerwania ~1ms;

	char text[100];
	uint8_t size = getHexFromEeprom();
	uint8_t *sensorsAddresses = createAddressArray(size);
	//uint8_t cur_meas = 0; //zmienna informująca o aktualnie wybranym układzie
	uint8_t address = 0;
	uint8_t length = 0;

	Rfm_xmit(SYNC_PATTERN | MASTER_ADDR);//ustawiamy programowalny bajt synchronizacji
	Rfm_rx_prepare();

	sei();
	//włączamy przerwania
	uartSendString("\r\n\r\nRFM12B - MASTER\r\n");//wyświetlamy powitanie

	while (1) {
		if (clickedSwitch(CLEAN) && size != 0) {
			writeStringToEeprom("00");
			size = getHexFromEeprom();
			//cur_meas = 0;
			sprintf(text, "\r\nReset\r\nsize: %d\r\n", size);
			uartSendString(text);
			pause(200);
		}

		if (clickedSwitch(ADD_SENSOR)) {
			uartSendString("click");
		}
		//Rfm_rx_prepare();
		receiveRFM12B(MASTER_ADDR, rx_buf, &length);
		if (length != 0) {
			if (rx_buf[0] == '?') {
				char pText[3];
				addSensor(sensorsAddresses, size, pText);
				sprintf(bufor, "N%s", pText);
				//uartSendString(bufor);
				Rfm_stop();
				Rfm_tx_frame_prepare((uint8_t*) bufor, strlen(bufor), 0x00);
				sendRFM12B(0x00, bufor);
				//Rfm_rx_prepare();
				waitForReceive(MASTER_ADDR, rx_buf, &length, 'O', 500);
				if (length != 0) {
					char n_size[3];
					uintToString(++size, n_size);
					writeStringToEeprom(n_size);
					sprintf(bufor, "Dodano nowy czujnik: %d\r\n", size);
					uartSendString(bufor);
					length = 0;
				} else {
					sprintf(bufor, "Adres: %d - BLAD DODANIA\r\n", size + 1);
					uartSendString(bufor);
				}
			} else if (size != 0) {
				char strAddress[3];
				rx_buf[length] = 0;
				strncpy(strAddress, (char*) rx_buf, 2);
				strAddress[2] = 0;
				address = strtol(strAddress, NULL, 16);
				if(address!=0x00)
				{
					sprintf(bufor, "Adres: %d ", address);	//,(char*)(rx_buf+2));
					uartSendString(bufor);
					uartSendString("Temperatura:");
					uartSendString((char*) (rx_buf + 2));
					uartSendString("\r\n");
				}
				length = 0;
			}
		}
	}
	return 0;
}
#elif defined  SLAVE
//--------------------------------------------------------------------------------------------------------
//Kod dla układu SLAVE
//--------------------------------------------------------------------------------------------------------

volatile uint8_t tick_flag = 0;
char newBufor[67];

ISR(TIMER1_COMPA_vect) {				//przerwanie co ~750ms
	tick_flag = 1;				//ustawia flagę odmierzenia tego czasu
}

int main(void) {
	setAllPins();
	initTactSwitchResetEeprom();
	initSPI(); //inicjalizacja magistrali SPI
	initRFM(); //wstępna konfiguracja układu RFM12B
	initUART(MYUBRR);
	initCtcTimer1(11800);
	initCtcTimer0(16);
	resetDS18B20();

	uint8_t address = getHexFromEeprom(); // domyślnie każdy układ powinien miec 0x00 w EEPROM//TODO: popraw eeprom
	char strAddress[3];
	uintToString(address, strAddress);
	uint8_t length = 0;

	Rfm_xmit(SYNC_PATTERN | address); //ustawiamy programowalny bajt synchronizacji
//Rfm_rx_prepare();

	sei();
//włączamy gobalny system przerwań.
	uartSendString("\r\n\r\nRFM12B - SLAVE\r\n"); //wyświetlamy powitanie
	while (1) {
		if (clickedSwitch(CLEAN) && address != 0x00) {
			writeStringToEeprom("00");
			address = 0x00;
			Rfm_xmit(SYNC_PATTERN | address);
			uintToString(address, strAddress);
			uartSendString("reset\r\n");
		}
		cli();
		//przerwania wyłączamy na czas komunikacji 1-wire
		if (tick_flag) { //wykonamy te instrukcje co 750ms
			tick_flag = 0;
			ok = temperatureMeasurment(bufor);
		}
		sei();
		if (ok && address != 0) {
			sprintf(newBufor, "%s%s", strAddress, bufor);
			uartSendString(newBufor);
			Rfm_stop(); //wyłączamy odbiornik
			Rfm_tx_frame_prepare((uint8_t*) newBufor, strlen(newBufor), MASTER_ADDR); //następnie przygotowujemy ramkę wraz z sumą CRC
			sendRFM12B(MASTER_ADDR, newBufor);
			Rfm_stop(); //wyłączamy odbiornik
			pause(1000); //i czekamy przed kolejnym wysłaniem temperatury
		}
		ok = 0;
		bufor[0] = 0;
		newBufor[0] = 0;
		if (address == 0x00) {
			sprintf(newBufor, "?");
			//uartSendString(newBufor);
			Rfm_stop();
			Rfm_tx_frame_prepare((uint8_t*) newBufor, strlen(newBufor), MASTER_ADDR); //następnie przygotowujemy ramkę wraz z sumą CRC
			sendRFM12B(MASTER_ADDR, newBufor);
			ok = waitForReceive(address, rx_buf, &length, 'N', 500);
			if (ok) {
				char newAddress[3];
				sprintf(newAddress, "%s", (char *) (rx_buf + 1));
				writeStringToEeprom(newAddress);
				address = (uint8_t) strtol(newAddress, NULL, 16);
				uintToString(address, strAddress);
				sprintf(newBufor, "O");
				//uartSendString(newBufor);
				Rfm_stop();
				Rfm_tx_frame_prepare((uint8_t*) newBufor, strlen(newBufor), MASTER_ADDR); //następnie przygotowujemy ramkę wraz z sumą CRC
				sendRFM12B(MASTER_ADDR, newBufor);
				Rfm_xmit(SYNC_PATTERN | address);
				rx_buf[0] = 0;
				length = 0;
				ok = 0;
			}
		}
	}
	return 0;
}
#else
int main(void) {
	writeStringToEeprom("00");
	return 0;
}

#endif
