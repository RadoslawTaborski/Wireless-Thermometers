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

#define MASTER_ADDR 0xFF //adres uk³adu master

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

	//char text[100];
	//uint8_t size = getHexFromEeprom();
	//uint8_t *sensorsAddresses = createAddressArray(size);
	//uint8_t cur_meas = 0;//zmienna informuj¹ca o aktualnie wybranym uk³adzie
	//uint8_t option = 0;
	//uint8_t address = 0;
	uint8_t length = 0;
	Rfm_xmit(SYNC_PATTERN | MASTER_ADDR);//ustawiamy programowalny bajt synchronizacji
	Rfm_rx_prepare();

	sei();
	//w³¹czamy przerwania
	uartSendString("\r\n\r\nRFM12B - MASTER\r\n");//wyœwietlamy powitanie

	while (1) {
		switch (Rfm_state_check()) {
			case RFM_RXC:
			Rfm_rx_get(rx_buf, &length); //tu sprawdzamy poprawnoœæ crc odebranej ramki
			ok = Rfm_rx_frame_good(rx_buf, &length, MASTER_ADDR);
			if (ok) {
				char strAddress[3]= {0};
				rx_buf[length] = 0;
				strncpy(strAddress,(char*)rx_buf,2);
				strAddress[2]=0;
				sprintf(bufor, "Adres: %s Temperatura:%s \r\n", strAddress, (char*)(rx_buf+2));
				uartSendString(bufor);
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
	return 0;
}
#elif defined  SLAVE
//--------------------------------------------------------------------------------------------------------
//Kod dla uk³adu SLAVE
//--------------------------------------------------------------------------------------------------------

volatile uint8_t tick_flag = 0;
char newBufor[67];

ISR(TIMER1_COMPA_vect) {				//przerwanie co ~750ms
	tick_flag = 1;				//ustawia flagê odmierzenia tego czasu
}

int main(void) {
	setAllPins();
	initTactSwitchResetEeprom();
	initSPI(); //inicjalizacja magistrali SPI
	initRFM(); //wstêpna konfiguracja uk³adu RFM12B
	initUART(MYUBRR);
	initCtcTimer1(11800);
	initCtcTimer0(16);
	resetDS18B20();

	uint8_t address = getHexFromEeprom(); // domyœlnie ka¿dy uk³ad powinien miec 0x00 w EEPROM
	char *strAddress=uintToString(address);

	Rfm_xmit(SYNC_PATTERN | address); //ustawiamy programowalny bajt synchronizacji
	//Rfm_rx_prepare();

	sei();
	//w³¹czamy gobalny system przerwañ.
	uartSendString("\r\n\r\nRFM12B - SLAVE\r\n"); //wyœwietlamy powitanie
	while (1) {
		cli();
		//przerwania wy³¹czamy na czas komunikacji 1-wire
		if (tick_flag) { //wykonamy te instrukcje co 750ms
			tick_flag = 0;
			ok = temperatureMeasurment(bufor);
		}
		sei();
		if (ok) {
			sprintf(newBufor, "%s%s", strAddress, bufor);
			uartSendString(newBufor);
			Rfm_tx_frame_prepare((uint8_t*) newBufor, strlen(newBufor), MASTER_ADDR); //nastêpnie przygotowujemy ramkê wraz z sum¹ CRC
			sendRFM12B(MASTER_ADDR, newBufor);
			Rfm_stop(); //wy³¹czamy odbiornik
			pause(1000); //i czekamy przed kolejnym wys³aniem temperatury
		}
		ok = 0;
		bufor[0] = 0;
		newBufor[0] = 0;
	}

	return 0;
}
#else
int main(void) {
	writeStringToEeprom("00");
	return 0;
}

#endif
