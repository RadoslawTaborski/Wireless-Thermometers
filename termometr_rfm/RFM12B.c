/*
 * RFM12B.c
 *
 *  Created on: 19-02-2014
 *      Author: Piotr Rzeszut
 *
 * Description: Funkcje do obs�ugi uk�adu RFM12B z poziomu procesora AVR
 */
#include "RFM12B.h"
#include "uart.h"

//definiujemy zmienne kontroluj�ce nadawanie, odbi�r oraz bufory nadawczy i odbiorczy
volatile rfm_state_t rfm_state=RFM_IDLE;

volatile char RFM_RX_BUF[RFM_BUFFER_SIZE];
volatile uint8_t rfm_rx_h=0;

//5 dodatkowych miejsc na bajty synchronizacji oraz ko�cz�ce trnasmisj�
volatile char RFM_TX_BUF[RFM_BUFFER_SIZE+5];
volatile uint8_t rfm_tx_h=0;
volatile uint8_t rfm_tx_t=0;

void initSPI(void){
	SPI_DDR &= ~(1<<SPI_MISO);
	SPI_DDR |= (1<<SPI_SCK)|(1<<SPI_MOSI)|(1<<SPI_SS);//konfiguracja kierunku i podcigania linii SPI
	SPI_PORT |= (1<<SPI_MISO);
	//do poprawnej pracy modu�u SPI w procesorze linia SS musi by� ustawiona jako wyj�cie

	SPCR |= (1<<SPE)|(1<<MSTR)|(1<<SPR0);//SPI w trybie master z podzia�em zegara przez 64


	CS_PORT |= (1<<CS_RFM);//dezaktywacja uk�adu RFM12B
	CS_DDR |= (1<<CS_RFM);//konfiguracja kierunku linii CS

	RFM_INT_DDR &= ~(1<<RFM_INT_PORT_NUM);
	RFM_INT_PORT |= (1<<RFM_INT_PORT_NUM);//w��czenie podci�gania na linii przerwania
}

uint16_t Rfm_xmit(uint16_t data){
	//na pocz�tek rodzielamy dane na 2 bajty
	uint8_t msb, lsb;
	lsb = data;
	msb = data>>8;
	CS_PORT &= ~(1<<CS_RFM);//aktywujemy lini� CS uk�adu
	//teraz wysy�amy 2 bajty jednocze�nie odbieraj�c 2 bajty z uk�adu
	SPDR = msb;
	while(!(SPSR&(1<<7)));
	msb = SPDR;
	SPDR = lsb;
	while(!(SPSR&(1<<7)));
	lsb = SPDR;
	//_delay_ms(1);
	CS_PORT |= (1<<CS_RFM);//dezaktywujemy lini� CS uk�adu
	//i zwracamy odebrane dane
	return( (((uint16_t)msb)<<8)+((uint16_t)lsb) );
}

void initRFM(void){
	Rfm_xmit(SW_RESET);//resetuj� programowo uk�ad RFM12B
	_delay_ms(250);

	//inicjalizacja RFM12B
	//ustawienie pasma 868MHz, konfiguracja FIFO
	Rfm_xmit(CONFIGURATION|EN_DATA_REG|EN_FIFO|BAND_868|CAP_12_0);
	//w��czenie oscylatora
	Rfm_xmit(POWER|EN_OSC|DIS_CLKO);
	//ustawienie pasma (musi by� takie samo w nadajniku i odbiorniku)
	//Dla naszego uk�adu cz�stotliwo�� musi zawiera� si� w przedziale 860480000-879515000Hz i musi by� podawana ze skokami 5000Hz
	Rfm_xmit(FREQUENCY|RF12_FREQUENCY_CALC_868(868000000UL));
	//ustawienie pr�dko�ci transmisji danych (musi by� takia sama w nadajniku i odbiorniku)
	Rfm_xmit(BAUD|BAUD_4800);
	//ustawienie pasma 134kHz i parametr�w odbiornika
	Rfm_xmit(RECEIVER|P20_VDI|BW134|LNA_0|RSSI_103);
	//ustawienie cyfroiwego filtra danych i opcji odzyskiwania zegara
	Rfm_xmit(DATA_FILTER|AUTO_CR|DIGITAL_F|DQD_4);
	//reset bufora FIFO, konfiguracja synchronizacji za pomoc� 2 bajt�w, ustawienie generowania przerwania FFIT po odebraniu 8 bit�w
	Rfm_xmit(FIFO_RST|FIFO_IT_8|FIFO_SYNC|HS_RST_DIS);


	//konfiguracja kontrolera cz�stotliwo�ci
	Rfm_xmit(AFC|KEEP_RX|REST_OFF|EN_FOFFSET|EN_AFC);
	//konfiguracja nadajnika i jego mocy (na ustawienie maksymalne)
	Rfm_xmit(TRANSMITER|FSK_PHASE_0|FSK_DF_90K|OUT_P_0);
	//konfiguracja p�tli PLL
	Rfm_xmit(PLL|PLL_DH_DIS|SEL_CLK_2_5|MAX_BAUD_256);

	//wy��czenie timera wakeup
	Rfm_xmit(WAKEUP_TIM|WUT_X_2_0|0);
	//wy��czenie oszcz�dzania energii
	Rfm_xmit(LOW_DC|LOW_DC_DIS);
	//ustawienie monitora napi�cia na 2,2V
	Rfm_xmit(BOD_CLK|CLK_5_00|BOD_2_2);
}

uint8_t Rfm_ready_wait(void){
	uint8_t i=0;

	CS_PORT &= ~(1<<CS_RFM);//za��czam pin CS uk�adu
	_delay_us(1);//czekam nieco aby uk�ad m�g� zareagowa�

	while(!((1<<SPI_MISO)&SPI_PIN)){//nast�pnie co 1ms sprawdzam, czy uk�ad jest wolny, tzn, czy wystawi� na linii MISO 1.
		_delay_ms(1);
		if((++i)==200){
			return 1;//po ok. 200ms przerywam oczekiwanie i zwracam 1 - czyli sygnalizuj� b��d
		}
	}
	return 0;//je�li p�tal zostanie przerwana z powodu wyst�pienia stanu wysokiego na linii MISO to zwracam 0 - nie ma b��du
}

void Rfm_stop(void){
	//wy��czamy przerwanie
	RFM_INT_MASK &= ~(1<<RFM_INT_NUM);
	//wy��czamy nadajnik i odbiornik
	Rfm_xmit(POWER|DIS_CLKO);
	//kasujemy ewentualne przerwania
	Rfm_xmit(STATUS_READ);
}

void Rfm_rx_prepare(void){
	//informuj� program w jakim stanie jest uk�ad RFM
	rfm_state=RFM_RX;
	//w��czamy odbiornik
	Rfm_xmit(POWER|EN_RX|EN_BASEBAND|EN_OSC|DIS_CLKO);
	_delay_ms(5);

	//para komend powoduj�ca w efekcie dzia�ania reset synchronizacji odbiornika
	Rfm_xmit(FIFO_RST|FIFO_IT_8|HS_RST_DIS);
	Rfm_xmit(FIFO_RST|FIFO_IT_8|EN_AFT_SYNC|HS_RST_DIS);

	//odczytuj� status, aby na pewno zosta�a zwolniona linia przerwania
	Rfm_xmit(STATUS_READ);

	//ustawiam wska�nik odczytu na pocz�tek
	rfm_rx_h=0;

	//aktywuj� przerwanie
	RFM_INT_MASK |= (1<<RFM_INT_NUM);
}

void Rfm_tx_start(void){
	//informuj� program w jakim stanie jest uk�ad RFM
	rfm_state=RFM_TX;
	//w��czamy nadajnik
	Rfm_xmit(STATUS_READ);
	Rfm_xmit(POWER|EN_TRANSMISSION|EN_SYNTH|EN_OSC|DIS_CLKO);

	//ustawiam wska�nik nadawania na pocz�tek
	rfm_tx_h=0;

	//aktywuj� przerwanie
	RFM_INT_MASK |= (1<<RFM_INT_NUM);
}

ISR(INT0_vect){
	//uartSendString("przerwanko\r\n");
	//odczytuj� status, aby na pewno zosta�a zwolniona linia przerwania
	uint16_t status=Rfm_xmit(STATUS_READ);
	//sprawdzi� czy to to przerwanie
	if(status&(M_FIFO_IT|M_TX_READY)){//je�li to przerwanie zosta�o wygenerowane

		if(rfm_state==RFM_RX){//je�li mamy odbiera� dane
			uint8_t data=Rfm_xmit(FIFO_READ);//to odczytujemy bufor fifo

			if(data==0x03){//je�li to znak zako�czenia transmisji

				rfm_state=RFM_RXC;//to ustawiamy stan zako�czenia odbioru
				//i wy��czamy nadajnik
				Rfm_stop();

			}else if(rfm_rx_h<RFM_BUFFER_SIZE){//je�li w czasie odbioru rozmiar bufora nie zostanie przekroczony

				RFM_RX_BUF[rfm_rx_h++]=data;//to zapisujemy dane

			}else{//w przeciwnym wypadku ustawimy odpowiedni stan i wy��czymy odbiornik

				rfm_state=RFM_RXOVF;

				Rfm_stop();

			}


		}else if(rfm_state==RFM_TX){//je�li mamy wysy�a� dane

			if(rfm_tx_h<=rfm_tx_t){//je�li jeszcze nie wys�ali�my wszystkich bajt�w

				Rfm_xmit(TX_WRITE|RFM_TX_BUF[rfm_tx_h++]);//to wysy�amy dany bajt i inkrementujemy wska�nik

			}else{//w przeciwnym razie wy��czamy nadajnik i sygnalizujemy zako�czenie nadawania

				rfm_state=RFM_TXC;

				Rfm_stop();
			}

		}

	}else{//je�li to inne przerwanie zwi�zane z nadawaniem to podejmujemy stosowne dzia�ania
		if(rfm_state==RFM_TX&&(status&M_TX_OVF)){
			rfm_state = RFM_TXOVF;
			Rfm_stop();
		}else if(rfm_state==RFM_RX&&(status&M_FIFO_OVF)){
			rfm_state = RFM_RXOVF;
			Rfm_stop();
		}
	}//innych przerwa� nie obs�ugujemy
}

rfm_state_t Rfm_state_check(void){
	return rfm_state;//sprawdzamy stan uk�adu
}

void Rfm_rx_get(uint8_t* buffer, uint8_t* num){
	for(uint8_t i=0;i<rfm_rx_h;i++){//kopiujemy zawarto�� bufora do wskazanego bufora w programie
		(*(buffer+i))=RFM_RX_BUF[i];
	}
	*num=rfm_rx_h;
}

void Rfm_tx_set(uint8_t* buffer, uint8_t num, uint8_t sync){
	RFM_TX_BUF[0]=0xAA;//sta�y bajt synchronizacji
	RFM_TX_BUF[1]=0x2D;//sta�y bajt synchronizacji
	RFM_TX_BUF[2]=sync;//definiowany bajt synchronizacji
	for(uint8_t i=0;i<num;i++){
		RFM_TX_BUF[i+3]=(*(buffer+i));
	}
	RFM_TX_BUF[num+3]=0x03;//bajt ko�cz�cy ramk�
	RFM_TX_BUF[num+4]=0xAA;//bajt ko�cz�cy transmisj�(dummy byte)
	rfm_tx_t=num+4;
}

//funkcja dodaj�ca kontrol� crc8 przy nadawaniu
void Rfm_tx_frame_prepare(uint8_t* buffer, uint8_t num, uint8_t sync){
	uint8_t crc;
	(*(buffer+num))=sync;//do bufora na koniec dopisujemy adres adresata wiadomo�ci
	crc=crc8(buffer,num+1);//nastepnie liczymy crc8 dla danych z do��czonym adresem
	sprintf((char*)(buffer+num),"%03d",crc);//i dopisujemy crc jako 3 cyfry w zapisie dziesi�tnym na ko�cu paczki danych
}

//funkcja sprawdzaj�ca kontrol� crc podczas odbioru
uint8_t Rfm_rx_frame_good(uint8_t* buffer, uint8_t* num, uint8_t sync){
	uint8_t crc;
	*(buffer+(*num))=0;//dodaj� na koniec znak ko�ca ci�gu
	crc=atoi((char*)(buffer+(*num)-3));//aby potem mie� pewno��, �e funkcja ta przekszta�ci 3 ostatnie znaki na liczb�
	*(buffer+(*num)-3)=sync;//za w�a�ciwymi danymi dopisuj� w�asny adres
	//i potem licz� crc8 - je�li wszystko ok to znaczy �e ani dane ani adres nie zosta�y przek�amane

	if(crc8(buffer,(*num)-2)==crc){//crc zgodne
		(*num)-=3;
		return 1;
	}else{//crc b��dne
		(*num)-=3;
		return 0;
	}
}
