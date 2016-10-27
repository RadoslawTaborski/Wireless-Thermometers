#include"uart.h"

void initUART(unsigned int baud_rate)
{
	UBRR0H = (unsigned char)(baud_rate >> 8);
	UBRR0L = (unsigned char) baud_rate; // ustawienie Baud Rate
	UCSR0B = 1<<TXEN0 | 1 << RXEN0 | 1<<RXCIE0; // umozliwienie odbierania i wysylania oraz wlaczenie przerwania RXC

	UCSR0C = 1<<UCSZ00 | 1 <<UCSZ01; // dlugosc slowa 8 bitow, domyslnie 1 bit stop
}

void uartSend(unsigned char data)
{
	while (!(UCSR0A & (1 << UDRE0))); // czeka az zwolni sie bufor nadajnika

	UDR0 = data; // umieszcza dana w buforze i ja wysyla
}

void uartSendString(char *s)	// wysyla lancuch z pamiêci RAM na UART
{
  register char c;
  while ((c = *s++)) uartSend(c);		// dopoki nie napotka 0 wysyla znak
}

unsigned char uartGet()
{
	while (!(UCSR0A & (1 << RXC0))); //czeka az pojawi sie dana do odbioru
	return UDR0; // odbiera dana
}

