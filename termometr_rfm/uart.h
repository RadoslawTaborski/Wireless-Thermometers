#ifndef uart_h
#define uart_h

#include <avr/io.h>
#include <util/delay.h>

#define FOSC 16000000 //czestotliwosc zegara
#define BAUDD 19200 // predkosc Baud Rate
#define MYUBRR FOSC/16/BAUDD-1 // rejestr Baud Rate

void initUART(unsigned int baud_rate); //inicjalizuje uart
void uartSend(unsigned char data); // wysyla znak
void uartSendString(char *s); // wysyla tablice znakow
unsigned char uartGet(); // odbiera znak

#endif
