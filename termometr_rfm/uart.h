#ifndef uart_h
#define uart_h

#include <avr/io.h>
#include <util/delay.h>

#define BAUDD 2400 // predkosc Baud Rate
#define MYUBRR F_CPU/16/BAUDD-1 // rejestr Baud Rate

void initUART(unsigned int baud_rate); //inicjalizuje uart
void uartSend(unsigned char data); // wysyla znak
void uartSendString(char *s); // wysyla tablice znakow
unsigned char uartGet(); // odbiera znak

#endif
