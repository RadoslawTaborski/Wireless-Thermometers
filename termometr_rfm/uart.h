#ifndef uart_h
#define uart_h

#include <avr/io.h>
#include <util/delay.h>

#define FOSC 16000000 //czestotliwosc zegara
#define BAUDD 19200 // predkosc Baud Rate
#define MYUBRR FOSC/16/BAUDD-1 // rejestr Baud Rate

void init_uart(unsigned int baud_rate); //inicjalizuje uart
void uart_send(unsigned char data); // wysyla znak
void uart_send_s(char *s); // wysyla tablice znakow
unsigned char uart_get(); // odbiera znak

#endif
