/*
 * RFM12B.h
 *
 *  Created on: 10-06-2013
 *      Author: Piotr Rzeszut
 *  
 * Description: Funkcje do obs³ugi uk³adu RFM12B z poziomu procesora AVR
 */
#ifndef RFM12B_H
#define RFM12B_H

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "RFM12B_reg.h"
#include "crc8.h"

//definicje pinów IO, na których pod³¹czona jest magistrala SPI
#define SPI_DDR DDRB
#define SPI_PORT PORTB
#define SPI_PIN PINB

#define SPI_SS PB2
#define SPI_MOSI PB3
#define SPI_MISO PB4
#define SPI_SCK PB5

//pin CS mo¿emy wybraæ dowolnie
#define CS_DDR DDRB
#define CS_PORT PORTB
#define CS_RFM PB2

//definicja przerwania wyzwalanego stanem niskim
#define RFM_INT_MASK EIMSK
#define RFM_INT_NUM INT0
#define RFM_INT_DDR DDRD
#define RFM_INT_PORT PORTD
#define RFM_INT_PORT_NUM PD2

//rozmiar bufora musi byæ naturaln¹ potêg¹ liczby 2
#define RFM_BUFFER_SIZE 32

typedef enum{
	RFM_IDLE,//bezczynnoœæ
	RFM_RX,//odbiór
	RFM_RXC,//odbiór zakoñczony
	RFM_RXOVF,//przepe³nienie bufora odbiorczego
	RFM_TX,//nadawanie
	RFM_TXC,//nadawanie zakoñczone
	RFM_TXOVF,//nadpisanie bufora nadawczego
} rfm_state_t;

void Rfm_spi_init(void);

uint16_t Rfm_xmit(uint16_t data);

void Rfm_init(void);

uint8_t Rfm_ready_wait(void);

void Rfm_stop(void);

void Rfm_rx_prepare(void);

rfm_state_t Rfm_state_check(void);

void Rfm_rx_get(uint8_t* buffer, uint8_t* num);

void Rfm_tx_start(void);

void Rfm_tx_set(uint8_t* buffer, uint8_t num, uint8_t sync);

void Rfm_tx_frame_prepare(uint8_t* buffer, uint8_t num, uint8_t sync);

uint8_t Rfm_rx_frame_good(uint8_t* buffer, uint8_t* num, uint8_t sync);

#endif //RFM12B_H
