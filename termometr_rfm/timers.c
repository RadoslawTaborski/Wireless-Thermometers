/*
 * timers.c
 *
 *  Created on: 27 paü 2016
 *      Author: rados
 */

#include "timers.h"
//TODO: uniezaleønic od taktowania
void initCtcTimer0(uint8_t val) {
	TCCR0A |= (1 << WGM01);
	//TCCR0B |= (1 << CS02) | (1 << CS00);
	TCCR0B |= (1 << CS01) | (1 << CS00);
	OCR0A = val;
	TIMSK0 |= _BV(OCIE0A);
}

void initCtcTimer1(uint16_t val) {
	//TCCR1B |= (1 << WGM12) | (1 << CS10) | (1 << CS12); //timer 1 - tryb ctc - /1024
	TCCR1B |= (1 << WGM11) | (1 << CS10) | (1 << CS11); //timer 1 - tryb ctc - /64
	OCR1A = val; //wartoúÊ dla przerwania co oko≥o 755ms
	TIMSK1 |= (1 << OCIE1A); //zezwolenie na przerwanie od ocr1A
}

void pause(uint16_t ms) {
	timer0 = 0;
	while (timer0 < ms)
		;
}
