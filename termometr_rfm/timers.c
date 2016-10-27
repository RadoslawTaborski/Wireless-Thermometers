/*
 * timers.c
 *
 *  Created on: 27 paü 2016
 *      Author: rados
 */

#include "timers.h"

void initCtcTimer0(uint8_t val)
{
	TCCR0A |= (1 << WGM01);
	TCCR0B |= (1 << CS02) | (1 << CS00);
	OCR0A = val;
	TIMSK0 |= _BV(OCIE0A);
}
