/*
 * timers.h
 *
 *  Created on: 27 paü 2016
 *      Author: rados
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#include <avr/io.h>
#include <avr/interrupt.h>

extern volatile uint16_t timer0;

void initCtcTimer0(uint16_t val);
void initCtcTimer1(uint16_t val);
void pause(uint16_t ms);

#endif /* TIMERS_H_ */
