/*
 * tactSwitch.h
 *
 *  Created on: 27 paü 2016
 *      Author: rados
 */

#ifndef TACTSWITCHES_H_
#define TACTSWITCHES_H_

#include <avr/io.h>
#include <util/delay.h>
#include "timers.h"

#define ADD_SENSOR (1<<PD7)
#define CLEAN (1<<PD6)

uint8_t clickedSwitch(uint8_t button);
void initTactSwitchResetEeprom();
void initTactSwitchAddSensor();

#endif /* TACTSWITCHES_H_ */
