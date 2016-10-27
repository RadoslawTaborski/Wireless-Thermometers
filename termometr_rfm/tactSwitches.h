/*
 * tactSwitch.h
 *
 *  Created on: 27 paü 2016
 *      Author: rados
 */

#ifndef TACTSWITCHES_H_
#define TACTSWITCHES_H_

#include <avr/io.h>

#define ADD_SENSOR PD7
#define CLEAR PD6

//uint8_t ClickedSwitch(uint8_t button);
void initTactSwitchResetEeprom();
void initTactSwitchAddSensor();

#endif /* TACTSWITCHES_H_ */
