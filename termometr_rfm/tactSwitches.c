/*
 * tactSwitch.c
 *
 *  Created on: 27 paü 2016
 *      Author: rados
 */
#include "tactSwitches.h"

void initTactSwitchResetEeprom()
{
	DDRD &= ~CLEAN;
	PORTD |= CLEAN;
}

void initTactSwitchAddSensor()
{
	DDRD &= ~ADD_SENSOR;
	PORTD |= ADD_SENSOR;
}

uint8_t clickedSwitch(uint8_t button){
	if(! (PIND & button))
	{
		pause(80);
		if(! (PIND & button)) return 1;
	}
	return 0;
}
