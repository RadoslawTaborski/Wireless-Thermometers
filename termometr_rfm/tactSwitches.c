/*
 * tactSwitch.c
 *
 *  Created on: 27 paü 2016
 *      Author: rados
 */
#include "tactSwitches.h"

void initTactSwitchResetEeprom()
{
	DDRD &= ~(1<<CLEAR);
	PORTD |= (1<<CLEAR);
}

void initTactSwitchAddSensor()
{
	DDRD &= ~(1<<ADD_SENSOR);
	PORTD |= (1<<ADD_SENSOR);
}

/*uint8_t ClickedSwitch(uint8_t button){
	if(! (PIND & button))
	{
		Pause(80);
		if(! (PIND & button)) return 1;
	}
	return 0;
}*/
