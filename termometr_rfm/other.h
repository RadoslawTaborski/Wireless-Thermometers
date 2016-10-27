/*
 * other.h
 *
 *  Created on: 24 lip 2016
 *      Author: rados
 */

#ifndef OTHER_H_
#define OTHER_H_

#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <string.h>
#include "uart.h"


uint8_t * createAddressArray(uint8_t size);
char * addSensor(uint8_t *array, uint8_t size);

void setAllPins();
char *uintToString(uint8_t address);

#endif /* OTHER_H_ */
