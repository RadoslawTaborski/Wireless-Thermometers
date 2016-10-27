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
#include <stdio.h>
#include "uart.h"
#include "dallas_one_wire.h"
#include "crc8.h"

uint8_t * createAddressArray(uint8_t size);
char* addSensor(uint8_t *array, uint8_t size);
void setAllPins();
char* uintToString(uint8_t address);
void resetDS18B20();
uint8_t temperatureMeasurment(char * bufor);

#endif /* OTHER_H_ */
