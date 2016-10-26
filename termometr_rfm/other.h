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

#define ADD_SENSOR PD7
#define CLEAR PD6


unsigned char EEPROM_odczyt(unsigned int uiAddress);
void EEPROM_zapisz(unsigned int uiAddress, unsigned char ucData);
void initializeAddressArray(uint8_t * array, uint8_t size);
char * addSensor(uint8_t *array, uint8_t size);
void readStringFromEeprom(char * address, int length);
void writeStringToEeprom(char *address);
void setAllPins();
char *uint8_tToCharArray(uint8_t address);
//uint8_t ClickedSwitch(uint8_t button);
//void Pause(uint8_t ms);

#endif /* OTHER_H_ */
