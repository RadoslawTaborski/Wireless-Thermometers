/*
 * Eeprom.h
 *
 *  Created on: 27 paü 2016
 *      Author: rados
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include <avr/io.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <string.h>

unsigned char readEeprom(unsigned int uiAddress);
void saveEeprom(unsigned int uiAddress, unsigned char ucData);
void readStringFromEeprom(char * address, int length);
void writeStringToEeprom(char *address);
uint8_t getHexFromEeprom();

#endif /* EEPROM_H_ */
