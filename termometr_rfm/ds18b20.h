/*
   Plik ds18b20.h

   (xyz.isgreat.org)
*/

#ifndef DS18B20_H
#define DS18B20_H

/* DS18B20 przy³¹czony do portu  PD7 AVRa  */
#define SET_ONEWIRE_PORT     PORTD  |=  _BV(5)
#define CLR_ONEWIRE_PORT     PORTD  &= ~_BV(5)
#define IS_SET_ONEWIRE_PIN   PIND   &   _BV(5)
#define SET_OUT_ONEWIRE_DDR  DDRD   |=  _BV(5)
#define SET_IN_ONEWIRE_DDR   DDRD   &= ~_BV(5)

unsigned char ds18b20_ConvertT(void);
int ds18b20_Read(unsigned char []);
void OneWireStrong(char);
unsigned char OneWireReset(void);
void OneWireWriteByte(unsigned char);
unsigned char OneWireReadByte(void);

#endif
