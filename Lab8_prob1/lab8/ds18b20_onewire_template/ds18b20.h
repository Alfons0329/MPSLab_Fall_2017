#ifndef TM_DS18B20_H
#define TM_DS18B20_H

#include "onewire.h"

typedef enum {
	TM_DS18B20_Resolution_9bits = 9,   /*!< DS18B20 9 bits resolution */
	TM_DS18B20_Resolution_10bits = 10, /*!< DS18B20 10 bits resolution */
	TM_DS18B20_Resolution_11bits = 11, /*!< DS18B20 11 bits resolution */
	TM_DS18B20_Resolution_12bits = 12  /*!< DS18B20 12 bits resolution */
} DS18B20_Resolution_t;


int DS18B20_ConvT(OneWire_t* OneWire, DS18B20_Resolution_t precision);
uint8_t DS18B20_Read(OneWire_t* OneWireStruct, float* destination);
uint8_t DS18B20_SetResolution(OneWire_t* OneWireStruct, DS18B20_Resolution_t resolution);
uint8_t DS18B20_Done(OneWire_t* OneWireStruct);


#endif
