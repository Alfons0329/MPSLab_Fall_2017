#ifndef TM_DS18B20_H
#define TM_DS18B20_H

#include "onewire.h"

typedef enum
{
	TM_DS18B20_Resolution_9bits = 9,   /*!< DS18B20 9 bits resolution */
	TM_DS18B20_Resolution_10bits = 10, /*!< DS18B20 10 bits resolution */
	TM_DS18B20_Resolution_11bits = 11, /*!< DS18B20 11 bits resolution */
	TM_DS18B20_Resolution_12bits = 12  /*!< DS18B20 12 bits resolution */
} DS18B20_Resolution_t;


int DS18B20_ConvT(OneWire_t* OneWire, DS18B20_Resolution_t precision);
uint8_t DS18B20_Read(OneWire_t* OneWireStruct, float* destination);
uint8_t DS18B20_SetResolution(OneWire_t* OneWireStruct, DS18B20_Resolution_t resolution);
uint8_t DS18B20_Done(OneWire_t* OneWireStruct);
/* Send ConvT through OneWire with resolution
 * param:
 *   OneWire: send through this
 *   resolution: temperature resolution
 * retval:
 *    0 -> OK
 *    1 -> Error
 */
int DS18B20_ConvT(OneWire_t* OneWire, DS18B20_Resolution_t resolution)
{
	// TODO
	return 0;
}

/* Read temperature from OneWire
 * param:
 *   OneWire: send through this
 *   destination: output temperature
 * retval:
 *    0 -> OK
 *    1 -> Error
 */
uint8_t DS18B20_Read(OneWire_t* OneWire, float *destination) //read the fucking temperature
{
	// TODO
	return 0;
}

/* Set resolution of the DS18B20
 * param:
 *   OneWire: send through this
 *   resolution: set to this resolution
 * retval:
 *    0 -> OK
 *    1 -> Error
 */
uint8_t DS18B20_SetResolution(OneWire_t* OneWire, DS18B20_Resolution_t resolution)
{
	// TODO
	return 0;
}

/* Check if the temperature conversion is done or not
 * param:
 *   OneWire: send through this
 * retval:
 *    0 -> OK
 *    1 -> Not yet
 */
uint8_t DS18B20_Done(OneWire_t* OneWire)
{
	// TODO
	return 0;
}


#endif
