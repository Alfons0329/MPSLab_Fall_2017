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


int global_temperature;

int DS18B20_ConvT(OneWire_t* OneWire, DS18B20_Resolution_t precision);
/*uint8_t DS18B20_Read(/*OneWire_t* OneWireStruct, float* destination*/
/*uint8_t DS18B20_SetResolution(OneWire_t* OneWireStruct, DS18B20_Resolution_t resolution);
uint8_t DS18B20_Done(OneWire_t* OneWireStruct);*/
/* Send ConvT through OneWire with resolution
 * param:
 *   OneWire: send through this
 *   resolution: temperature resolution
 * retval:
 *    0 -> OK
 *    1 -> Error
 */
 //setting the resolution of thermometer, default is 12 bit type
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
//ONEWIRE_DELAY parameter which passed in is us (1E-6 second)
//Let's read the fucking temperature
uint8_t DS18B20_Read(/*OneWire_t* OneWire, float *destination*/)
{
	// TODO
	global_temperature = 0;
	OneWire_Reset(OneWire_B);//reset all the state first, or say re-initilize the one wire system
	OneWire_SkipROM();
	OneWire_WriteByte(0x44);//tell the one wire thermometer
	ONEWIRE_DELAY(750000); //The required time for ADC conversion 750ms=750000us
	OneWire_SkipROM();
	OneWire_WriteByte(0xBE); //Read the scratch pad for temperature data
	OneWire_Reset(OneWire_B);//reset all the state first, or say re-initilize the one wire system
	//sequential read
	for(int i=0;i<16;i++)
	{
		global_temperature |= OneWire_ReadBit()<<i;
	}
	OneWire_Reset(OneWire_B);//reset all the state first, or say re-initilize the one wire system
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
 //set the default resolution or not?
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
