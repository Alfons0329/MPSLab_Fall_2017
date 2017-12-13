#ifndef TM_DS18B20_H
#define TM_DS18B20_H
#include "onewire.h"
#include "ref.h"
typedef enum
{
	TM_DS18B20_Resolution_9bits = 9,   /*!< DS18B20 9 bits resolution */
	TM_DS18B20_Resolution_10bits = 10, /*!< DS18B20 10 bits resolution */
	TM_DS18B20_Resolution_11bits = 11, /*!< DS18B20 11 bits resolution */
	TM_DS18B20_Resolution_12bits = 12  /*!< DS18B20 12 bits resolution */
} DS18B20_Resolution_t;


int global_temperature,cnt2;

int DS18B20_ConvT(OneWire_t* OneWire, DS18B20_Resolution_t precision);
/*uint8_t DS18B20_Read(OneWire_t* OneWireStruct, float* destination*/
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
	//I think the default is OK
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
//delay_us parameter which passed in is us (1E-6 second)
//Let's read the fucking temperature
/*
Each of the operation has to go thorugh the following 3 steps
1.initialization
2.ROM command
3.DS18B20 Function Command
*/
uint8_t DS18B20_Read(/*OneWire_t* OneWire, float *destination*/)
{
	global_temperature = 0;
	OneWire_Reset();//reset all the state first, or say re-initilize the one wire system
	OneWire_SkipROM();//only one thermometer, so skip it
	//OneWire_Reset();
	OneWire_WriteByte(0x44);//tell the one wire thermometer I want the ADC thermal conversion
	delay_us(750000); //The required time for ADC conversion 750ms=750000us
	//but I will give the system a bit more time to do, doesnt need to be so strict
	OneWire_Reset(); //reset for next command
	OneWire_SkipROM();//only one thermometer, so skip it
	//OneWire_Reset();
	OneWire_WriteByte(ONEWIRE_CMD_RSCRATCHPAD); //Read the scratch pad for temperature data
	//sequential read
	int cnt=0, read_data=0;
	for(int i=0;i<16;i++)
	{
		read_data=OneWire_ReadBit();
		if(i>=4 && i<=10)
		{
			global_temperature |= read_data<<cnt;
			cnt++;
		}
	}
	//global_temperature=cnt2%global_temperature;
	//cnt2++;
	OneWire_Reset();//reset all the state first, or say re-initilize the one wire system
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
	return 0;
}


#endif
