#ifndef ONEWIRE_H_
#define ONEWIRE_H_
#define GPIO_PIN_8   ((uint16_t) 0x0100)
#define ONEWIRE_CMD_RSCRATCHPAD			0xBE
#define ONEWIRE_CMD_WSCRATCHPAD			0x4E
#define ONEWIRE_CMD_CPYSCRATCHPAD		0x48
#define ONEWIRE_CMD_RECEEPROM			0xB8
#define ONEWIRE_CMD_RPWRSUPPLY			0xB4
#define ONEWIRE_CMD_SEARCHROM			0xF0
#define ONEWIRE_CMD_READROM				0x33
#define ONEWIRE_CMD_MATCHROM			0x55
#define ONEWIRE_CMD_SKIPROM				0xCC
#include "gpio.h"
#include "ref.h" //some useful function can be found here
typedef struct
{
	GPIO_TypeDef* GPIOx;           /*!< GPIOx port to be used for I/O functions */
	uint32_t GPIO_Pin;             /*!< GPIO Pin to be used for I/O functions */
}	OneWire_t;
 //GPIOB PIN8 for thermometer usgae
 /*
  Due to pull up resistor
 If I set the input for DS18B20
 則它會因為pullup resistor的關係 變成高電位
 ds18b20 如果偵測到我要輸出零 則會壓低電壓反之則會維持高電壓
 */
/* Send reset through OneWireStruct
 * Please implement the reset protocol
 * param:
 *   OneWireStruct: wire to send
 * retval:
 *    0 -> Reset OK
 *    1 -> Reset Failed
 */
int OneWire_Reset()
{
	ONEWIRE_INPUT();
	ONEWIRE_OUTPUT();
	GPIOB->ODR = 0x0; // high -> low
	delay_us(480);
	ONEWIRE_INPUT();
	delay_us(70);
	int masked_value = ((GPIOB->IDR)>>8) & 0x1;
	delay_us(410);
    return (masked_value == 0)?1:0;
}

/* Write 1 bit through OneWireStruct
 * Please implement the send 1-bit protocol
 * param:
 *   OneWireStruct: wire to send
 *   bit: bit to send
 */
 //ref DS18B20 pdf 22
void OneWire_WriteBit(int bit)
{
	delay_us(2); //pdf says the time interval b/w two write operation shouldbe at 1us
	ONEWIRE_INPUT(); //rise the high voltage to make the required negedge pulse signal
	if(bit) //master write1
	{
		ONEWIRE_OUTPUT(); //master pulls down the DQ
		GPIOB->ODR = 0x0;
		ONEWIRE_INPUT();//chenage to input make high
		delay_us(55); //accumulate the time to fit the 60 us criteria
	}
	else //master write 0
	{
		ONEWIRE_OUTPUT(); //master pulls down the DQ
		GPIOB->ODR = 0x0;
		delay_us(70); //accumulate the time to fit the 60 us criteria
	}
	ONEWIRE_INPUT(); //rise again for pulse , if does not implement this, temperature will not be updated
	//this is also a "KIND OF" release line
}

/* Read 1 bit through OneWireStruct
 * Please implement the read 1-bit protocol
 * param:
 *   OneWireStruct: wire to read from
 */

int OneWire_ReadBit()
{
	// TODO
	int data = 0;
	ONEWIRE_INPUT(); //rise the high voltage to make the required required negedge pulse signal pulse signal
	ONEWIRE_OUTPUT();//master pulls down the DQ
	GPIOB->ODR = 0x0;
	delay_us(3);//required delay
	//release line
	ONEWIRE_INPUT();
	data = (GPIOB->IDR >> 8) & 0x1;//mask the answer
	delay_us(60); //wait 60 us to forcily compelete 60us period
	return data;
}

/* A convenient API to write 1 byte through OneWireStruct
 * Please use OneWire_WriteBit to implement
 * param:
 *   OneWireStruct: wire to send
 *   byte: byte to send
 */
void OneWire_WriteByte(int data_to_be_wirtten)
{
	//DATA IS SENT FROM LSB!!!! TO MSB!!! RIGHT TO LEFT
	for(int i=0;i<8;i++)
	{
		OneWire_WriteBit(data_to_be_wirtten & 0x1);
		data_to_be_wirtten >>= 1;
	}
}
/* A convenient API to read 1 byte through OneWireStruct
 * Please use OneWire_ReadBit to implement
 * param:
 *   OneWireStruct: wire to read from
 */
int OneWire_ReadByte(OneWire_t* OneWireStruct)
{
	//read from LSB to MSB!!
	//shift and use bitwise or to make the bit read from LSB to MSB, bit by bit
	//and finally, all the data has been successfully parsed.
	int data = 0;
	for(int i=0;i<8;i++)
	{
		data |= OneWire_ReadBit();
		data <<= 1;
	}
	return data;
}

/* Send ROM Command, Skip ROM, through OneWireStruct
 * You can use OneWire_WriteByte to implement
 */
void OneWire_SkipROM(/*OneWire_t* OneWireStruct*/)
{
	OneWire_WriteByte(ONEWIRE_CMD_SKIPROM); //skip ROM command, since there is only 1 thermometer
}
void ONEWIRE_INPUT() //PB8 input configuration
{
	//GPIOB 8 for one wite central wire
	GPIOB->MODER   &= 0b11111111111111001111111111111111;
}

void ONEWIRE_OUTPUT() //PB8 output configuration
{
	//GPIOB 8 for one wite central wire
	GPIOB->MODER   = 0b00000000000000010000000000000000;
}

#endif /* ONEWIRE_H_ */
