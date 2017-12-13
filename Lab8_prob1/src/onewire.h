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
/*void OneWire_Init(OneWire_t* OneWireStruct, GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin);
void OneWire_SkipROM(OneWire_t* OneWireStruct);
uint8_t OneWire_Reset(OneWire_t* OneWireStruct);
uint8_t OneWire_ReadByte(OneWire_t* OneWireStruct);
void OneWire_WriteByte(OneWire_t* OneWireStruct, uint8_t byte);
void OneWire_WriteBit(OneWire_t* OneWireStruct, uint8_t bit);
uint8_t OneWire_ReadBit(OneWire_t* OneWireStruct);*/

/* Init OneWire Struct with GPIO information
 * param:
 *   OneWire: struct to be initialized
 *   GPIOx: Base of the GPIO DQ used, e.g. GPIOA
 *   GPIO_Pin: The pin GPIO DQ used, e.g. 5
 */
 //GPIOB PIN8 for thermometer usgae
void OneWire_Init(OneWire_t* OneWireStruct, GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin)
{
	OneWire_Reset();
}

/* Send reset through OneWireStruct
 * Please implement the reset protocol
 * param:
 *   OneWireStruct: wire to send
 * retval:
 *    0 -> Reset OK
 *    1 -> Reset Failed
 */
int OneWire_Reset(/*OneWire_t* OneWireStruct*/)
{
	//thermeter does not init well
	GPIOB->BRR = GPIO_PIN_8;
	ONEWIRE_OUTPUT();
	delay_us(480);
	ONEWIRE_INPUT();
	delay_us(70);
	delay_us(410); //delay more
	int masked_value = (GPIOB->IDR)>>8;
	//now check if the DS18B20 has really done its job of lowering the voltage
    return (masked_value == 0)?1:0;
}

/* Write 1 bit through OneWireStruct
 * Please implement the send 1-bit protocol
 * param:
 *   OneWireStruct: wire to send
 *   bit: bit to send
 */
 //ref DS18B20 pdf 22
void OneWire_WriteBit(/*OneWire_t* OneWireStruct, */int bit)
{
	//the accumulated delay should last at least 60 us
	// delay_us(2); //pdf says the time interval b/w two write operation shouldbe at 1us
	// ONEWIRE_INPUT(); //rise the high voltage to make the negedge
	if(bit) //master write1
	{
		GPIOB->BRR = GPIO_PIN_8; //
		ONEWIRE_OUTPUT(); //master pulls down the DQ
		delay_us(10); //release in 15 us

		ONEWIRE_INPUT();//chenage to input make high

		delay_us(55); //accumulate the time to fit the 60 us criteria
		ONEWIRE_INPUT(); //rise again
	}
	else //master write 0
	{
		GPIOB->BRR = GPIO_PIN_8;
		ONEWIRE_OUTPUT();
		delay_us(65);

		ONEWIRE_INPUT();

		delay_us(5); //it says delay at least 60 us
		ONEWIRE_INPUT();
	}
}

/* Read 1 bit through OneWireStruct
 * Please implement the read 1-bit protocol
 * param:
 *   OneWireStruct: wire to read from
 */
/*
 Due to pull up resistor
If I set the input for DS18B20
則它會因為pullup resistor的關係 變成高電位
ds18b20 如果偵測到我要輸出零 則會壓低電壓反之則會維持高電壓
 * */
int OneWire_ReadBit(/*OneWire_t* OneWireStruct*/)
{
	// TODO
	int data = 0;
	// delay_us(50); //make a delay since the pdf says, each read operation should last as long as 60us
	// ONEWIRE_INPUT(); //rise the high voltage to make the negedge
	GPIOB->BRR = GPIO_PIN_8; // high -> low
	ONEWIRE_OUTPUT();
	delay_us(3);
	//release line
	ONEWIRE_INPUT();
	delay_us(10);

	data = (GPIOB->IDR >> 8) & 0x1;
	delay_us(50); //wait 50 us to compelete 60us period
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
		OneWire_WriteBit(data_to_be_wirtten&0x1);
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
	GPIOB->PUPDR   &= 0b11111111111111001111111111111111;
	GPIOB->PUPDR   |= 0b00000000000000010000000000000000;
	GPIOB->OSPEEDR &= 0b11111111111111001111111111111111;
	GPIOB->OSPEEDR |= 0b00000000000000010000000000000000;
}

void ONEWIRE_OUTPUT() //PB8 output configuration
{
	//GPIOB 8 for one wite central wire
	GPIOB->MODER   &= 0b11111111111111001111111111111111;
	GPIOB->MODER   |= 0b00000000000000010000000000000000;
	GPIOB->PUPDR   &= 0b11111111111111001111111111111111;
	GPIOB->PUPDR   |= 0b00000000000000010000000000000000;
	GPIOB->OSPEEDR &= 0b11111111111111001111111111111111;
	GPIOB->OSPEEDR |= 0b00000000000000010000000000000000;
	GPIOB->OTYPER  |= 0b00000000000000000000000000000000;
}

#endif /* ONEWIRE_H_ */
