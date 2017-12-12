#ifndef ONEWIRE_H_
#define ONEWIRE_H_

#include "gpio.h"

extern void delay_us(); //from delay_us.s

typedef struct
{
	GPIO_TypeDef* GPIOx;           /*!< GPIOx port to be used for I/O functions */
	uint32_t GPIO_Pin;             /*!< GPIO Pin to be used for I/O functions */
} OneWire_t;
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
	// TODO
	ONEWIRE_INPUT();
	GPIOB->BRR = GPIO_PIN_8; // high -> low
	ONEWIRE_OUTPUT();
	GPIOB->ODR = 0<<8;
	ONEWIRE_DELAY(480);
	ONEWIRE_INPUT();
	ONEWIRE_DELAY(70); //wait the so-called 15-60 for lowering the voltage
	//now check if the DS18B20 has really done its job of lowering the voltage
	while((GPIOB->IDR)>>8 == 1); //busy waiting until the volatage is lowered to 1
	int masked_value = (GPIOB->IDR)>>8;
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
	ONEWIRE_DELAY(2); /*pdf says the time interval b/w two write operation should
	be at 1us*/
	if(bit) //master write1
	{
		GPIOB->BRR = GPIO_PIN_8;
		ONEWIRE_OUTPUT();
		ONEWIRE_DELAY(5); //Release in 15 us
		ONEWIRE_INPUT();//chenage to input
		ONEWIRE_DELAY(60); //accumulate the time to fit the 60 us criteria
	}
	else
	{
		GPIOB->BRR = GPIO_PIN_8;
		ONEWIRE_OUTPUT();
		ONEWIRE_DELAY(70); //it says delay at least 60 us
	}
}

/* Read 1 bit through OneWireStruct
 * Please implement the read 1-bit protocol
 * param:
 *   OneWireStruct: wire to read from
 */
int OneWire_ReadBit(OneWire_t* OneWireStruct)
{
	// TODO
	int data = 0;
	ONEWIRE_DELAY(45); //make a delay since the pdf says, each read operation should last as long as 60us
	ONEWIRE_INPUT();
	GPIOB->BRR = GPIO_PIN_8; // high -> low
	ONEWIRE_OUTPUT();
	ONEWIRE_DELAY(1);
	ONEWIRE_INPUT();
	data = (GPIOB->IDR >> 8) & 0x1;
	return data;
}

/* A convenient API to write 1 byte through OneWireStruct
 * Please use OneWire_WriteBit to implement
 * param:
 *   OneWireStruct: wire to send
 *   byte: byte to send
 */
void OneWire_WriteByte(/*OneWire_t* OneWireStruct*/, int data_to_be_wirtten)
{
	//DATA IS SENT FROM LSB!!!! TO MSB!!! RIGHT TO LEFT
	for(int i=0;i<8;i++)
	{
		OneWire_WriteBit(data&0x1);
		data >>= 1;
	}
}

/* A convenient API to read 1 byte through OneWireStruct
 * Please use OneWire_ReadBit to implement
 * param:
 *   OneWireStruct: wire to read from
 */
 //read from LSB to MSB!!
int OneWire_ReadByte(OneWire_t* OneWireStruct)
{
	int data = 0;
	for(int i=0;i<8;i++)
	{
		data |= OneWire_ReadBit();
		data <<= 1;
	}
	return data;
	//shift and use bitwise or to make the bit read from LSB to MSB, bit by bit
	//and finally, all the data has been successfully parsed.
}

/* Send ROM Command, Skip ROM, through OneWireStruct
 * You can use OneWire_WriteByte to implement
 */
void OneWire_SkipROM(/*OneWire_t* OneWireStruct*/)
{
	OneWire_WriteByte(0xCC); //skip ROM command, since there is only 1 thermometer
}
void ONEWIRE_INPUT() //PB8 input configuration
{
	//GPIOB 8 for one wite central wire
	GPIOB->MODER   &= 0b11111111111111001111111111111111;
	GPIOB->PUPDR   &= 0b11111111111111001111111111111111;
	GPIOB->PUPDR   |= 0b00000000000000010000000000000000;
	GPIOB->OSPEEDR &= 0b11111111111111001111111111111111;
	GPIOB->OSPEEDR |= 0b00000000000000010000000000000000;
	GPIOB->OTYPER  |= 0b00000000000000000000000100000000;
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
	GPIOB->OTYPER  |= 0b00000000000000000000000100000000;
}
void ONEWIRE_DELAY(unsigned u_sec)
{
	for(int i=0;i<u_sec;i++)
	{
		delay_us();
	}
}
#endif /* ONEWIRE_H_ */
