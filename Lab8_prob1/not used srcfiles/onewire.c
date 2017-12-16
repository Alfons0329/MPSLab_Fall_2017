#include "onewire.h"

/* Init OneWire Struct with GPIO information
 * param:
 *   OneWire: struct to be initialized
 *   GPIOx: Base of the GPIO DQ used, e.g. GPIOA
 *   GPIO_Pin: The pin GPIO DQ used, e.g. 5
 */
void OneWire_Init(OneWire_t* OneWireStruct, GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin) {
	// TODO
}

/* Send reset through OneWireStruct
 * Please implement the reset protocol
 * param:
 *   OneWireStruct: wire to send
 * retval:
 *    0 -> Reset OK
 *    1 -> Reset Failed
 */
uint8_t OneWire_Reset(OneWire_t* OneWireStruct)
{
	// TODO
	ONEWIRE_INPUT();
	GPIOA->BRR = GPIO_PIN_8; // high -> low
	ONEWIRE_OUTPUT();
	ONEWIRE_DELAY(480);
	ONEWIRE_INPUT();
	ONEWIRE_DELAY(70);
	ONEWIRE_DELAY(410);
    return 0;
}

/* Write 1 bit through OneWireStruct
 * Please implement the send 1-bit protocol
 * param:
 *   OneWireStruct: wire to send
 *   bit: bit to send
 */
void OneWire_WriteBit(OneWire_t* OneWireStruct, uint8_t bit)
{
	// TODO
}

/* Read 1 bit through OneWireStruct
 * Please implement the read 1-bit protocol
 * param:
 *   OneWireStruct: wire to read from
 */
uint8_t OneWire_ReadBit(OneWire_t* OneWireStruct)
{
	// TODO
}

/* A convenient API to write 1 byte through OneWireStruct
 * Please use OneWire_WriteBit to implement
 * param:
 *   OneWireStruct: wire to send
 *   byte: byte to send
 */
void OneWire_WriteByte(OneWire_t* OneWireStruct, uint8_t byte)
{
	// TODO
}

/* A convenient API to read 1 byte through OneWireStruct
 * Please use OneWire_ReadBit to implement
 * param:
 *   OneWireStruct: wire to read from
 */
uint8_t OneWire_ReadByte(OneWire_t* OneWireStruct)
{
	// TODO
}

/* Send ROM Command, Skip ROM, through OneWireStruct
 * You can use OneWire_WriteByte to implement
 */
void OneWire_SkipROM(OneWire_t* OneWireStruct)
{
	// TODO
}
void ONEWIRE_INPUT()
{
	//GPIOB 8
	GPIOB->MODER   &= 0b11111111111111001111111111111111;
	GPIOB->PUPDR   &= 0b11111111111111001111111111111111;
	GPIOB->PUPDR   |= 0b00000000000000010000000000000000;
	GPIOB->OSPEEDR &= 0b11111111111111001111111111111111;
	GPIOB->OSPEEDR |= 0b00000000000000010000000000000000;
	GPIOB->OTYPER  |= 0b00000000000000000000000100000000;
}

void ONEWIRE_OUTPUT()
{
	ㄆㄆ
	GPIOB->MODER   &= 0b11111111111111001111111111111111;
	GPIOB->MODER   |= 0b00000000000000010000000000000000;
	GPIOB->PUPDR   &= 0b11111111111111001111111111111111;
	GPIOB->PUPDR   |= 0b00000000000000010000000000000000;
	GPIOB->OSPEEDR &= 0b11111111111111001111111111111111;
	GPIOB->OSPEEDR |= 0b00000000000000010000000000000000;
	GPIOB->OTYPER  |= 0b00000000000000000000000100000000;
}
