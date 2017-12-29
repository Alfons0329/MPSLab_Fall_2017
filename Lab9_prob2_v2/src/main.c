#include <stdio.h>
#include <string.h>
#include "UART.h"
#include "ADC.h"
/*************************Pin configuration*******************************
PA PB PC
Use PA10 for RX (RX for board is receiver and TX for computer is sender)
and PA9 for TX (TX for board is sender and TX for computer is receiver)
Use PB1 for light-sensitive resistor
Use PC13 for user button
**************************************************************************/
//RX push pull TX open drain
uint8_t text[] = "UART FUCKINGLY WORKS HELL YEAH \r\n";
void GPIO_Init(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN; //Turn on GPIO AB and C;
	//UART init use PA14 for RX and PA15 for TX ,RX is the input and TX is the outptu port for the UART
	//the UART part
	//USART1_RX as alternate function PA14 for RX and PA15 for TX
	GPIOA->MODER   	&= 0b11111111110000111111111111111111;
	GPIOA->MODER   	|= 0b00000000001010000000000000000000;
	GPIOA->PUPDR   	&= 0b11111111110000111111111111111111;
	GPIOA->OSPEEDR 	&= 0b11111111110000111111111111111111;
	GPIOA->OSPEEDR 	&= 0b11111111111010111111111111111111;
	GPIOA->OTYPER  	&= 0b11111111111111111111100111111111; //reset is fine
	GPIOA->OTYPER  	|= 0b11111111111111111111110111111111;

	GPIOA->AFR[1] = (GPIOA->AFR[1] & 0xFFFFF00F) | 0x00000770;

	//the light-sensitive resistor part
	//light-sensitive resistor as input
	GPIOB->MODER	&= 0b11111111111111111111111111111100;
	//the User button part
	//User button init
	GPIOC->MODER   	&= 0b11110011111111111111111111111111;
}

int main()
{

	GPIO_Init();
	USART1_Init();
	while(1)
	{
		USART1_Transmit(text,(uint32_t)sizeof(text));
	}

	return 0;
}
