#include <stdio.h>
#include <string.h>
#include "mylib.h"
//Pin configuration
//PA PB PC
//Use PA14 for RX and PA15 for TX where RX is the receiver and TX is the transmitter (data output)
//Use PB1 for light-sensitive resistor
//Use PC13 for user button
uint8_t text [] = "helloworld";
void GPIO_Init(void)
{
	RCC->AHB2ENR 	|= 0x7; //Turn on GPIO AB and C;
	//UART init use PA14 for RX and PA15 for TX ,RX is the input and TX is the outptu port for the UART
	//the UART part
	//USART1_RX as alternate function PA14 for RX and PA15 for TX
	GPIOA->MODER   &= 0b00001111111111111111111111111111;
	GPIOA->MODER   |= 0b11110000000000000000000000000000;
	GPIOA->PUPDR   &= 0b00001111111111111111111111111111;
	GPIOA->OSPEEDR &= 0b00001111111111111111111111111111;
	//GPIOA->OTYPER  &= 0b11111111111111111111100111111111; reset is fine
	//the light-sensitive resistor part
	//light-sensitive resistor as input

	//the User button part
	//User button init
	GPIOC->MODER   &= 0b11110011111111111111111111111111;
}
void configureADC()
{
	// TODO
}
void startADC()
{
	// TODO
}
void USART1_Init(void)
{
	/* Enable clock for USART??? */
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	// CR1
	MODIFY_REG(USART1->CR1, USART_CR1_M | USART_CR1_PS | USART_CR1_PCE | USART_CR1_TE | USART_CR1_RE |
	USART_CR1_OVER8, USART_CR1_TE | USART_CR1_RE);
	// CR2 for how much bit indicating the stop, now 1 bit
	MODIFY_REG(USART1->CR2, USART_CR2_STOP, 0x0); // 1-bit stop
	// CR3
	MODIFY_REG(USART1->CR3, (USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT), 0x0); // none hwflowctl
	MODIFY_REG(USART1->BRR, 0xFF, 4000000L/9600L);
	/* In asynchronous mode, the following bits must be kept cleared:
	- LINEN and CLKEN bits in the USART_CR2 register,
	- SCEN, HDSEL and IREN bits in the USART_CR3 register.*/
	USART1->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
	USART1->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);
	// Enable UART
	USART1->CR1 |= (USART_CR1_UE);
}
void USART1_Transmit(uint8_t *arr, uint32_t size)
{
	/*
	 Transmission Procedure
	• Program	the	M	bits	in	USART_CR1	to	define	the	word	length.
	• Select	the	desired	baud	rate	using	the	USART_BRR	register.
	• Program	the	number	of	stop	bits	in	USART_CR2.
	• Enable	the	USART
	• Write	the	data	to	send	in	the	USART_TDR	register
	• After	writing	the	last	data	into	the	USART_TDR	register,	wait	until
	TC=1.	This	indicates	that	the	transmission	of	the	last	frame	is
	complete
	 * */
	/* This is the data structure of UART
	typedef struct
	{
	  __IO uint32_t CR1;         !< USART Control register 1,                 Address offset: 0x00
	  __IO uint32_t CR2;         !< USART Control register 2,                 Address offset: 0x04
	  __IO uint32_t CR3;         !< USART Control register 3,                 Address offset: 0x08
	  __IO uint32_t BRR;         !< USART Baud rate register,                 Address offset: 0x0C
	  __IO uint16_t GTPR;        !< USART Guard time and prescaler register,  Address offset: 0x10
	  uint16_t  RESERVED2;       !< Reserved, 0x12
	  __IO uint32_t RTOR;        !< USART Receiver Time Out register,         Address offset: 0x14
	  __IO uint16_t RQR;         !< USART Request register,                   Address offset: 0x18
	  uint16_t  RESERVED3;       !< Reserved, 0x1A
	  __IO uint32_t ISR;         !< USART Interrupt and status register,      Address offset: 0x1C
	  __IO uint32_t ICR;         !< USART Interrupt flag Clear register,      Address offset: 0x20
	  __IO uint16_t RDR;         !< USART Receive Data register,              Address offset: 0x24
	  uint16_t  RESERVED4;       !< Reserved, 0x26
	  __IO uint16_t TDR;         !< USART Transmit Data register,             Address offset: 0x28
	  uint16_t  RESERVED5;       !< Reserved, 0x2A
  } USART_TypeDef;
	*/
	for(int i=0;i<size;i++)
	{
		// while(!); //polling the USART device is ready
		USART1->TDR = arr[i];//transmitt data register, get the data and send to USART port
	}
	/********************************************************************************
	After	writing	the	last	data	into	the	USART_TDR	register,	wait	until
	TC=1.	This	indicates	that	the	transmission	of	the	last	frame	is
	complete
	Useing polling the USART device until all is done for the procedure
	*******************************************************************************/
	while (!READ_BIT(USART1->ISR, USART_ISR_TC));

}
int main()
{
	GPIO_Init();
	USART1_Init();
	while(1)
	{
		USART1_Transmit(text,(uint32_t)strlen(text));
	}

	return 0;
}
