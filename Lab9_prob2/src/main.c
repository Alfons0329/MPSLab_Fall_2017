#include <stdio.h>
#include <string.h>
#include "stm32l476xx.h"
#include "mylib.h"
#include "gpio.h"
//Pin configuration
//PA PB PC
//Use PA14 for RX and PA15 for TX where RX is the receiver and TX is the transmitter (data output)
//Use PB for light-sensitive resistor
extern float resistor;
uint8_t text [] = "helloworld";
void GPIO_Init(void)
{
	RCC->AHB2ENR 	|= 0x7; //Turn on GPIO AB and C;
	//UART init use PA14 for RX and PA15 for TX ,RX is the input and TX is the outptu port for the UART
	//the UART part
	// USART1_RX as alternate function PA14 for RX and PA15 for TX
	TM_GPIO_Init(GPIOA, 14, TM_GPIO_Mode_AF, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
	// USART1_TX as alternate function PA14 for RX and PA15 for TX
	TM_GPIO_Init(GPIOA, 15, TM_GPIO_Mode_AF, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
	//light-sensitive resistor as input
	TM_GPIO_Init(GPIOB, 1, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low); //does not matter for input
	//User button init
	TM_GPIO_Init(GPIOC, 13, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low); //does not matter for input
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
int USART1_Transmit(uint8_t *arr, uint32_t size)
{
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
	for(int i=0;<size;i++)
	{
		while(!); //polling the USART device is ready
		USART1->TDR = arr[i];//transmitt data register, get the data and send to USART port
	}
	/********************************************************************************
	After	writing	the	last	data	into	the	USART_TDR	register,	wait	until
	TC=1.	This	indicates	that	the	transmission	of	the	last	frame	is
	complete
	Useing polling the USART device until all is done for the procedure
	*******************************************************************************/
	while();
	return ;
}
int main()
{
	GPIO_Init();

}
