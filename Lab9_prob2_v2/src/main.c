#include <stdio.h>
#include <string.h>
#include "mylib.h"
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
	GPIOA->MODER   &= 0b11111111110000111111111111111111;
	GPIOA->MODER   |= 0b00000000001010000000000000000000;
	GPIOA->PUPDR   &= 0b11111111110000111111111111111111;
	GPIOA->OSPEEDR &= 0b11111111110000111111111111111111;
	GPIOA->OSPEEDR &= 0b11111111111010111111111111111111;
	GPIOA->OTYPER  &= 0b11111111111111111111100111111111; //reset is fine
	GPIOA->OTYPER  |= 0b11111111111111111111110111111111;

	GPIOA->AFR[1] = (GPIOA->AFR[1] & 0xFFFFF00F) | 0x00000770;

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
/* From reference manual
Serial data are transmitted and received through these pins in normal USART mode. The
frames are comprised of:
• An Idle Line prior to transmission or reception
• A start bit
• A data word (7, 8 or 9 bits) least significant bit first
• 0.5, 1, 1.5, 2 stop bits indicating that the frame is complete
• The USART interface uses a baud rate generator
• A status register (USART_ISR)
• Receive and transmit data registers (USART_RDR, USART_TDR)
• A baud rate register (USART_BRR)
• A guard-time register (USART_GTPR) in case of Smartcard mode.
*/
void USART1_Init(void)
{
	//f CK can be f LSE , f HSI , f PCLK , f SYS .,we can just use the clock from STM32, which is 4MHz
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	/*************************UART CR1 setting***********************************
	CR1 clear the bits of M(Data length/8bits is default) PS() PCE() TE RE, then set the bits of TE RE where TE enables the function of transmittion and
	RE enables the function of reception
	*****************************************************************************/
	MODIFY_REG(USART1->CR1, USART_CR1_M | USART_CR1_PS | USART_CR1_PCE | USART_CR1_TE | USART_CR1_RE |
	USART_CR1_OVER8, USART_CR1_TE | USART_CR1_RE);
	// CR2 for how much bit indicating the stop, now 1 bit
	MODIFY_REG(USART1->CR2, USART_CR2_STOP, 0x0); //0x0 for 1-bit stop
	// CR3 clear the bits of RTSE , CTSE and ONEBIT, these are used for RS232, different from our LAB9
	MODIFY_REG(USART1->CR3, (USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT), 0x0);
	/*uint16_t brr15_4 = USART1->BRR & 0b1111111111110000;
	brr15_4 >>= 4;
	uint16_t brr2_0 = USART1->BRR & 0b111;
	brr2_0 <<= 1;
	uint16_t baud_x = brr15_4 | brr2_0;*/
	/*************************************Baud rate setting,********************
	oversampling by16 (since over8 is cleared)
	USARTDIV is how fast the communication port in computer wants to transmit and receive (they should be the same value)
	default terminal setting is 9600, then we set the baud rate = fCK/USARTDIV , for default fCK = fSYS --> 4M / 9.6K
	or check manual p 1319 for 72MHz--> BRR with OVER16 USARTDIV = 9600-->1D4C so for 4MHz is 1D4C/18 about 416(DEC) which = 4MHz/9.6K
	****************************************************************************/
	MODIFY_REG(USART1->BRR, 0xFFFF /*clear all and reset*/, 4000000/9600);

	/**************************asynchronous mode setting************************
	In asynchronous mode, the following bits must be kept cleared:
	- LINEN and CLKEN bits in the USART_CR2 register,
	- SCEN, HDSEL and IREN bits in the USART_CR3 register.
	****************************************************************************/
	USART1->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
	USART1->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);
	// Enable UART
	USART1->CR1 |= (USART_CR1_UE);
}
void USART1_Transmit(uint8_t *arr, uint32_t size)
{
	/*
	Character transmission procedure
	1. Program the M bits in USART_CR1 to define the word length.
	2. Select the desired baud rate using the USART_BRR register.
	3. Program the number of stop bits in USART_CR2.
	4. Enable the USART by writing the UE bit in USART_CR1 register to 1.
	5. Select DMA enable (DMAT) in USART_CR3 if multibuffer communication is to take
	place. Configure the DMA register as explained in multibuffer communication.
	6. Set the TE bit in USART_CR1 to send an idle frame as first transmission.
	7. Write the data to send in the USART_TDR register (this clears the TXE bit). Repeat this
	for each data to be transmitted in case of single buffer.
	8. After writing the last data into the USART_TDR register, wait until TC=1. This indicates
	that the transmission of the last frame is complete. This is required for instance when
	the USART is disabled or enters the Halt mode to avoid corrupting the last
	transmission. */
	for(int i=0;i<size;i++)
	{
		while (!READ_BIT(USART1->ISR, USART_ISR_TXE)); //polling the USART  is ready
		USART1->TDR = arr[i];//transmitt data register, get the data and send to USART port
	}
	while (!READ_BIT(USART1->ISR, USART_ISR_TC));

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
