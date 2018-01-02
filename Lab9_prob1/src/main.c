#include <stdio.h>
#include <string.h>
#include "lcd.h"
#include "UART.h"

/*************************Pin configuration*******************************
PA PB PC
Use PA10 for RX (RX for board is receiver and TX for computer is sender)
and PA9 for TX (TX for board is sender and TX for computer is receiver)
Use PB0 for light-sensitive resistor
Use PC13 for user button
**************************************************************************/
//RX push pull TX open drain
uint8_t text[] = "UART FUCKINGLY WORKS HELL YEAH \r\n";
uint8_t buf[50];
char msg[10];
int pos_l=0;
char cmd[128];
int	mode;	/* 0: unread
		 * 1: finished reading
		 * 2: light mode
		 */

void GPIO_Init()
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

	//The light-sensitive resistor part
	//Light-sensitive resistor as ANALOG input for ADC, we now use pb0 for the purpose
	GPIOB->MODER	&= 0b11111111111111111111111111111100;
	GPIOB->MODER	|= 0b11111111111111111111111111111111;
	//The user button part
	//User button init
	GPIOC->MODER   	&= 0b11110011111111111111111111111111;
}

void INT_setup (void)
{
	/* set up the NVIC priorities */
	NVIC_SetPriority (USART1_IRQn, 0);
	NVIC_SetPriority (SysTick_IRQn, 1);
	NVIC_SetPriority (ADC1_2_IRQn, 2);

	/* enable the NVIC IRQ for uart */
	uart_IRQE ();
}

void USART1_IRQHandler (void)
{
	int	i;
	char	s[2];

	switch (mode) {
	case 0:
		s[0] = USART1->RDR;
		s[1] = 0;
		if (s[0] == '\177') {
			if (strlen (cmd) == 0)
				break;
			cmd[strlen (cmd) - 1] = 0;
			uart_write ("\r");
			for (i = 0; i < strlen (cmd) + 3; ++i)
				uart_write (" ");
			uart_write ("\r> ");
			uart_write (cmd);
			break;
		}
		uart_write (s);
		strcat (cmd, s);
		if (s[0] == '\n')
			mode = 1;
		break;
	case 1:
		break;
	case 2:
		s[0] = USART1->RDR;
		if (s[0] == 'q') {
			mode = 0;
			SysTick_stop ();
			ADC_stop ();
		}
		break;
	}
}


int main()
{
	GPIO_Init();
	USART1_Init();
	LCD_init();
	INT_setup();
	//configureADC();
	//startADC();
	while(1)
	{
		readline();
		write_str_to_LCD (msg);
		strcat (msg," showed\r\n");
		USART1_Transmit((uint8_t *)msg, strlen(msg));
	}

	return 0;
}
