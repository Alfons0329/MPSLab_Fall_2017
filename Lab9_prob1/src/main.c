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
extern float resistor_value;
extern void fpu_enable();
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

int readline() {
	char c;
	int pos = 0;
	do {
		UART_Receive(&c);
		if (c == '\r') {
			USART1_Transmit((uint8_t *) "\r\n", 2);
			c = '\0';
		}
		else {
			USART1_Transmit((uint8_t *) &c, 1);
		}
		if (c == '\x7F') {
			pos = pos > 0 ? pos - 1 : 0;
		}
		else {
			msg[pos] = c;
			pos++;
		}
	} while (c != '\0');
	return pos;
}

int stridx (int pos)
{
	return pos / 0x40 * 16 + pos % 0x40;
}

void pos_inc (void)
{
	++pos_l;
	if (pos_l > 0x0f && pos_l < 0x40)
		pos_l = 0x40;
	else if (pos_l > 0x4f)
		pos_l = 0;
}

void write_str_to_LCD (const char *s)
{
	if (stridx(pos_l) == strlen(s)) {
		LCD_write (0x01, 0);			/* clear screen */
		pos_l = 0;				/* reset the position */
	} else {
		LCD_write (0x80 + pos_l, 0);		/* set up the DD RAM address */
		LCD_write (s[stridx (pos_l)], 1);		/* write a character from the string */
		pos_inc ();
	}
}

int main()
{
	//fpu_enable();
	GPIO_Init();
	USART1_Init();
	LCD_init();
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
