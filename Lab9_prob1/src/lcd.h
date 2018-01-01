#include <stdio.h>
#include <string.h>
#include "mylib.h"

/**********************************************************************
 * connected to		pin name	MCU pin		mode
 * ===============================================================
 *
 * LCD RS		D4		PB5		output
 * LCD R/W		D3		PB3		output
 * LCD E		D2		PA10		output
 * LCD D0		A0		PA0		output
 * LCD D1		A1		PA1		output
 * LCD D2		D7		PA8		output
 * LCD D3		D8		PA9		output
 * LCD D4		A2		PA4		output
 * LCD D5		D13		PA5		output
 * LCD D6		D12		PA6		output
 * LCD D7		D11		PA7		output
 *
 **********************************************************************/
void LCD_write (int data, int RS);
static void LCD_init_GPIO (void);
void LCD_init (void)
{
	LCD_init_GPIO();
	LCD_write (0x38, 0);	/* function setting */
	LCD_write (0x06, 0);	/* entering mode */
	LCD_write (0x0c, 0);	/* display on */
	LCD_write (0x01, 0);	/* clear screen */
	LCD_write (0x80, 0);	/* reset the DD RAM address */
}

void LCD_write (int data, int RS)
{
	/* LCD R/W pin = 0 (write) */
	GPIOB->BRR |= GPIO_BRR_BR3;
	/* LCD RS pin = RS (0: inst, 1: data) */
	GPIOB->ODR &= ~GPIO_ODR_OD5_Msk;
	GPIOB->ODR |= RS << 5;

	/* set D0 */
	GPIOA->ODR &= ~GPIO_ODR_OD0_Msk;
	GPIOA->ODR |= (data % 2) << 0;
	data /= 2;
	/* set D1 */
	GPIOA->ODR &= ~GPIO_ODR_OD1_Msk;
	GPIOA->ODR |= (data % 2) << 1;
	data /= 2;
	/* set D2 */
	GPIOA->ODR &= ~GPIO_ODR_OD8_Msk;
	GPIOA->ODR |= (data % 2) << 8;
	data /= 2;
	/* set D3 */
	GPIOA->ODR &= ~GPIO_ODR_OD9_Msk;
	GPIOA->ODR |= (data % 2) << 9;
	data /= 2;
	/* set D4 */
	GPIOA->ODR &= ~GPIO_ODR_OD4_Msk;
	GPIOA->ODR |= (data % 2) << 4;
	data /= 2;
	/* set D5 */
	GPIOA->ODR &= ~GPIO_ODR_OD5_Msk;
	GPIOA->ODR |= (data % 2) << 5;
	data /= 2;
	/* set D6 */
	GPIOA->ODR &= ~GPIO_ODR_OD6_Msk;
	GPIOA->ODR |= (data % 2) << 6;
	data /= 2;
	/* set D7 */
	GPIOA->ODR &= ~GPIO_ODR_OD7_Msk;
	GPIOA->ODR |= (data % 2) << 7;
	data /= 2;

	/* set E high */
	GPIOA->BSRR |= GPIO_BSRR_BS10;
	/* delay for a while (10 ms) */
	delay_ms (10);
	/* set E low */
	GPIOA->BRR |= GPIO_BRR_BR10;
	/* delay for a while (10 ms) */
	delay_ms (10);
}

static void LCD_init_GPIO(void)
{
	/* enable AHB2 clock for port A, B */
	RCC->AHB2ENR |= 0x3;

	/* set PA0, 1, 4, 5, 6, 7, 8, 9, 10 as output mode */
	GPIOA->MODER &= ~ (GPIO_MODER_MODE0_Msk |
			GPIO_MODER_MODE1_Msk |
			GPIO_MODER_MODE4_Msk |
			GPIO_MODER_MODE5_Msk |
			GPIO_MODER_MODE6_Msk |
			GPIO_MODER_MODE7_Msk |
			GPIO_MODER_MODE8_Msk |
			GPIO_MODER_MODE9_Msk |
			GPIO_MODER_MODE10_Msk);
	GPIOA->MODER |= GPIO_MODER_MODE0_0 |
			GPIO_MODER_MODE1_0 |
			GPIO_MODER_MODE4_0 |
			GPIO_MODER_MODE5_0 |
			GPIO_MODER_MODE6_0 |
			GPIO_MODER_MODE7_0 |
			GPIO_MODER_MODE8_0 |
			GPIO_MODER_MODE9_0 |
			GPIO_MODER_MODE10_0;
	/* set PB3, 5 as output mode */
	GPIOB->MODER &= ~ (GPIO_MODER_MODE3_Msk | GPIO_MODER_MODE5_Msk);
	GPIOB->MODER |= GPIO_MODER_MODE3_0 | GPIO_MODER_MODE5_0;

	/* set PA0, PA1, PA4, PA5, PA6, PA7, PA8, PA9, PA10 and PB3, PB5 as pull-up outputs */
	GPIOA->PUPDR &= ~ (GPIO_PUPDR_PUPD0_Msk |
			GPIO_PUPDR_PUPD1_Msk |
			GPIO_PUPDR_PUPD4_Msk |
			GPIO_PUPDR_PUPD5_Msk |
			GPIO_PUPDR_PUPD6_Msk |
			GPIO_PUPDR_PUPD7_Msk |
			GPIO_PUPDR_PUPD8_Msk |
			GPIO_PUPDR_PUPD9_Msk |
			GPIO_PUPDR_PUPD10_Msk);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD0_0 |
			GPIO_PUPDR_PUPD1_0 |
			GPIO_PUPDR_PUPD4_0 |
			GPIO_PUPDR_PUPD5_0 |
			GPIO_PUPDR_PUPD6_0 |
			GPIO_PUPDR_PUPD7_0 |
			GPIO_PUPDR_PUPD8_0 |
			GPIO_PUPDR_PUPD9_0 |
			GPIO_PUPDR_PUPD10_0;
	GPIOB->PUPDR &= ~ (GPIO_PUPDR_PUPD3_Msk | GPIO_PUPDR_PUPD5_Msk);
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD3_0 | GPIO_PUPDR_PUPD5_0;

	/* set PA0, 1, 4, 5, 6, 7, 8, 9, 10 as medium speed mode */
	GPIOA->OSPEEDR &= ~ (GPIO_OSPEEDR_OSPEED0_Msk |
			GPIO_OSPEEDR_OSPEED1_Msk |
			GPIO_OSPEEDR_OSPEED4_Msk |
			GPIO_OSPEEDR_OSPEED5_Msk |
			GPIO_OSPEEDR_OSPEED6_Msk |
			GPIO_OSPEEDR_OSPEED7_Msk |
			GPIO_OSPEEDR_OSPEED8_Msk |
			GPIO_OSPEEDR_OSPEED9_Msk |
			GPIO_OSPEEDR_OSPEED10_Msk);
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED0_0 |
			GPIO_OSPEEDR_OSPEED1_0 |
			GPIO_OSPEEDR_OSPEED4_0 |
			GPIO_OSPEEDR_OSPEED5_0 |
			GPIO_OSPEEDR_OSPEED6_0 |
			GPIO_OSPEEDR_OSPEED7_0 |
			GPIO_OSPEEDR_OSPEED8_0 |
			GPIO_OSPEEDR_OSPEED9_0 |
			GPIO_OSPEEDR_OSPEED10_0;
	/* set PB3, 5 as medium speed mode */
	GPIOB->OSPEEDR &= ~ (GPIO_OSPEEDR_OSPEED3_Msk | GPIO_OSPEEDR_OSPEED5_Msk);
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED3_0 | GPIO_OSPEEDR_OSPEED5_0;
}
