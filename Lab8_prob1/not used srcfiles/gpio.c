/**
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen Majerle
 * |
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software,
 * | and to permit persons to whom the Software is furnished to do so,
 * | subject to the following conditions:
 * |
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * |
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */
#include "gpio.h"

void TM_GPIO_INT_EnableClock(GPIO_TypeDef* GPIOx);
void TM_GPIO_INT_Init(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, TM_GPIO_Mode_t GPIO_Mode, TM_GPIO_OType_t GPIO_OType, TM_GPIO_PuPd_t GPIO_PuPd, TM_GPIO_Speed_t GPIO_Speed);
void TM_GPIO_Init(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, TM_GPIO_Mode_t GPIO_Mode, TM_GPIO_OType_t GPIO_OType, TM_GPIO_PuPd_t GPIO_PuPd, TM_GPIO_Speed_t GPIO_Speed) {
	/* Check input */
	if (GPIO_Pin == 0x00) {
		return;
	}

	/* Enable clock for GPIO */
	TM_GPIO_INT_EnableClock(GPIOx);

	/* Do initialization */
	TM_GPIO_INT_Init(GPIOx, GPIO_Pin, GPIO_Mode, GPIO_OType, GPIO_PuPd, GPIO_Speed);
}

void TM_GPIO_SetPullResistor(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, TM_GPIO_PuPd_t GPIO_PuPd) {
	/* Set GPIO PUPD register */
	GPIOx->PUPDR = (GPIOx->PUPDR & ~(0x03 << (2 * GPIO_Pin))) | ((uint32_t)(GPIO_PuPd << (2 * GPIO_Pin)));

}

void TM_GPIO_SetPinAsInput(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin) {
	/* Set 00 bits combination for input */
	GPIOx->MODER &= ~(0x03 << (2 * GPIO_Pin));
}

void TM_GPIO_SetPinAsOutput(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin) {
	/* Set 01 bits combination for output */
	GPIOx->MODER = (GPIOx->MODER & ~(0x03 << (2 * GPIO_Pin))) | (0x01 << (2 * GPIO_Pin));
}

uint32_t TM_GPIO_GetPortSource(GPIO_TypeDef* GPIOx) {
	/* Get port source number */
	/* Offset from GPIOA                       Difference between 2 GPIO addresses */
	return ((uint32_t)GPIOx - (GPIOA_BASE)) / ((GPIOB_BASE) - (GPIOA_BASE));
}


void TM_GPIO_INT_EnableClock(GPIO_TypeDef* GPIOx) {
	RCC->AHB2ENR |= (1 << TM_GPIO_GetPortSource(GPIOx));
}

void TM_GPIO_INT_Init(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, TM_GPIO_Mode_t GPIO_Mode, TM_GPIO_OType_t GPIO_OType, TM_GPIO_PuPd_t GPIO_PuPd, TM_GPIO_Speed_t GPIO_Speed) {
	uint8_t pinpos;

	/* Set GPIO PUPD register */
	GPIOx->PUPDR = (GPIOx->PUPDR & ~(0x03 << (2 * GPIO_Pin))) | ((uint32_t)(GPIO_PuPd << (2 * GPIO_Pin)));

	/* Set GPIO MODE register */
	GPIOx->MODER = (GPIOx->MODER & ~((uint32_t)(0x03 << (2 * GPIO_Pin)))) | ((uint32_t)(GPIO_Mode << (2 * GPIO_Pin)));

	/* Set only if output or alternate functions */
	if (GPIO_Mode == TM_GPIO_Mode_OUT || GPIO_Mode == TM_GPIO_Mode_AF) {
		/* Set GPIO OTYPE register */
		GPIOx->OTYPER = (GPIOx->OTYPER & ~(uint32_t)(0x01 << GPIO_Pin)) | ((uint32_t)(GPIO_OType << GPIO_Pin));

		/* Set GPIO OSPEED register */
		GPIOx->OSPEEDR = (GPIOx->OSPEEDR & ~((uint32_t)(0x03 << (2 * GPIO_Pin)))) | ((uint32_t)(GPIO_Speed << (2 * GPIO_Pin)));
	}
}

