#include "stm32l476xx.h"
#include "core_cm4.h"
#include <stdio.h>
#include <stdlib.h>

extern void GPIO_init();
extern void max7219_send(unsigned char address, unsigned char data);
extern void max7219_init();

int keypad_value[4][4] ={	{1,2,3,10},
							{4,5,6,11},
							{7,8,9,12},
							{15,0,14,13}};

void SystemClock_Config()
{
    //TODO: Setup system clock and SysTick timer interrupt
	//use the clock from processor
	SysTick->CTRL |= 0x00000004;
	SysTick->LOAD = 400000;
	SysTick->CTRL |= 0x00000003;
}

void keypad_init()
{
	RCC->AHB2ENR   |= 0b00000000000000000000000000000110; //open port B and port C

	GPIOC->MODER   &= 0b11111111111111111111111100000000; //pc 3 2 1 0 as input of keypad
	GPIOC->MODER   |= 0b00000000000000000000000001010101;
	GPIOC->PUPDR   &= 0b11111111111111111111111100000000;
	GPIOC->PUPDR   |= 0b00000000000000000000000001010101;
	GPIOC->OSPEEDR &= 0b11111111111111111111111100000000;
	GPIOC->OSPEEDR |= 0b00000000000000000000000001010101;
	GPIOC->ODR     |= 0b00000000000000000000000000001111;

	GPIOB->MODER   &= 0b11111111111111110000000011111111; //pb 7 6 5 4 as output of keypad
	GPIOB->PUPDR   &= 0b11111111111111110000000011111111;
	GPIOB->PUPDR   |= 0b00000000000000001010101000000000;
}


char key_value = 0;
void EXIT_Setup()
{
	//TODO: Setup EXTI interrupt
}

void SystemClock_Config()
{
	//TODO: Setup system clock and SysTick timer interrupt
}

void SysTick_Handler(void) {
	//TODO: Scan the keypad column
}

void EXTIx_IRQHandler(void){
	//TODO: Read the keypad row value
}

int main(){
	SystemClock_Config();
	GPIO_init();
	max7219_init();
	keypad_init();
	EXTI_Setup();
	while(1){
		display(key_value,2);
	}
}
