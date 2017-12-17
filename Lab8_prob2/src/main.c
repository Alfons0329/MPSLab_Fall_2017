#include "stm32l476xx.h"
#include "core_cm4.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

int scan_col = 0;
char key_value = 0;

extern void GPIO_init();
extern void max7219_send(unsigned char address, unsigned char data);
extern void max7219_init();

void display(int data, int num_digs)
{
	int tmp;
	for(int i = 1; i <= num_digs; i++){
		tmp =  data % 10;
		data /= 10;
		max7219_send(i, tmp);
		if(data == 0){
			num_digs = i;
			break;
		}
	}
	for(int i = 8; i > num_digs; i--)
		max7219_send(i, 0xF);
}

void keypad_init()
{
	// SET keypad gpio OUTPUT //
	// open clk
	RCC->AHB2ENR |= 0b110;

	//Set PB5,6,7,8 as output mode
	GPIOB->MODER   &= 0b11111111111111000000001111111111; //pc 3 2 1 0 as input of keypad
	GPIOB->MODER   |= 0b00000000000000010101010000000000;
	GPIOB->PUPDR   &= 0b11111111111111000000001111111111;
	GPIOB->PUPDR   |= 0b00000000000000010101010000000000;
	GPIOB->OSPEEDR &= 0b11111111111111000000001111111111;
	GPIOB->OSPEEDR |= 0b00000000000000010101010000000000;

	// SET keypad gpio INPUT //
	//Set PC0,1,2,3 as INPUT mode
	GPIOC->MODER   &= 0b11111111111111111111111100000000;
	GPIOC->PUPDR   &= 0b11111111111111111111111100000000;
	GPIOC->PUPDR   |= 0b00000000000000000000000010101010;
	GPIOC->OSPEEDR &= 0b11111111111111111111111100000000;
	GPIOC->OSPEEDR |= 0b00000000000000000000000001010101;
}

void SystemClock_Config()
{
	//http://jingyan.eeboard.com/article/36792
	//TODO: Setup system clock and SysTick timer interrupt
	RCC->APB2ENR |= 1;
	SysTick->LOAD  = 400000; //0.1s
	SysTick->VAL   = 0; //Load the SysTick Counter Value
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void EXTI_Setup(){
	//select source for EXTI0.1.2.3
	SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI0_PC | SYSCFG_EXTICR1_EXTI1_PC | SYSCFG_EXTICR1_EXTI2_PC | SYSCFG_EXTICR1_EXTI3_PC);
	// setup EXTI
	//select falling edge 0.1.2.3
	EXTI->IMR1 |= (EXTI_IMR1_IM3 | EXTI_IMR1_IM2 | EXTI_IMR1_IM1 | EXTI_IMR1_IM0);
	//enable interrupt 0.1.2.3
	EXTI->FTSR1 |= (EXTI_FTSR1_FT3 | EXTI_FTSR1_FT2 | EXTI_FTSR1_FT1 | EXTI_FTSR1_FT0);
	//clear
	EXTI->PR1 |= (EXTI_PR1_PIF3 | EXTI_PR1_PIF2 | EXTI_PR1_PIF1 | EXTI_PR1_PIF0);

	//enable interrupt mask
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);
	//clear pending flag
	NVIC_ClearPendingIRQ(EXTI0_IRQn);
	NVIC_ClearPendingIRQ(EXTI1_IRQn);
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
	NVIC_ClearPendingIRQ(EXTI3_IRQn);
	//set priority
	NVIC_SetPriority(EXTI0_IRQn,0);
	NVIC_SetPriority(EXTI1_IRQn,0);
	NVIC_SetPriority(EXTI2_IRQn,0);
	NVIC_SetPriority(EXTI3_IRQn,0);
	//set systick priority level
	//SCB->SHP[(((uint32_t)(int32_t)-1) & 0xFUL)-4UL] = (uint8_t)((1 << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
	SCB->SHP[11] = 0xFF;
}

void SysTick_Handler(void) {
	GPIOB->BRR = (0b1111 << 5);
	GPIOB->BSRR = (1 << (((scan_col+1)%4)+5));
	scan_col = (scan_col+1)%4;
}

void EXTI0_IRQHandler(void){
	if(scan_col == 0)
		key_value = 1;
	else if(scan_col == 1)
		key_value = 2;
	else if(scan_col == 2)
		key_value = 3;
	else if(scan_col == 3)
		key_value = 10;
	//clear pending flag
	NVIC_ClearPendingIRQ(EXTI0_IRQn);
	NVIC_ClearPendingIRQ(EXTI1_IRQn);
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
	NVIC_ClearPendingIRQ(EXTI3_IRQn);
	//clear exti
	EXTI->PR1 |= (EXTI_PR1_PIF3 | EXTI_PR1_PIF2 | EXTI_PR1_PIF1 | EXTI_PR1_PIF0);
}

void EXTI1_IRQHandler(void){
	if(scan_col == 0)
		key_value = 4;
	else if(scan_col == 1)
		key_value = 5;
	else if(scan_col == 2)
		key_value = 6;
	else if(scan_col == 3)
		key_value = 11;
	//clear pending flag
	NVIC_ClearPendingIRQ(EXTI0_IRQn);
	NVIC_ClearPendingIRQ(EXTI1_IRQn);
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
	NVIC_ClearPendingIRQ(EXTI3_IRQn);
	//clear exti
	EXTI->PR1 |= (EXTI_PR1_PIF3 | EXTI_PR1_PIF2 | EXTI_PR1_PIF1 | EXTI_PR1_PIF0);
}

void EXTI2_IRQHandler(void){
	if(scan_col == 0)
		key_value = 7;
	else if(scan_col == 1)
		key_value = 8;
	else if(scan_col == 2)
		key_value = 9;
	else if(scan_col == 3)
		key_value = 12;
	//clear pending flag
	NVIC_ClearPendingIRQ(EXTI0_IRQn);
	NVIC_ClearPendingIRQ(EXTI1_IRQn);
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
	NVIC_ClearPendingIRQ(EXTI3_IRQn);
	//clear exti
	EXTI->PR1 |= (EXTI_PR1_PIF3 | EXTI_PR1_PIF2 | EXTI_PR1_PIF1 | EXTI_PR1_PIF0);
}

void EXTI3_IRQHandler(void){
	if(scan_col == 0)
		key_value = 15;
	else if(scan_col == 1)
		key_value = 0;
	else if(scan_col == 2)
		key_value = 14;
	else if(scan_col == 3)
		key_value = 13;
	//clear pending flag
	NVIC_ClearPendingIRQ(EXTI0_IRQn);
	NVIC_ClearPendingIRQ(EXTI1_IRQn);
	NVIC_ClearPendingIRQ(EXTI2_IRQn);
	NVIC_ClearPendingIRQ(EXTI3_IRQn);
	//clear exti
	EXTI->PR1 |= (EXTI_PR1_PIF3 | EXTI_PR1_PIF2 | EXTI_PR1_PIF1 | EXTI_PR1_PIF0);
}

int main(){
	GPIO_init();
	max7219_init();
	keypad_init();
	SystemClock_Config();
	EXTI_Setup();
	while(1){
		display(key_value, 2);
	}
}
