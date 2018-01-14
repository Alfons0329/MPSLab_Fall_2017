#include "stm32l476xx.h"
#include <stdio.h>
//Use cable color to imply what the fucking color it represents
#define KEYPAD_ROW_MAX 4
#define KEYPAD_COL_MAX 4
#define SECOND_SLICE 99
#define CYC_COUNT_UP 39999
//Global and static data declaration
int duty_cycle_R = 50; // PA5 + AF1 which is corressponding to TIM2_CH1
int duty_cycle_G = 50; // PA1 + AF2 which is corressponding to TIM5_CH2
int duty_cycle_B = 50; // PA6 + AF2 which is corressponding to TIM3_CH1
int keypad_value[4][4] ={{0,1,2,3},
						 {4,5,6,7},
						 {8,9,10,11},
						 {12,13,14,15}};
/******************************Reference data is here*************************
//reference book p.1038 p.905
//ref: STM32 PWM
// https://read01.com/zh-tw/DGKMyB.html#.Wh2RU0qWY2w
// http://blog.csdn.net/akunainiannian/article/details/24316143
// http://www.zendei.com/article/12325.html
// preload register and shadow register
// https://read01.com/zh-tw/BgB8jG.html#.Wh6Qt0qWY2w
*****************************************************************************/
void keypad_init()//keypad along with GPIO Init together
{
	RCC->AHB2ENR   |= 0b00000000000000000000000000000111; //open port A,B,C

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

void Timer_init() //Use 3
{
	// PA5 + AF1 which is corressponding to TIM2_CH1
	// PA1 + AF2 which is corressponding to TIM5_CH2
	// PA6 + AF2 which is corressponding to TIM3_CH1
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
	//setting for timer 2
	TIM2->CR1 &= 0x0000; //p1027 Turned on the counter as the count up mode
	TIM2->CCMR1 |= (0b00 << TIM_CCMR1_CC1S_Pos); //channel 1 as the output
	TIM2->CCMR1 |= (0b110 << TIM_CCMR1_OC1M_Pos); //channel 1 as the PWM mode
	TIM2->CCER |= (0b1 << TIM_CCER_CC1E_Pos) //channel 1 as the compare output enable for PWM
	TIM2->ARR = (uint32_t)SECOND_SLICE;//Reload value
	TIM2->PSC = (uint32_t)CYC_COUNT_UP;//Prescaler
	// TIM2->EGR = TIM_EGR_UG;//Reinitialize the counter
	//setting for timer 3
	TIM3->CR1 &= 0x0000; //p1027 Turned on the counter as the count up mode
	TIM3->ARR = (uint32_t)SECOND_SLICE;//Reload value
	TIM3->PSC = (uint32_t)CYC_COUNT_UP;//Prescaler
	// TIM3->EGR = TIM_EGR_UG;//Reinitialize the counter
	//setting for timer 5
	TIM5->CR1 &= 0x0000; //p1027 Turned on the counter as the count up mode
	TIM5->ARR = (uint32_t)SECOND_SLICE;//Reload value
	TIM5->PSC = (uint32_t)CYC_COUNT_UP;//Prescaler
	// TIM5->EGR = TIM_EGR_UG;//Reinitialize the counter
}

void PWM_channel_init() //Use 3 timer but one channel for each to do
{
	// PA5 + AF1 which is corressponding to TIM2_CH1
	// PA1 + AF2 which is corressponding to TIM5_CH2
	// PA6 + AF2 which is corressponding to TIM3_CH1
	//setting for timer 2 channel 1
	TIM2->CR1 &= 0x0000; //disable the counter first
	TIM2->CCR1 = duty_cycle_R;
	TIM2->EGR |= (0b1 << TIM_EGR_UG_Pos); 	//update the counter again p1035
	TIM2->CR1 |= (0b1 << TIM_CR1_CEN_Pos);
	//setting for timer 3 channel 2
	//setting for timer 5 channel 1
}

void GPIO_init_AF() //GPIO Alternate Function Init
{
	// PA5 + AF1 which is corressponding to TIM2_CH1
	// PA1 + AF2 which is corressponding to TIM5_CH2
	// PA6 + AF2 which is corressponding to TIM3_CH1
				       //10987654321098765432109876543210
	GPIOA->MODER   	&= 0b11111111111111111100001111110011; //pc 3 2 1 0 as input of keypad
	GPIOA->MODER   	|= 0b00000000000000000001010000000100;
	//PortA Pin		   //10987654321098765432109876543210
	GPIOA->AFR[0]	=  0b00000010000100000000000000100000;
}
void keypad_scan() //Mapping the color changing logic
{

}
int main()
{
	keypad_init();
	GPIO_init_AF();
	Timer_init();
	PWM_channel_init();
}
