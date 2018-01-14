#include "stm32l476xx.h"
#include "mylib.h"
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

void GPIO_init_AF() //GPIO Alternate Function Init
{
	// PB3 + AF1 which is corressponding to TIM2_CH2 RED
	// PA1 + AF2 which is corressponding to TIM5_CH2 GREEN
	// PA6 + AF2 which is corressponding to TIM3_CH1 BLUE
					   //10987654321098765432109876543210
	GPIOA->MODER   	&= 0b11111111111111111100001111110011;
	GPIOA->MODER   	|= 0b00000000000000000001010000000100;
	//PortA Pin		   //10987654321098765432109876543210
	GPIOA->AFR[0]	=  0b00000010000100000000000000100000;

	//PB3 TIM2_CH2
	GPIOB->AFR[0] 	&= ~GPIO_AFRL_AFSEL3;//AFR[0] LOW
	GPIOB->AFR[0] 	|= (0b0001<<GPIO_AFRL_AFSEL3_Pos);//PB3 Alternate function mode
}

/*void Timer_init() //Use 3
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
	TIM2->CCER |= (0b1 << TIM_CCER_CC1E_Pos); //channel 1 as the compare output enable for PWM
	TIM2->ARR = (uint32_t)99;//Reload value
	TIM2->PSC = (uint32_t)39999;//Prescaler
	TIM2->EGR |= (0b1 << TIM_EGR_UG_Pos); 	//update the counter again p1035
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
}*/
void PWM_channel_init()
{
	/***********************setting for the TIM2_CH1 RED**************************/
	// PB3 + AF1 which is corressponding to TIM2_CH2 RED
	//Output compare 2 mode
	TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
	//110: PWM mode 1: TIMx_CNT<TIMx_CCR2-->active, or inactive
	TIM2->CCMR1 |= (0b0110 << TIM_CCMR1_OC2M_Pos);

	//Output Compare 2 Preload Enable
	TIM2->CCMR1 &= ~TIM_CCMR1_OC2PE;//OCxPE
	//1: enable TIMx_CCR1 Preload
	TIM2->CCMR1 |= (0b1 << TIM_CCMR1_OC2PE_Pos);
	//enable auto reload pre-load
	TIM2->CR1 |= TIM_CR1_ARPE;

	//duty cycle initial 50 (CCR2/ARR)
	TIM2->CCR2 = duty_cycle;
	//enable output compare
	TIM2->CCER |= TIM_CCER_CC2E;

	/***********************setting for the TIM5_CH2 GREEN**************************/
	// PA1 + AF2 which is corressponding to TIM5_CH2 GREEN
	//Output compare 2 mode
	TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
	//110: PWM mode 1: TIMx_CNT<TIMx_CCR2-->active, or inactive
	TIM2->CCMR1 |= (0b0110 << TIM_CCMR1_OC2M_Pos);

	//Output Compare 2 Preload Enable
	TIM2->CCMR1 &= ~TIM_CCMR1_OC2PE;//OCxPE
	//1: enable TIMx_CCR1 Preload
	TIM2->CCMR1 |= (0b1 << TIM_CCMR1_OC2PE_Pos);
	//enable auto reload pre-load
	TIM2->CR1 |= TIM_CR1_ARPE;

	//duty cycle initial 50 (CCR2/ARR)
	TIM2->CCR2 = duty_cycle;
	//enable output compare
	TIM2->CCER |= TIM_CCER_CC2E;

	/***********************setting for the TIM3_CH1 BLUE**************************/
	// PA6 + AF2 which is corressponding to TIM3_CH1 BLUE
	//Output compare 2 mode
	TIM2->CCMR1 &= ~TIM_CCMR1_OC2M;
	//110: PWM mode 1: TIMx_CNT<TIMx_CCR2-->active, or inactive
	TIM2->CCMR1 |= (0b0110 << TIM_CCMR1_OC2M_Pos);

	//Output Compare 2 Preload Enable
	TIM2->CCMR1 &= ~TIM_CCMR1_OC2PE;//OCxPE
	//1: enable TIMx_CCR1 Preload
	TIM2->CCMR1 |= (0b1 << TIM_CCMR1_OC2PE_Pos);
	//enable auto reload pre-load
	TIM2->CR1 |= TIM_CR1_ARPE;

	//duty cycle initial 50 (CCR2/ARR)
	TIM2->CCR2 = duty_cycle;
	//enable output compare
	TIM2->CCER |= TIM_CCER_CC2E;

}

void set_timer()
{
	int prescaler = (4000000 / freq / 100);
	//TIM2->PSC = (uint32_t) prescaler;
	// prescaler value
	TIM2->CCR2 = duty_cycle;
	// compare 2 preload value
}

void keypad_scan() //Mapping the color changing logic
{

}
int main()
{
	//use the time delay mode to make the interleaving and the color changing scheme
	keypad_init();
	GPIO_init_AF();
	Timer_init();
	while(1)
	{
		PWM_channel_init();
	}
	return 0;
}
