#include "stm32l476xx.h"
//#define TIME_SEC 12.70
extern void GPIO_init();
extern void max7219_init();
extern void Display();
extern void max7219_send(unsigned char address, unsigned char data);
extern void max7219_init();
void systemclk_setting() //initialize the system clock before doing timing, USE 1 MHZ!!!!!!!!, the code is used from Lab7_prob1
{
    //temporarily use ths hsi clock before turning off the pll clock for configuration since the system still need the clock to work
    RCC->CR |= RCC_CR_HSION; //turn on the hsi clock before configuraion
    while((RCC->CR & RCC_CR_HSIRDY) == 0); //wait till the hsi clock has been really turned on

    RCC->CFGR = 0x00000000; //CFGR reset value
    RCC->CR  &= 0xFEFFFFFF; //PLL off
    while (RCC->CR & 0x02000000); //busy waiting till PLL is really halted

    //after halted, configure the PLLCFGR to set the clock speed
    RCC->PLLCFGR &= 0x00000001; //off all except the MSI clock source
    RCC->PLLCFGR |= 0b011000000000000100001110001; //customization PLLN PLLM PLLR settings

    RCC->CR |= RCC_CR_PLLON; //turn on the pll clock again
	while((RCC->CR & RCC_CR_PLLRDY) == 0); //busy waiting till PLL is really turned on

	RCC->CFGR |= RCC_CFGR_SW_PLL; //set the clock source as pll clock (customized)
    while ((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL); //wait till the pll clock is really set
}
void Timer_init( TIM_TypeDef *timer)
{
    //todo: Initialize timer, dataset from the stm32l476xx.h
    RCC_APB1ENR1 |= RCC_APB1ENR1_TIM2EN; //turn on the timer2
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->PSC = 1048576*(/) //prescaler auto reload value after exceed this
    TIM2->EGR = 
}
void Timer_start(TIM_TypeDef *timer)
{
    //todo: start timer and show the time on the 7-SEG LED.
}
void display_clr()
{
    for(int i=1;i<=8;i++)
    {
        max7219_send(i,0xF);
    }
    return 0;
}
int main()
{
	GPIO_init();
	max7219_init();
	Timer_init();
	Timer_start();
	while(1)
	{
		//todo: Polling the timer count and do lab requirements
	}
}
