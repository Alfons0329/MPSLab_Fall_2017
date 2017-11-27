#include "stm32l476xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define TIME_SEC 12.70
#define UPPER_BOUND = TIME_SEC*100
extern void GPIO_init();
extern void max7219_init();
extern void Display();
extern void max7219_send(unsigned char address, unsigned char data);
extern void max7219_init();
unsigned int millisecond;
void Timer_init( TIM_TypeDef *timer)
{
    //todo: Initialize timer, dataset from the stm32l476xx.h
    RCC_APB1ENR1 |= RCC_APB1ENR1_TIM2EN; //turn on the timer2
    TIM2->CR1 &= 0x0000; //Turned on the counter as the count up mode
    TIM2->PSC = 39999U;  //prescaler, how many counter clock cycle I have to update my counter
    TIM2->ARR = 99U; //how much counter to do in one second, 0 is included so use 99 rather than 100
    //the formula is now on 4MHz = (39999+1)*(99+1) = 40000*100= 400w = 4MHz
    TIM2->EGR = TIM_EGR_UG;  //re-initailzie timer to startup
    //counter will be incremented by one symbolize the millisecond
    //1 millisecond is 40000/4000000 = 1/100, so each 40000 clock cycle, increse the counter by one
    //the time precision should be 0.01sec, so set the arr be 99 which means a second should be divieded into 100 parts
}
void Timer_start(TIM_TypeDef *timer)
{
    //todo: start timer and show the time on the 7-SEG LED.
    //enable the counter
    TIM2->CR1 |= TIM_CR1_CEN //Turn on the counter mode, change in the control register
    TIM2->SR1 &= ~(TIM_SR_UIF) //off the user interrupt mode, so the cpu can keep working on the clock increment
}
int display_length()
{
    return (int) log10(millisecond);
}
void timer_display(int data,int len)
{
    for(int i = 1;i <= len;i ++)
    {
        if(i == 3)
        {
            max7219_send(i,(data%10) | (0x80)); //the float digit bit has to be turned on
        }
        else
        {
            max7219_send(i,data%10);
        }
        data %= 10;
    }

}
void display_clr()
{
    for(int i = 1;i <= 8;i ++)
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
        if(TIME_SEC < 0.01 || TIME_SEC > 10000.0)
        {
            timer_display(0,3); //3 bit float number, 0.00
        }
        if(TIM2->)
	}

}
