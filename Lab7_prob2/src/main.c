#include "stm32l476xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define TIME_SEC 12.70
#define TARGET_SEC = TIME_SEC / 1
#define TARGET_MSEC = TIME_SEC * 100 - ( TARGET_SEC * 10 )
extern void GPIO_init();
extern void max7219_init();
extern void Display();
extern void max7219_send(unsigned char address, unsigned char data);
extern void max7219_init();
unsigned int millisecond;
unsigned int second;
unsigned int get_current_counter_value;
void Timer_init( TIM_TypeDef *timer)
{
    //todo: Initialize timer, dataset from the stm32l476xx.h
    RCC_APB1ENR1 |= RCC_APB1ENR1_TIM2EN; //turn on the timer2
    TIM2->CR1 &= 0x0000; //Turned on the counter as the count up mode
    TIM2->PSC = 39999U;  //prescaler, how many counter clock cycle I have to update my counter
    TIM2->ARR = 99U; //how much counter to do in one second, 0 is included so use 99 rather than 100
    //the formula is now on 4MHz = (39999+1)*(99+1) = 40000*100= 400w = 4MHz
    TIM2->EGR = 0x0001;  //re-initailzie timer to startup
    /*
    Bit 0 UG: Update generation
    This bit can be set by software, it is automatically cleared by hardware.
    0: No action
    1: Re-initialize the counter and generates an update of the registers. Note that the prescaler
    counter is cleared too (anyway the prescaler ratio is not affected). The counter is cleared if
    the center-aligned mode is selected or if DIR=0 (upcounting), else it takes the auto-reload
    value (TIMx_ARR) if DIR=1 (downcounting).
    */
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
void timer_display_reset(int data,int len)
{
    unsigned int tmp_ms = millisecond, tmp_s = second;
    if( data == 0 && len == 3 )
    {
        for(int i = 1;i <= len;i ++)
        {
            if(i == 3)
            {
                max7219_send(i,(data % 10) | (0x80)); //the float digit bit has to be turned on
            }
            else
            {
                max7219_send(i,data % 10);
            }
            data %= 10;
        }
    }
    else
    {
        for(int i = 1;i <= len;i ++)
        {
            if(i<3)
            {
                max7219_send(i,tmp_ms % 10);
                tmp_ms %= 10;
            }
            else if(i == 3)
            {
                max7219_send(i,(data % 10) | (0x80)); //the float digit bit has to be turned on
            }
            else
            {
                max7219_send(i,tmp_s % 10);
                tmp_s %= 10;
            }

        }
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
    millisecond = 0;
    get_current_counter_value = 0;
	while(1)
	{
		//todo: Polling the timer count and do lab requirements
        if(TIME_SEC < 0.01 || TIME_SEC > 10000.0)
        {
            timer_display(0,3); //3 bit float number, 0.00
        }
        get_current_counter_value = TIM2->CNT; //get the current counter value since it symbolizes the millisecond
        millisecond = get_current_counter_value;
        /*
        Bit 0 UIF: Update interrupt flag
        This bit is set by hardware on an update event. It is cleared by software.
        0: No update occurred
        1: Update interrupt pending. This bit is set by hardware when the registers are updated:
        At overflow or underflow (for TIM2 to TIM4) and if UDIS=0 in the TIMx_CR1 register.
        When CNT is reinitialized by software using the UG bit in TIMx_EGR register, if URS=0 and
        UDIS=0 in the TIMx_CR1 register.
        When CNT is reinitialized by a trigger event (refer to the synchro control register description),
        if URS=0 and UDIS=0 in the TIMx_CR1 register.
        */
        timer_display(6,display_length(second*100+millisecond));
        if(TIM2->SR & 0x0001) //one second is reached
        {
            second+=1;
            TIM2->SR &= ~(TIM_SR_UIF); //reset again for next counter event
        }

        get_current_counter_value = TIM2->CNT; //get the current counter value since it symbolizes the millisecond
        millisecond = get_current_counter_value;

        if( second >= TARGET_SEC && millisecond >= TARGET_MSEC)
        {
            timer_display(6,display_length(second*100+millisecond));
            while(1); //halt here
        }
        get_current_counter_value = TIM2->CNT; //get the current counter value since it symbolizes the millisecond
        millisecond = get_current_counter_value;
        timer_display(6,display_length(second*100+millisecond));
	}
    return 0;
}

/*typedef struct
//
{
  __IO uint32_t CR1;        !< TIM control register 1,                   Address offset: 0x00
  __IO uint32_t CR2;        !< TIM control register 2,                   Address offset: 0x04
  __IO uint32_t SMCR;       !< TIM slave mode control register,          Address offset: 0x08
  __IO uint32_t DIER;       !< TIM DMA/interrupt enable register,        Address offset: 0x0C
  __IO uint32_t SR;         !< TIM status register,                      Address offset: 0x10
  __IO uint32_t EGR;        !< TIM event generation register,            Address offset: 0x14
  __IO uint32_t CCMR1;      !< TIM capture/compare mode register 1,      Address offset: 0x18
  __IO uint32_t CCMR2;      !< TIM capture/compare mode register 2,      Address offset: 0x1C
  __IO uint32_t CCER;       !< TIM capture/compare enable register,      Address offset: 0x20
  __IO uint32_t CNT;        !< TIM counter register,                     Address offset: 0x24
  __IO uint32_t PSC;        !< TIM prescaler,                            Address offset: 0x28
  __IO uint32_t ARR;        !< TIM auto-reload register,                 Address offset: 0x2C
  __IO uint32_t RCR;        !< TIM repetition counter register,          Address offset: 0x30
  __IO uint32_t CCR1;       !< TIM capture/compare register 1,           Address offset: 0x34
  __IO uint32_t CCR2;       !< TIM capture/compare register 2,           Address offset: 0x38
  __IO uint32_t CCR3;       !< TIM capture/compare register 3,           Address offset: 0x3C
  __IO uint32_t CCR4;       !< TIM capture/compare register 4,           Address offset: 0x40
  __IO uint32_t BDTR;       !< TIM break and dead-time register,         Address offset: 0x44
  __IO uint32_t DCR;        !< TIM DMA control register,                 Address offset: 0x48
  __IO uint32_t DMAR;       !< TIM DMA address for full transfer,        Address offset: 0x4C
  __IO uint32_t OR1;        !< TIM option register 1,                    Address offset: 0x50
  __IO uint32_t CCMR3;      !< TIM capture/compare mode register 3,      Address offset: 0x54
  __IO uint32_t CCR5;       !< TIM capture/compare register5,            Address offset: 0x58
  __IO uint32_t CCR6;       !< TIM capture/compare register6,            Address offset: 0x5C
  __IO uint32_t OR2;        !< TIM option register 2,                    Address offset: 0x60
  __IO uint32_t OR3;        !< TIM option register 3,                    Address offset: 0x64
} TIM_TypeDef;

*/
