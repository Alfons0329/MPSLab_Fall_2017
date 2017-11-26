/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32l4xx.h"
#include "stm32l4xx_nucleo.h"

extern void GPIO_init();
void GPIO_init_AF(){
//TODO: Initial GPIO pin as alternate function for buzzer. You can choose to use C or assembly to finish this function.
}
void Timer_init(){
    //TODO: Initialize timer
}
void PWM_channel_init(){
    //TODO: Initialize timer PWM channel
}
int main(){
	GPIO_init();
   GPIO_init_AF();
	Timer_init();
   PWM_channel_init();
    //TODO: Scan the keypad and use PWM to send the corresponding frequency square wave to buzzer.
}
