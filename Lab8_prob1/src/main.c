#include <stdio.h>
#include <stdlib.h>
#include "ds18b20.h"
#include "onewire.h"
#include "ref.h"
extern void GPIOAC_init();
extern void max7219_send(unsigned char address, unsigned char data);
extern void max7219_init();
extern int global_temperature;
int mode;
void SystemClock_Config()
{
    //TODO: Setup system clock and SysTick timer interrupt

}
void SysTick_Handler(void)
{
    //TODO: Show temperature on 7-seg display

}
int check_the_fucking_button()
{
	static int debounce = 0;
	if( (GPIOC->IDR & 0b0010000000000000) == 0)
	{ // pressed
	    debounce = debounce >= 1 ? 1 : debounce+1 ;
	    return 0;
	}
	else if( debounce >= 1 )
	{
	    debounce = 0;
	    return 1;
	}
	return 0;
}
int main()
{
    SystemClock_Config();
    GPIOAC_init();
	mode=0;
    while(1)
    {
        if(check_the_fucking_button())
        {
			mode^=1; //mode exchange if button pressed
        }
		if(mode)
		{

		}
    }
}
