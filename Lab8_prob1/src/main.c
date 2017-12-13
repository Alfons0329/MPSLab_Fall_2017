#include <stdio.h>
#include <stdlib.h>
#include "ds18b20.h"
#include "onewire.h"
#include "ref.h"
extern void GPIOAC_init();
extern void max7219_send(unsigned char address, unsigned char data);
extern void max7219_init();
int mode, pre_temperature;
//configuration ref to: http://home.eeworld.com.cn/my/space-uid-116357-blogid-31714.html

void SystemClock_Config()
{
    //TODO: Setup system clock and SysTick timer interrupt
	//use the clock from processor
	SysTick->CTRL |= 0x00000004; //
	SysTick->LOAD = (uint32_t)7999999; //unsigned int 32 bit counter 8000000 (2s interrupt once)
	//system interrupt happens for every 8000000 cpu cycles, that is the peroid of 2 second
	SysTick->CTRL |= 0x00000007; //processor clock, turn on all
}
/*########################################################################
 * UNIT TEST LOG
 * INTERRUPT UNIT TEST PASS 2017/12/13, 19:23
 * READY FOR THERMAL TEST DEBUGGING
 * #######################################################################*/
void SysTick_Handler(void) // IF INTERRUPT HAPPENS, DO THIS TASK!
{
    //TODO: Show temperature on 7-seg display
	//DS18B20_Read(); //Interrupt happens, lets read the temperature from the one wire thermometer
	DS18B20_Read();
	//global_temperature++;
	display(global_temperature,2);
}
int check_the_fucking_button()
{
	static int debounce = 0;
	if( (GPIOC->IDR & 0b0010000000000000) == 0)
	{
	    debounce = debounce >= 1 ? 1 : debounce+1 ; //btn is pressed
	    return 0;
	}
	else if( debounce >= 1 )
	{
	    debounce = 0;
	    return 1;
	}
	return 0;
}
int display_clr(int num_digs)
{
	for(int i=1;i<=num_digs;i++)
	{
		max7219_send(i,0xF);
	}
	return 0;
}
int display(int data, int num_digs)
{
    //getting the value from LSB to MSB which is right to left
    //7 segpanel from 1 to 7 (not zero base)

    for(int i=1;i<=num_digs;i++)
    {
        max7219_send(i,data%10);
        data/=10; //get the next digit
    }
    if(data>99999999 || data<-9999999)
        return -1; //out of range error
    else
        return 0; //end this function
}
int main()
{
    SystemClock_Config();
    GPIOAC_init();
	max7219_init();
	mode=0;
	pre_temperature=0;
	cnt2=70;
	display_clr(8);
    while(1)
    {

    	if(check_the_fucking_button())
        {
			mode^=1; //mode exchange if button pressed
        }
		if(!mode)
		{

			pre_temperature=global_temperature; //update the pre_temperature
		}
		else
		{
			//display(pre_temperature,2); //show the old temperature
		}
    }
    pre_temperature+=0;
	return 0;
}
