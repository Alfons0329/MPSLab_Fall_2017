#include <stdio.h>
#include <stdlib.h>
extern void GPIO_init();
extern void max7219_send(unsigned char address, unsigned char data);
extern void max7219_init();
void SystemClock_Config()
{
//TODO: Setup system clock and SysTick timer interrupt
}
void SysTick_Handler(void)
{
//TODO: Show temperature on 7-seg display
}
int main()
{
    SystemClock_Config();
    GPIO_init();
    while(1)
    {
        if(user_press_button())
        {
            //TODO: Enable or disable Systick timer
        }
    }
}
