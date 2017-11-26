#include "stm32l476xx.h"
extern void GPIO_init();
extern void delay_1s();
//reference to manual p225/1830
/*
f(PLLR) = f(PLL CLK INPUT)*(PLLN/(PLLM*PLLR))
SYS_CLK    PLLN    PLLM   PLLR   OUTPUT RESULT
1            8       8     4       4*8/32 = 1MHz
6            12      4     2       4*12/8 = 6MHz
10           20      4     2       4*20/8 = 10MHz
16           32      4     2       4*32/8 = 16MHz
40           80      4     2       4*80/8 = 40MHz
*/
unsigned int pll_cofig[5] =
{
    //15|PLLN[7:0]|7|PLLM[2:0]|3 2|PLLSRC[1:0]
    0b,
    0b,
    0b,
    0b,
}
void systemclk_setting(int state)
{
    state %= 5 //state cycle
    RCC->CFGR = 0x00000000; //CFGR reset value
    RCC->CR  &= 0xFEFFFFFF; //PLL off
    while (RCC->CR & 0x02000000); //wait till PLL is really halted
    //after halted, configure the PLLCFGR to set the clock speed



    RCC->CFGR = 0x00000003; //set PLL as the system clock
}
int main()
{

}
