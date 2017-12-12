#include "stm32l476xx.h"
extern void delay_1s();
extern void GPIO_init();
unsigned int button_cnt;
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
    //PLLR[1:0]|PLLREN|23~16|15|PLLN[7:0]|7|PLLM[2:0]|3 2|PLLSRC[1:0]

    //MSI set as clock entry
    //654321098765432109876543210
    0b111000000000000100001110001,
    0b011000000000000110000110001,
    0b011000000000001010000110001,
    0b011000000000010000000110001,
    0b011000000000101000000110001
};

void systemclk_setting(int state)
{
    state %= 5; //state cycle
    //temporarily use ths hsi clock before turning off the pll clock for configuration since the system still need the clock to work
    RCC->CR |= RCC_CR_HSION; //turn on the hsi clock before configuraion
    while((RCC->CR & RCC_CR_HSIRDY) == 0); //wait till the hsi clock has been really turned on

    RCC->CFGR = 0x00000000; //CFGR reset value
    RCC->CR  &= 0xFEFFFFFF; //PLL off
    while (RCC->CR & 0x02000000); //busy waiting till PLL is really halted

    //after halted, configure the PLLCFGR to set the clock speed
    RCC->PLLCFGR &= 0x00000001; //off all except the MSI clock source
    RCC->PLLCFGR |= pll_cofig[state]; //customization PLLN PLLM PLLR settings

    RCC->CR |= RCC_CR_PLLON; //turn on the pll clock again
	while((RCC->CR & RCC_CR_PLLRDY) == 0); //busy waiting till PLL is really turned on

	RCC->CFGR |= RCC_CFGR_SW_PLL; //set the clock source as pll clock (customized)
    while ((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL); //wait till the pll clock is really set
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
    GPIO_init();
    static int state = 0;
    button_cnt = 0;
    systemclk_setting(0); //initil state
    while(1)
    {
        if(check_the_fucking_button())
        {
            state+=1;
            systemclk_setting(state);
        }

		GPIOA->ODR = 0b0000000000000000;
        if(check_the_fucking_button())
        {
            state+=1;
            systemclk_setting(state);
        }

		delay_1s();
        if(check_the_fucking_button())
        {
            state+=1;
            systemclk_setting(state);
        }

		GPIOA->ODR = 0b0000000000100000;
        if(check_the_fucking_button())
        {
            state+=1;
            systemclk_setting(state);
        }

		delay_1s();
        if(check_the_fucking_button())
        {
            state+=1;
            systemclk_setting(state);
        }

    }
    return 0;
}
