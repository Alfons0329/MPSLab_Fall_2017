#include "stm32l476xx.h"

//freq
#define DO 261.6
#define RE 293.7
#define MI 329.6
#define FA 349.2
#define SO 392.0
#define LA 440.0
#define SI 493.9
#define HDO 523.3

//keypad
#define keypad_row_max 4
#define keypad_col_max 4
int keypad_value[4][4] ={	{1,2,3,0},
							{4,5,6,0},
							{7,8,9,0},
							{14,0,15,0}};
int freq = -1;
int duty_cycle_R = 50; //TIM2_CH1 PA5
int duty_cycle_G = 50; //TIM2_CH2 PB3
int duty_cycle_B = 50; //TIM2_CH3 PA2

//extern void GPIO_init();
void keypad_init()
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



void GPIO_init_AF(){
//TODO: Initial GPIO pin as alternate function for buzzer. You can choose to use C or assembly to finish this function.
	//PB3 TIM2_CH2
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL3;//AFR[0] LOW
	GPIOB->AFR[0] |= (0b0001<<GPIO_AFRL_AFSEL3_Pos);//PB3 Alternate function mode
	//PA5 TIM2_CH1
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL5;//AFR[0] LOW
	GPIOA->AFR[0] |= (0b0001<<GPIO_AFRL_AFSEL5_Pos);//PA5
	//PA2 TIM2_CH3
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL2;//AFR[0] LOW
	GPIOA->AFR[0] |= (0b0001<<GPIO_AFRL_AFSEL2_Pos);//PA2
}
void Timer_init(){
   //TODO: Initialize timer
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	//TIM2->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
	//TIM2->CR1 &= ~(TIM_CR1_DIR_Pos);
	//TIM2->CR1 &= ~(TIM_CR1_CMS_Pos);
	TIM2->CR1 &= 0x0000; //Turned on the counter as the count up mode
	//SET_REG(TIM2->CR1, TIM_CR1_DIR | TIM_CR1_CMS, TIM_COUNTERMODE_DOWN);// Edge-aligned mode, down counter
	TIM2->ARR = (uint32_t)100;//Reload value
	TIM2->PSC = (uint32_t)39999;//Prescaler
	TIM2->EGR = TIM_EGR_UG;//Reinitialize the counter
	//TIM2->CR1 |= TIM_CR1_CEN;
}

void PWM_channel_init(){
   //TODO: Initialize timer PWM channel
	//ref: STM32 PWM
	// https://read01.com/zh-tw/DGKMyB.html#.Wh2RU0qWY2w
	// http://blog.csdn.net/akunainiannian/article/details/24316143
	// http://www.zendei.com/article/12325.html
	// preload register and shadow register
	// https://read01.com/zh-tw/BgB8jG.html#.Wh6Qt0qWY2w
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

	//TIM2_CH2: duty cycle initial 50 (CCR2/ARR)
	TIM2->CCR2 = 50;
	//enable output compare
	TIM2->CCER |= TIM_CCER_CC2E;

	//TIM2_CH1: duty cycle initial 50 (CCR1/ARR)
	TIM2->CCR1 = 50;
	//enable output compare
	TIM2->CCER |= TIM_CCER_CC1E;

	//TIM2_CH3: duty cycle initial 50 (CCR3/ARR)
	TIM2->CCR3 = 50;
	//enable output compare
	TIM2->CCER |= TIM_CCER_CC3E;
}

void set_timer()
{
	int prescaler = (4000000 / freq / 100);
	TIM2->PSC = (uint32_t) prescaler;
	//TIM2_CH1: R
	TIM2->CCR1 = duty_cycle_R;
	//TIM2_CH2: G
	TIM2->CCR2 = duty_cycle_G;
	//TIM2_CH3: B
	TIM2->CCR3 = duty_cycle_B;
}

/*int keypad_scan()
{
    //if pressed , keypad return the value of that key, otherwise, return 255 for no pressed (unsigned char)
    int keypad_row=0,keypad_col=0;
    int key_val=-1;
    int curr = -1, prev = -2, check = -1;
    while(1){
    	key_val=0;
    	for(keypad_row=0;keypad_row<keypad_row_max;keypad_row++) //output data from 1st row
        {
    		for(keypad_col=0;keypad_col<keypad_col_max;keypad_col++) //read input data from 1st col
            {
    			//use pc 3210 for X output row
                //use pb 3210 for Y input col
                GPIOC->ODR&=0; //clear the output value
                GPIOC->ODR|=(1<<keypad_row);//shift the value to send data for that row, data set
                int masked_value=GPIOB->IDR&0xf0, is_pressed=(masked_value>>(keypad_col+4))&1;
                if(is_pressed) //key is pressed
                {
                    key_val=keypad_value[keypad_row][keypad_col];
                }

            }
        }
    	prev = curr;
    	curr = key_val;
    	// ring while keep press same button
    	if (curr == prev)
    		check = 100;
    	else
    		check = curr;
  		switch (check)
    	{
    		case 1:
    			freq = DO;
    			set_timer();
    			TIM2->CR1 |= TIM_CR1_CEN;
    			break;
    		case 2:
    			freq = RE;
    			set_timer();
    			TIM2->CR1 |= TIM_CR1_CEN;
    			break;
    		case 3:
    			freq = MI;
    			set_timer();
    			TIM2->CR1 |= TIM_CR1_CEN;
    			break;
    		case 4:
    			freq = FA;
    			set_timer();
    			TIM2->CR1 |= TIM_CR1_CEN;
    			break;
    		case 5:
    			freq = SO;
    			set_timer();
    			TIM2->CR1 |= TIM_CR1_CEN;
    			break;
    		case 6:
    			freq = LA;
    			set_timer();
    			TIM2->CR1 |= TIM_CR1_CEN;
    			break;
    		case 7:
    			freq = SI;
    			set_timer();
    			TIM2->CR1 |= TIM_CR1_CEN;
    			break;
    		case 8:
    			freq = HDO;
    			set_timer();
    			TIM2->CR1 |= TIM_CR1_CEN;
    			break;
    		case 14: //duty cycle -5
    			duty_cycle = duty_cycle == 10 ? duty_cycle : duty_cycle - 5;
    			break;
    		case 15: //duty cycle +5
    			duty_cycle = duty_cycle == 90 ? duty_cycle : duty_cycle + 5;
    			break;
    		case 100: //empty loop
    			break;
    		default: //stop timer
    			TIM2->CR1 &= ~TIM_CR1_CEN;
    			freq = -1;
    			break;
    	}
    }
    return key_val;
}*/
void PWM()
{
	freq = SO;
	duty_cycle_R = 50;
	duty_cycle_G = 50;
	duty_cycle_B = 50;
	while(1){
		set_timer();
		TIM2->CR1 |= TIM_CR1_CEN;
	}
	/*while(1){
		for(int i=0; i<100; ++i){
			duty_cycle = i;
			set_timer();
			int j=10000000;
			while(j!=0){
				TIM2->CR1 |= TIM_CR1_CEN;
				--j;
			}
			//TIM2->CR1 &= ~TIM_CR1_CEN;
		}
	}*/
}

int main()
{
	keypad_init();
	GPIO_init_AF();
	Timer_init();
	PWM_channel_init();
	PWM();
	//keypad_scan();
}

