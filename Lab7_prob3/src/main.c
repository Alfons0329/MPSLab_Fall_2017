#include "stm32l4xx.h"
#include "stm32l4xx_nucleo.h"
#define DO 261.6
#define RE 293.7
#define MI 329.6
#define FA 349.2
#define SO 392.0
#define LA 440.0
#define SI 493.9
#define HDO 523.3

#define keypad_row_max 4
#define keypad_col_max 4
unsigned int keypad_value[4][4] ={{1,2,3,10},
                            	  {4,5,6,11},
								  {7,8,9,0},
								  {0,0,0,0}};

float freq = 0;
int curr = -2, prev = -3, check = -4;
int duty_cycle = 50;

extern void GPIO_init();

void keypad_init()
{
    GPIO_init(); //have initialized in arm
    RCC->AHB2ENR   |= 0b00000000000000000000000000000111; //safely initialize again

    GPIOC->MODER   &= 0b11111111111111111111111100000000; //use pb 3210 for Y input col
    GPIOC->MODER   |= 0b00000000000000000000000001010101; //use pb 3210 for Y input col
    GPIOC->PUPDR   &= 0b11111111111111111111111100000000; //clear and set output use pup since we want 1 to be sent high level voltage
    GPIOC->PUPDR   |= 0b00000000000000000000000001010101; //clear and set output use pup since we want 1 to be sent high level voltage
    GPIOC->OSPEEDR &= 0b11111111111111111111111100000000;
    GPIOC->OSPEEDR |= 0b00000000000000000000000001010101;
    GPIOC->ODR     |= 0b00000000000000000000000011110000;

    GPIOB->MODER   &= 0b11111111111111111111111100000000; //use pc 3210 for X output row
    GPIOB->PUPDR   &= 0b11111111111111111111111100000000; //clear and set input as pdown mode
    GPIOB->PUPDR   |= 0b00000000000000000000000010101010; //clear and set input as pdown mode
}


void GPIO_init_AF(){
	//TODO: Initial GPIO pin as alternate function for buzzer. You can choose to use C or assembly to finish this function.
		/* GPIO: set PB4 as alternate function */
		//RCC->AHB2ENR |= 0b1;	/* enable AHB2 clock for port A */
		GPIOA->MODER = (GPIOA->MODER & 0xFFFFF3FF)| 0b010000000000;
		GPIOA->AFR[0] = (GPIOA->AFR[0] & 0xFF0FFFFF) | 0x100000;	/* PA5: AF2 (TIM2_CH1) */ //AFRL
}
void Timer_init(){
    //TODO: Initialize timer
	//enable TIM2 timer clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	//downcounter
	TIM2->CR1 |= TIM_CR1_DIR;
	//enable auto-reload preload
	TIM2->CR1 |= TIM_CR1_ARPE;
	//reload value (auto-reload presaler value)
	TIM2->ARR = (uint32_t) 100;
	// re-initialize the counter and generates an update of the registers
	TIM2->EGR = TIM_EGR_UG;

	// select compare 2 (channel 2 is configured as output)
	TIM2->CCMR1 &= 0xFFFFFCFF;
	// set output compare 2 mode to PWM mode 1
	TIM2->CCMR1 |= (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
	TIM2->CCR1 = TIM2->ARR/2; //duty cycle=50%
	//TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
	// enable output compare 2 preload register on TIM2_CCR2
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE;
	// enable compare 2 output
	TIM2->CCER |= TIM_CCER_CC1E;

}


void PWM_channel_init(){
    //TODO: Initialize timer PWM channel
}

void timer_config()
{
	TIM2->PSC = (uint32_t) (4000000 / freq / 100);
	// prescalar value
	TIM2->CCR1 = duty_cycle;
	// compare 2 preload value
}

void keypad_scan()
{
    //if pressed , keypad return the value of that key, otherwise, return 255 for no pressed (unsigned char)
    int keypad_row=0,keypad_col=0;
    //char key_val=-1;
    while(1)
    {
        for(keypad_row=0;keypad_row<keypad_row_max;keypad_row++) //output data from 1st row
        {
            for(keypad_col=0;keypad_col<keypad_col_max;keypad_col++) //read input data from 1st col
            {
                /*use pc 3210 for X output row
                use pb 3210 for Y input col*/
            	GPIOC->ODR&=0; //clear the output value
                GPIOC->ODR|=(1<<keypad_row);//shift the value to send data for that row, data set
                int masked_value=GPIOB->IDR&0xf, is_pressed=(masked_value>>keypad_col)&1;
                if(is_pressed) //key is pressed
                {
                    check=keypad_value[keypad_row][keypad_col];
                    switch (check)
                    {
                    case 1:
                    	freq = DO;
                    	timer_config();
                    	TIM2->CR1 |= TIM_CR1_CEN; //start timer
                    	break;
                    case 2:
                    	freq = RE;
                    	timer_config();
                    	TIM2->CR1 |= TIM_CR1_CEN;
                    	break;
                    case 3:
                    	freq = MI;
                    	timer_config();
                    	TIM2->CR1 |= TIM_CR1_CEN;
                    	break;
                    case 4:
                    	freq = FA;
                   		timer_config();
                   		TIM2->CR1 |= TIM_CR1_CEN;
                    	break;
               		case 5:
                 		freq = SO;
                    	timer_config();
                    	TIM2->CR1 |= TIM_CR1_CEN;
                    	break;
                    case 6:
                    	freq = LA;
                    	timer_config();
                    	TIM2->CR1 |= TIM_CR1_CEN;
                    	break;
                    case 7:
                    	freq = SI;
                    	timer_config();
                    	TIM2->CR1 |= TIM_CR1_CEN;
                    	break;
                    case 8:
                    	freq = HDO;
                    	timer_config();
                    	TIM2->CR1 |= TIM_CR1_CEN;
                    	break;
                    case 10:
                    	duty_cycle = (duty_cycle == 90) ? duty_cycle : duty_cycle + 5;
                    	break;
                    case 11:
                    	duty_cycle = (duty_cycle == 10) ? duty_cycle : duty_cycle - 5;
                    	break;
                    default:
                    	TIM2->CR1 &= ~TIM_CR1_CEN;
                    	freq = -1;
                    	break;
                    }
                }
            }
        }
    }
}

int main(){
	GPIO_init();
	GPIO_init_AF();
	keypad_init();
	Timer_init();
	//PWM_channel_init();
    //TODO: Scan the keypad and use PWM to send the corresponding frequency square wave to buzzer.
	keypad_scan();
}
