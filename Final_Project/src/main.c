#include "stm32l476xx.h"
#include "mylib.h"
#include "ds18b20.h"
#include "onewire.h"
#include <stdio.h>
//Use cable color to imply what the fucking color it represents
#define KEYPAD_ROW_MAX 4
#define KEYPAD_COL_MAX 4

#define SECOND_SLICE 255
#define COUNT_UP 255

#define DELTA_VALUE 30
#define RED_START 10
#define GREEN_START 91
#define BLUE_START 172

#define CYCLE_MODE 0
#define CONTROL_MODE 1
#define TEMP_MODE 2
#define SPEED_LIMIT 30000
#define SLOWEST_SPEED 240000
//Global and static data declaration
int cur_state = 0; //default state 0 for color changing and 1 for self-control color scheme
int duty_cycle_R = 50; // PB3 + AF1 which is corresponding to TIM2_CH1 REG
int duty_cycle_G = 50; // PA1 + AF2 which is corresponding to TIM5_CH2 GREEN
int duty_cycle_B = 50; // PA6 + AF2 which is corresponding to TIM3_CH1 BLUE
int keypad_value[4][4] = {{0,1,2,3},
						  {4,5,6,7},
						  {8,9,10,11},
						  {12,13,14,15}};
//FSM data structure is here
int prev=-1, curr=-1, check;
int on = 0;
//remember off state
int off_state, off_R, off_G, off_B;
//cycle mode control
int state_R = 1;
int state_G = 1;
int state_B = 1;
//control cycle speed
int speed = 150000;
/******************************Reference data is here*************************
 * reference book p.1038 p.905
 * ref: STM32 PWM
 * https://read01.com/zh-tw/DGKMyB.html#.Wh2RU0qWY2w
 * http://blog.csdn.net/akunainiannian/article/details/24316143
 * http://www.zendei.com/article/12325.html
 * preload register and shadow register
 * https://read01.com/zh-tw/BgB8jG.html#.Wh6Qt0qWY2w
*****************************************************************************/
void SystemClock_Config()
{
    //TODO: Setup system clock and SysTick timer interrupt
	//use the clock from processor
	SysTick->CTRL |= 0x00000004; //
	SysTick->LOAD = (uint32_t)7999999; //unsigned int 32 bit counter 8000000 (2s interrupt once)
	//system interrupt happens for every 8000000 cpu cycles, that is the peroid of 2 second
	SysTick->CTRL |= 0x00000007; //processor clock, turn on all
}
void SysTick_Handler(void) // IF INTERRUPT HAPPENS, DO THIS TASK!
{
    //TODO: Show temperature on 7-seg display
	DS18B20_Read();
}
void keypad_init()//keypad along with GPIO Init together
{

	RCC->AHB2ENR   |= 0b00000000000000000000000000000111; //open port A,B,C
				      //10987654321098765432109876543210
	GPIOC->MODER   &= 0b11111111111111111111111100000000; //pc 3 2 1 0 as input of keypad
	GPIOC->MODER   |= 0b00000000000000000000000001010101;
	GPIOC->PUPDR   &= 0b11111111111111111111111100000000;
	GPIOC->PUPDR   |= 0b00000000000000000000000001010101;
	GPIOC->OSPEEDR &= 0b11111111111111111111111100000000;
	GPIOC->OSPEEDR |= 0b00000000000000000000000001010101;
	GPIOC->ODR     |= 0b00000000000000000000000000001111;
	                  //10987654321098765432109876543210
	GPIOB->MODER   &= 0b11111111111111110000000011111111; //pb 7 6 5 4 as output of keypad
	GPIOB->PUPDR   &= 0b11111111111111110000000011111111;
	GPIOB->PUPDR   |= 0b00000000000000001010101000000000;

}

void GPIO_init_AF() //GPIO Alternate Function Init
{
	/***************pin and alternate function***************
	 * PB3 + AF1 which is corresponding to TIM2_CH2 RED
	 * PA1 + AF2 which is corresponding to TIM5_CH2 GREEN
	 * PA6 + AF2 which is corresponding to TIM3_CH1 BLUE
	 ********************************************************/
					   //10987654321098765432109876543210
	GPIOA->MODER   	&= 0b11111111111111111100001111110011;
	GPIOA->MODER   	|= 0b00000000000000000010100000001000;
	//PortA Pin		   //10987654321098765432109876543210
	GPIOA->AFR[0]	=  0b00000010000100000000000000100000;

	//PB3 TIM2_CH2
	GPIOB->AFR[0] 	&= ~GPIO_AFRL_AFSEL3;//AFR[0] LOW
	GPIOB->AFR[0] 	|= (0b0001<<GPIO_AFRL_AFSEL3_Pos);//PB3 Alternate function mode
}

void Timer_init() //Use 3
{
	// PA3 + AF1 which is corresponding to TIM2_CH1
	// PA1 + AF2 which is corresponding to TIM5_CH2
	// PA6 + AF2 which is corresponding to TIM3_CH1
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;

	//setting for timer 2
	TIM2->CR1 &= 0x0000; //p1027 Turned on the counter as the count up mode
	TIM2->ARR = (uint32_t)SECOND_SLICE;//Reload value
	TIM2->PSC = (uint32_t)COUNT_UP;//Prescaler
	TIM2->EGR = TIM_EGR_UG; 	//update the counter again p1035

	//setting for timer 3
	TIM3->CR1 &= 0x0000; //p1027 Turned on the counter as the count up mode
	TIM3->ARR = (uint32_t)SECOND_SLICE;//Reload value
	TIM3->PSC = (uint32_t)COUNT_UP;//Prescaler
	TIM3->EGR = TIM_EGR_UG;//Reinitialize the counter

	//setting for timer 5
	TIM5->CR1 &= 0x0000; //p1027 Turned on the counter as the count up mode
	TIM5->ARR = (uint32_t)SECOND_SLICE;//Reload value
	TIM5->PSC = (uint32_t)COUNT_UP;//Prescaler
	TIM5->EGR = TIM_EGR_UG;//Reinitialize the counter
}

void PWM_channel_init()
{
	/***********************setting for the TIM2_CH2 RED**************************/
	// PB3 + AF1 which is corresponding to TIM2_CH2 RED
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

	//duty cycle initial 50 (CCR2/ARR)
	//TIM2->CCR2 = duty_cycle_R;
	//enable output compare
	TIM2->CCER |= TIM_CCER_CC2E;

	/***********************setting for the TIM5_CH2 GREEN**************************/
	// PA1 + AF2 which is corresponding to TIM5_CH2 GREEN
	//Output compare 2 mode
	TIM5->CCMR1 &= ~TIM_CCMR1_OC2M;
	//110: PWM mode 1: TIMx_CNT<TIMx_CCR2-->active, or inactive
	TIM5->CCMR1 |= (0b0110 << TIM_CCMR1_OC2M_Pos);

	//Output Compare 2 Preload Enable
	TIM5->CCMR1 &= ~TIM_CCMR1_OC2PE;//OCxPE
	//1: enable TIMx_CCR1 Preload
	TIM5->CCMR1 |= (0b1 << TIM_CCMR1_OC2PE_Pos);
	//enable auto reload pre-load
	TIM5->CR1 |= TIM_CR1_ARPE;

	//duty cycle initial 50 (CCR2/ARR)
	//TIM5->CCR2 = duty_cycle_G;
	//enable output compare
	TIM5->CCER |= TIM_CCER_CC2E;

	/***********************setting for the TIM3_CH1 BLUE**************************/
	// PA6 + AF2 which is corresponding to TIM3_CH1 BLUE
	//Output compare 2 mode
	TIM3->CCMR1 &= ~TIM_CCMR1_OC1M;
	//110: PWM mode 1: TIMx_CNT<TIMx_CCR2-->active, or inactive
	TIM3->CCMR1 |= (0b0110 << TIM_CCMR1_OC1M_Pos);

	//Output Compare 2 Preload Enable
	TIM3->CCMR1 &= ~TIM_CCMR1_OC1PE;//OCxPE
	//1: enable TIMx_CCR1 Preload
	TIM3->CCMR1 |= (0b1 << TIM_CCMR1_OC1PE_Pos);
	//enable auto reload pre-load
	TIM3->CR1 |= TIM_CR1_ARPE;

	//duty cycle initial 50 (CCR2/ARR)
	//TIM3->CCR1 = duty_cycle_B;
	//enable output compare
	TIM3->CCER |= TIM_CCER_CC1E;

}

void set_timer()
{
	// int prescaler = (4000000 / freq / 100);
	//TIM2->PSC = (uint32_t) prescaler;
	//TIM2_CH2
	//prescaler value
	TIM2->CCR2 = duty_cycle_R; // compare 2 preload value

	//TIM5_CH2
	//prescaler value
	TIM5->CCR2 = duty_cycle_G; // compare 2 preload value

	//TIM3_CH1
	//prescaler value
	TIM3->CCR1 = duty_cycle_B; // compare 2 preload value
}

void stop_timer()
{
	TIM2->CR1 &= ~TIM_CR1_CEN;
	TIM5->CR1 &= ~TIM_CR1_CEN;
	TIM3->CR1 &= ~TIM_CR1_CEN;
}

void start_timer()
{
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM5->CR1 |= TIM_CR1_CEN;
	TIM3->CR1 |= TIM_CR1_CEN;
}

void cycle_mode(int delay_time){
	PWM_channel_init();
	if (state_R){
		if (duty_cycle_R > SECOND_SLICE){
			state_R = 0;
		} else {
			duty_cycle_R += 20;
		}
	} else {
		if (duty_cycle_R < 20){
			state_R = 1;
		} else {
			duty_cycle_R -= 20;
		}
	}

	if (state_G){
		if (duty_cycle_G > SECOND_SLICE){
			state_G = 0;
	} else {
		duty_cycle_G += 40;
		}
	} else {
		if (duty_cycle_G < 40){
			state_G = 1;
		} else {
			duty_cycle_G -= 40;
		}
	}

	if (state_B){
		if (duty_cycle_B > SECOND_SLICE){
			state_B = 0;
		} else {
			duty_cycle_B += 50;
		}
	} else {
		if (duty_cycle_B < 50){
			state_B = 1;
		} else {
			duty_cycle_B -= 50;
		}
	}
	set_timer();
	start_timer();
	delay_ms(delay_time);
}

int keypad_scan()
{
    //if pressed , keypad return the value of that key, otherwise, return 255 for no pressed (unsigned char)
    int keypad_row=0, keypad_col=0, key_val=-1;

    for(keypad_row=0;keypad_row<KEYPAD_ROW_MAX;keypad_row++) //output data from 1st row
    {
        for(keypad_col=0;keypad_col<KEYPAD_COL_MAX;keypad_col++) //read input data from 1st col
        {
            //use pc 3210 for X output row
            //use pb 3210 for Y input col
        	GPIOC->ODR &= 0; //clear the output value
            GPIOC->ODR |= (1<<keypad_row);//shift the value to send data for that row, data set
            int masked_value=GPIOB->IDR&0xf0;
			int is_pressed=(masked_value>>(keypad_col+4))&1;
            if(is_pressed) //key is pressed
            {
                key_val = keypad_value[keypad_row][keypad_col];
                return key_val; //directly back
            }
            else
            {
            	key_val = -1; //if not pressed, just clear the screen
            }
        }
    }
    return key_val; //return -1 if keypad is not pressed, in such condition the color should maintain the current state
}

//RGB 1 1.5 2
void chromatic_scheme(int key_val)
{
	prev = curr;
	curr = key_val;
	if (curr == prev)
		check = 100;
	else
		check = curr;
	//check on/off then check other button
	if (check==15){
		if(on){
			on = 0;
			//remember setting
			off_state = cur_state;
			off_R = duty_cycle_R;
			off_G = duty_cycle_G;
			off_B = duty_cycle_B;
			//light off
			duty_cycle_R = 0;
			duty_cycle_G = 0;
			duty_cycle_B = 0;
			set_timer();
			stop_timer();
		}else {
			on = 1;
			cur_state = off_state;
			duty_cycle_R = off_R;
			duty_cycle_G = off_G;
			duty_cycle_B = off_B;
			set_timer();
			start_timer();
		}
	} else {
		if(on){
			switch (check)
			{
				case 0:
				{
					if(duty_cycle_R < SECOND_SLICE)
						duty_cycle_R += DELTA_VALUE;
					else
						duty_cycle_R = SECOND_SLICE;
					break;
				}
				case 1:
				{
					if(duty_cycle_G < SECOND_SLICE)
						duty_cycle_G += DELTA_VALUE;
					else
						duty_cycle_G = SECOND_SLICE;
					break;
				}
				case 2:
				{
					if(duty_cycle_B < SECOND_SLICE)
						duty_cycle_B += DELTA_VALUE;
					else
						duty_cycle_B = SECOND_SLICE;
					break;
				}
				case 3:
				{
					if (cur_state==CONTROL_MODE)
						cur_state = CYCLE_MODE;
					else{
						if(speed>SPEED_LIMIT){
							speed -= 30000;
						} else {
							speed = SLOWEST_SPEED;
						}
					}
					break;
				}
				case 4:
				{
					if(duty_cycle_R > DELTA_VALUE)
						duty_cycle_R -= DELTA_VALUE;
					else
						duty_cycle_R = 0;
					break;
				}
				case 5:
				{
					if(duty_cycle_G > DELTA_VALUE)
						duty_cycle_G -= DELTA_VALUE;
					else
						duty_cycle_G = 0;
					break;
				}
				case 6:
				{
					if(duty_cycle_B > DELTA_VALUE)
						duty_cycle_B -= DELTA_VALUE;
					else
						duty_cycle_B = 0;
					break;
				}
				case 7:
				{
					cur_state = CONTROL_MODE;
					break;
				}
				case 8:
				{
					duty_cycle_R = SECOND_SLICE;
					duty_cycle_G = 0;
					duty_cycle_B = 0;
					break;
				}
				case 9:
				{
					duty_cycle_R = 0;
					duty_cycle_G = SECOND_SLICE;
					duty_cycle_B = 0;
					break;
				}
				case 10:
				{
					duty_cycle_R = 0;
					duty_cycle_G = 0;
					duty_cycle_B = SECOND_SLICE;
					break;
				}
				case 11:
				{
					// temperature
					cur_state = TEMP_MODE;
					break;
				}
				case 12: //RG
				{
					duty_cycle_R = SECOND_SLICE;
					duty_cycle_G = SECOND_SLICE;
					duty_cycle_B = 0;
					break;
				}
				case 13: //GB
				{
					duty_cycle_R = 0;
					duty_cycle_G = SECOND_SLICE; // try the coef
					duty_cycle_B = SECOND_SLICE;
					break;
				}
				case 14: //RB
				{
					duty_cycle_R = SECOND_SLICE;
					duty_cycle_G = 0;
					duty_cycle_B = SECOND_SLICE;
					break;
				}
				case 100: //maintain the default state for nop-like operation
				{
					if(cur_state == CYCLE_MODE)
					{
						cycle_mode(speed);
					}
					else if(cur_state == CONTROL_MODE)
					{
						set_timer();
						start_timer();
					}
					else if(cur_state == TEMP_MODE) //temp 20~35
					{
						duty_cycle_R = (global_temperature - 20) * 17;
						duty_cycle_G = 0;
						duty_cycle_B = SECOND_SLICE - (( global_temperature - 20 ) * 17);
					}
					break;
				}
				default:
				{
					if(cur_state == CYCLE_MODE)
					{
						cycle_mode(speed);
					}
					else if(cur_state == CONTROL_MODE)
					{
						set_timer();
						start_timer();
					}
					else if(cur_state == TEMP_MODE)
					{
						duty_cycle_R = (global_temperature - 20) * 17;
						duty_cycle_G = 0;
						duty_cycle_B = SECOND_SLICE - (( global_temperature - 20 ) * 17);
					}
					break;
				}
			}
		}
	}
}

int main()
{
	//use the time delay mode to make the interleaving and the color changing scheme
 	keypad_init();
	GPIO_init_AF();
	Timer_init();
	duty_cycle_R = RED_START;
	duty_cycle_G = GREEN_START;
	duty_cycle_B = BLUE_START;
	cur_state = CYCLE_MODE;
	while(1)
	{
		PWM_channel_init();
		chromatic_scheme(keypad_scan());
	}
	return 0;
}
