#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stm32l476xx.h"
#define keypad_row_max 4
#define keypad_col_max 4
unsigned int keypad_value[4][4] ={{1,2,3,10},
                            	  {4,5,6,11},
								  {7,8,9,12},
								  {16,0,16,13}};
extern void GPIO_init();
extern void max7219_send(unsigned char address, unsigned char data);
extern void max7219_init();
void keypad_init()
{
    GPIO_init(); //have initialized in arm
    RCC->AHB2ENR   |= 0b00000000000000000000000000000111; //safely initialize again

/*  GPIOC->MODER   &= 0b11111111111111111111111100000000; //use pb 3210 for Y input col
    GPIOC->MODER   |= 0b00000000000000000000000001010101; //use pb 3210 for Y input col
    GPIOC->PUPDR   &= 0b11111111111111111111111100000000; //clear and set output use pup since we want 1 to be sent high level voltage
    GPIOC->PUPDR   |= 0b00000000000000000000000001010101; //clear and set output use pup since we want 1 to be sent high level voltage
    GPIOC->OSPEEDR &= 0b11111111111111111111111100000000;
    GPIOC->OSPEEDR |= 0b00000000000000000000000001010101;
    GPIOC->ODR     |= 0b00000000000000000000000011110000;*/

 /* GPIOB->MODER   &= 0b11111111111111111111111100000000; //use pc 3210 for X output row
    GPIOB->PUPDR   &= 1111111111111111111111111100000000; //clear and set input as pdown mode
    GPIOB->PUPDR   |= 0b00000000000000000000000010101010;*/ //clear and set input as pdown mode
    GPIOB->MODER   &= 0b11111111111111111111111100000000;
    GPIOB->MODER   |= 0b00000000000000000000000001010101;
    GPIOB->PUPDR   &= 0b11111111111111111111111100000000;
    GPIOB->PUPDR   |= 0b00000000000000000000000001010101;
    GPIOB->OSPEEDR &= 0b11111111111111111111111100000000;
    GPIOB->OSPEEDR |= 0b00000000000000000000000001010101;
    GPIOB->ODR     |= 0b00000000000000000000000011110000;

    GPIOC->MODER   &= 0b11111111111111111111111100000000;
    GPIOC->PUPDR   &= 0b11111111111111111111111100000000;
    GPIOC->PUPDR   |= 0b00000000000000000000000010101010;
}
/* TODO: scan keypad value
* return:
* >=0: key pressed value
* -1: no key press
*/
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
    int i=0,dig=0;
    //display_clr(8); //clear the old number for trash removing
    for(i=1;i<=num_digs;i++)
    {
        max7219_send(i,data%10);
        dig=data%10;
        data/=10; //get the next digit
    }
    if(data>99999999 || data<-9999999)
        return -1; //out of range error
    else
        return 0; //end this function
}
char keypad_scan()
{
    //if pressed , keypad return the value of that key, otherwise, return 255 for no pressed (unsigned char)
    int keypad_row=0,keypad_col=0,nothing_is_pressed=1;
    char key_val=-1;
    bool hash_map[14]={0}; //hashmap for pressed key
    memset(hash_map,0,sizeof(bool)*14);
    int out_sum=0;
    bool clear=1;
    int state=0;

    while(1)
    {
    	switch(out_sum){
    	case 0 ... 9:
			if(out_sum!=0 || clear!=1)
				display(out_sum,1);
			else
				display_clr(8);
 			break;
 		case 10 ... 99:
			display(out_sum,2);
    		break;
    	case 100 ... 999:
			display(out_sum,3);
    		break;
    	case 1000 ... 9999:
    		display(out_sum,4);
    		break;
    	case 10000 ... 99999:
    		display(out_sum,5);
    		break;
    	case 100000 ... 999999:
    		display(out_sum,6);
    		break;
    	case 1000000 ... 9999999:
    		display(out_sum,7);
    		break;
    	case 10000000 ... 99999999:
    		display(out_sum,8);
    		break;
    	}
        nothing_is_pressed=1;
        int sum;
        switch(state){
        case 0:
        	for(keypad_col=0;keypad_col<keypad_col_max;keypad_col++){ //output data from 1st row
				GPIOB->ODR&=0b0000; //clear the output value
				GPIOB->ODR|=(1<<keypad_col);//shift the value to send data for that row, data set
				for(keypad_row=0;keypad_row<keypad_row_max;keypad_row++){ //read input data from 1st col
					/*use pc 3210 for X output row
					use pb 3210 for Y input col*/

					int masked_value=GPIOC->IDR&0xf,is_pressed=(masked_value>>keypad_row)&1,buffer_idx=0;

					 //clear buffer
					if(is_pressed){ //key is pressed
						nothing_is_pressed=0;
						clear=0;
						key_val=keypad_value[keypad_row][keypad_col];
						if(key_val<14){
							hash_map[key_val]=1; //if that key is pressed, mark the hash_map value to be 1
						}else{
							out_sum=0;
							clear=1;
						}
						sum=0;
						for(int i=0;i<14;i++){ //clear pressed
							if(hash_map[i]){
								sum+=i;
							}
						}

					}

					switch(out_sum){
					case 0 ... 9:
						if(out_sum!=0 || clear!=1)
							display(out_sum,1);
						else
							display_clr(8);
						break;
					case 10 ... 99:
						display(out_sum,2);
						break;
					case 100 ... 999:
						display(out_sum,3);
						break;
					case 1000 ... 9999:
						display(out_sum,4);
						break;
					case 10000 ... 99999:
						display(out_sum,5);
						break;
					case 100000 ... 999999:
						display(out_sum,6);
						break;
					case 1000000 ... 9999999:
						display(out_sum,7);
						break;
					case 10000000 ... 99999999:
						display(out_sum,8);
						break;
					}
				}
			}
        	switch(out_sum){
        	case 0 ... 9:
        		if(out_sum!=0 || clear!=1)
        			display(out_sum,1);
        		else
        			display_clr(8);
        	    break;
        	case 10 ... 99:
        		display(out_sum,2);
        	 	break;
        	case 100 ... 999:
        		display(out_sum,3);
        		break;
        	case 1000 ... 9999:
        	    display(out_sum,4);
        	    break;
        	case 10000 ... 99999:
        	    display(out_sum,5);
        	    break;
        	case 100000 ... 999999:
        	    display(out_sum,6);
        	    break;
        	case 1000000 ... 9999999:
        	   	display(out_sum,7);
        	    break;
        	case 10000000 ... 99999999:
        	    display(out_sum,8);
        	    break;
        	}
			if(nothing_is_pressed){
				state=1; //if not pressed, just clear the screen
				for(int i=0;i<14;i++)
					hash_map[i]=0;
			}

        	break;
        case 1:
        	out_sum+=sum;
        	sum = 0;
        	state = 0;
        	switch(out_sum){
        	case 0 ... 9:
				if(out_sum!=0 || clear!=1)
					display(out_sum,1);
				else
					display_clr(8);
     			break;
     		case 10 ... 99:
				display(out_sum,2);
        		break;
        	case 100 ... 999:
				display(out_sum,3);
        		break;
        	case 1000 ... 9999:
        		display(out_sum,4);
        		break;
        	case 10000 ... 99999:
        		display(out_sum,5);
        		break;
        	case 100000 ... 999999:
        		display(out_sum,6);
        		break;
        	case 1000000 ... 9999999:
        		display(out_sum,7);
        		break;
        	case 10000000 ... 99999999:
        		display(out_sum,8);
        		break;
        	}
        	break;
        }
    }
    return key_val;
}

int main()
{
    GPIO_init();
    max7219_init();
    keypad_init();
    display_clr(8);
    keypad_scan();
    return 0;
}
