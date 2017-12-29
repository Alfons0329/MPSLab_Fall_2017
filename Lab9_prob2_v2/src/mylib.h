#include "stm32l476xx.h"
#ifndef _MYLIB_H_
#define _MYLIB_H_
/*
This lib include some useful functions
*/
/*
reference from http://stm32.kosyak.info/doc/group___exported__macro.html

*/
#define GPIO_PIN_0   ((uint16_t) 0x0001)
#define GPIO_PIN_1   ((uint16_t) 0x0002)
#define GPIO_PIN_2   ((uint16_t) 0x0004)
#define GPIO_PIN_3   ((uint16_t) 0x0008)
#define GPIO_PIN_4   ((uint16_t) 0x0010)
#define GPIO_PIN_5   ((uint16_t) 0x0020)
#define GPIO_PIN_6   ((uint16_t) 0x0040)
#define GPIO_PIN_7   ((uint16_t) 0x0080)
#define GPIO_PIN_8   ((uint16_t) 0x0100)
#define GPIO_PIN_9   ((uint16_t) 0x0200)
#define GPIO_PIN_10  ((uint16_t) 0x0400)
#define GPIO_PIN_11  ((uint16_t) 0x0800)
#define GPIO_PIN_12  ((uint16_t) 0x1000)
#define GPIO_PIN_13  ((uint16_t) 0x2000)
#define GPIO_PIN_14  ((uint16_t) 0x4000)
#define GPIO_PIN_15  ((uint16_t) 0x8000)
#define GPIO_PIN_ALL ((uint16_t) 0xFFFF)

#define SET_BIT(REG, BIT)   ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)   ((REG) & (BIT))
#define CLEAR_REG(REG)   ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)   ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)   WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

float resistor = 0.0f;
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

static inline void delay_us(int n_in){
	asm("push {r0}\r\n"
			"mov r0, r0\r\n"
			"LOOP_US:\r\n"
			"nop\r\n"
			"subs r0, #1\r\n"
			"BGT LOOP_US\r\n"
			"POP {r0}\r\n"
			:: "r" (n_in));

}

static inline void delay_ms(int n){
	asm("push {r0}\r\n"
			"mov r0, r0\r\n"
			"LOOP:\r\n"
			"subs r0, #1\r\n"
			"BGT LOOP\r\n"
			"POP {r0}\r\n"
			:: "r" (n*1333));
}



#endif
