#include <stdio.h>
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
			"mov r0, %0\r\n"
			"LOOP:\r\n"
			"subs r0, #1\r\n"
			"BGT LOOP\r\n"
			"POP {r0}\r\n"
			:: "r" (n*1333));
}
void GPIO_Init(void)
{

}
void configureADC()
{
	// TODO
}
void startADC()
{
	// TODO
}
int main()
{

}
