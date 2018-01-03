#include "delay.h"
void delay(int d){
	int temp = TIM2->CNT;
	while(d){
		while(temp == TIM2->CNT);
		d--;
		temp = TIM2->CNT;
	}
	return;
}