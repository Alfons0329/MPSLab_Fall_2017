#include "stm32.h"
#include "delay.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
extern void GPIO_init();
extern void FPU_init();

void WriteToLCD(int input, int isCMD){
	int ifront = input >> 4;
	int iback = input - (ifront << 4);
	if(isCMD == 1){
		//RS low, RW low
		GPIOA->BRR = 0x3;
		GPIOA->ODR &= ~(0xF << 4);
		GPIOA->ODR |= (iback << 4);
		GPIOB->ODR &= ~(0xF << 3);
		GPIOB->ODR |= (ifront << 3);
	}
	else {
		//RS high, RW low
		GPIOA->BSRR = (0x2 << 15) | 0x1;
		GPIOA->ODR &= ~(0xF << 4);
		GPIOA->ODR |= (iback << 4);
		GPIOB->ODR &= ~(0xF << 3);
		GPIOB->ODR |= (ifront << 3);
	}
	//high EN
	GPIOA->BSRR = (1 << 9);
	delay(5);
	GPIOA->BRR = (1 << 9);
	delay(100);
}
void action(char* input){

	char* name_with_extension;
	name_with_extension = malloc(strlen(input)+1+8); /* make space for the new string (should check the return value ...) */
	strcpy(name_with_extension, input); /* copy name into the new var */
	strcat(name_with_extension, " showed\r\n"); /* add the extension */
	UART_Transmit( name_with_extension, strlen(name_with_extension));

}
void LCD_init(){
	WriteToLCD(0x38, 1);// Function Setting(8bit, double row)
	WriteToLCD(0x06, 1);// Entering mode
	WriteToLCD(0x0C, 1);// Display on
	WriteToLCD(0x01, 1);// Clear screen
	WriteToLCD(0x80, 1);// Move to top left
}
void USART3_Init(void) {
	// Enable clock for USART3
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
	// CR3
	USART3->CR1 &= ~(USART_CR1_M | USART_CR1_PS | USART_CR1_PCE | USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8);
	USART3->CR1 |=  USART_CR1_TE | USART_CR1_RE;
	// CR2
	USART3->CR2 &= ~USART_CR2_STOP; // 1-bit stop
	// CR3
	USART3->CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT);// none hwflowctl
	USART3->BRR &= ~(0xFF);
	USART3->BRR |= 40000000L/9600L;
	// In asynchronous mode, the following bits must be kept cleared:
	//- LINEN and CLKEN bits in the USART_CR2 register,
	//- SCEN, HDSEL and IREN bits in the USART_CR3 register.
	USART3->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
	USART3->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);
	// Enable UART
	USART3->CR1 |= (USART_CR1_UE);
}
void GPIO_init_AF(){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
	//PC13 input mode(00)
	GPIOC->MODER &= ~(0x3 << 26);
	//PB11(Serial3_RX).10(Serial3_TX)
	//PB10.11 AF mode(10)
	GPIOB->MODER &= ~(0xF << 20);
	GPIOB->MODER |= 0xA << 20;
	//push-pull
	GPIOB->OTYPER &= ~(0x3 << 10);
	//low speed
	GPIOB->OSPEEDR &= ~(0xF << 20);
	//no pull
	GPIOB->PUPDR &= ~(0xF << 20);
	//PB10.11 select AF
	GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFF << 8)) | (0x77 << 8); // AF7 for pin
	//PA5 output mode(01)
	GPIOA->MODER &= ~(0x3 << 10);
	GPIOA->MODER |= 0x1 << 10;
	//Set PA5 as high speed mode(10)
	GPIOA->OSPEEDR &= ~(0x3 << 10);
	GPIOA->OSPEEDR |= 0x2 << 10;
	GPIOA->BRR |= 1 << 5;
}
void Timer_init(){
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;//tim2 clock enable
	TIM2->PSC = 40-1;//Prescalser
	TIM2->ARR = (uint32_t)40000000-1;//Reload value
	TIM2->EGR = TIM_EGR_UG;//Reinitialize the count
	TIM2->CR1 |= TIM_CR1_CEN;//start timer
}

void SystemClock_Config(void){
	//select HSI clock
	RCC->CFGR &= 0xFFFFFFFC;
	RCC->CFGR |= 0x1;
	RCC->CR |= RCC_CR_HSION;
	while((RCC->CR & RCC_CR_HSIRDY) == 0);//wait for HSI ready
	RCC->CR &= ~RCC_CR_PLLON;
	while((RCC->CR & RCC_CR_PLLRDY) == 1);//wait for PLL turn off
	//select PLL clock
	RCC->CFGR &= 0xFFFFFFFC;
	RCC->CFGR |= 0x3;
	//Mask PLLR PLLN PLLM
	RCC->PLLCFGR &= 0xF9EE808C;
	//PLLM (4~16)MHz = 2
	RCC->PLLCFGR |= (1 << 4);
	//PLLN (64~344)MHz
	RCC->PLLCFGR |= (10 << 8);
	//PLLR = 2 <80MHz
	RCC->PLLCFGR |= (0 << 25);
	//PLLSRC = HSI16
	RCC->PLLCFGR |= 0x00000002;
	//HPRE
	RCC->CFGR &= 0xFFFFFF0F;
	RCC->CFGR |= 0;//SYSCLK
	RCC->CR |= RCC_CR_PLLON;
	RCC->PLLCFGR |= (1 << 24);
	Timer_init();
}

void UART_Transmit(char *input) {
	if(SysTick->CTRL & SysTick_CTRL_TICKINT_Msk){
		while (!(USART3->ISR & USART_ISR_TXE));
		USART3->TDR = (uint16_t)'\r';
	}
	char *arr = input;
	while(*arr){
		// Wait to be ready, buffer empty
		while (!(USART3->ISR & USART_ISR_TXE));
		// Send data
		USART3->TDR = (uint16_t)(*arr++);
	}

}

void write_LCD(char *input){
	//WriteToLCD(0x01, 1);
	int position=0;
	int count=0;
	for(count =0;count<strlen(input);count ++){
		WriteToLCD((0x80 | position), 1);// Move to DD RAM position
			WriteToLCD(input[count], 0);
			position++;
	}
}

int main(){
	GPIO_init();
	SystemClock_Config();
	Timer_init();
	LCD_init();

	WriteToLCD(0x01, 1);// Clear screen
	GPIO_init_AF();
	USART3_Init();

	char input[8];
	int inputlen = 0;
	// Wait to be ready, buffer empty
	while (!(USART3->ISR & USART_ISR_TXE));
	while(1){
		if (USART3->ISR & USART_ISR_RXNE){
			// Wait to be ready, buffer empty
			while (!(USART3->ISR & USART_ISR_TXE));

			if((char)USART3->RDR == '\r'){		//enter
				//WriteToLCD(0x01, 1);
				USART3->TDR = (uint16_t)'\r';
				while (!(USART3->ISR & USART_ISR_TXE));
				USART3->TDR = (uint16_t)'\n';
				if(inputlen > 0){
					input[inputlen] = '\0';
					write_LCD("        ");
					write_LCD(input);
					action(input);
				}
				inputlen = 0;
				while (!(USART3->ISR & USART_ISR_TXE));
			}
			else {
				USART3->TDR = (uint16_t)(USART3->RDR);
				input[inputlen++] = (char)(USART3->RDR);
			}
		}
	}
}

