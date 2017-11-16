	.syntax unified
	.cpu cortex-m4
	.thumb

.data
	student_id1: .byte 0, 4, 1, 0, 1, 3, 7 //TODO: put your student id here
	student_id2: .byte 0, 4, 1, 6, 3, 2, 4

.text
	.global max7219_init
	.equ	RCC_AHB2ENR,	0x4002104C
	.equ	GPIOA_MODER,	0x48000000
	.equ	GPIOA_OSPEEDER,	0x48000008
	.equ	GPIOA_PUPDR,	0x4800000C
	.equ	GPIOA_IDR,		0x48000010
	.equ	GPIOA_ODR,		0x48000014
	.equ	GPIOA_BSRR,		0x48000018 //set bit
	.equ	GPIOA_BRR,		0x48000028 //clear bit

	//Din, CS, CLK offset
	.equ 	DIN,	0b100000 	//PA5
	.equ	CS,		0b1000000	//PA6
	.equ	CLK,	0b10000000	//PA7

	//max7219
	.equ	DECODE,			0x19 //decode control
	.equ	INTENSITY,		0x1A //brightness
	.equ	SCAN_LIMIT,		0x1B //how many digits to display
	.equ	SHUT_DOWN,		0x1C //shut down -- we did't use this actually
	.equ	DISPLAY_TEST,	0x1F //display test -- we did' use this actually

max7219_init:
	//TODO: Initialize max7219 registers
	push {r0, r1, LR}
	ldr r0, =DECODE
	ldr r1, =0xFF //CODE B decode for digit 0-7
	bl max7219_send

	ldr r0, =DISPLAY_TEST
	ldr r1, =0x0 //normal operation
	bl max7219_send

	ldr r0, =INTENSITY
	ldr r1, =0xA //brightness (21/32)
	bl max7219_send

	ldr r0, =SCAN_LIMIT
	ldr r1, =0x1 //light up digit 0-6
	bl max7219_send

	ldr r0, =SHUT_DOWN
	ldr r1, =0x1 //normal operation
	bl max7219_send

	pop {r0, r1, PC}
	BX LR
