    .syntax unified
    .cpu cortex-m4
    .thumb
.text
    .global GPIO_init
    .equ	RCC_AHB2ENR,	0x4002104C
    .equ	GPIOA_MODER,	0x48000000
    .equ	GPIOA_OSPEEDER,	0x48000008
    .equ	GPIOA_PUPDR,	0x4800000C
    .equ	GPIOA_IDR,		0x48000010
    .equ	GPIOA_ODR,		0x48000014
    .equ	GPIOA_BSRR,		0x48000018 //set bit
    .equ	GPIOA_BRR,		0x48000028 //clear bit
    .equ    GPIOB_MODER  , 0x48000400
    .equ    GPIOB_OTYPER , 0x48000404
    .equ    GPIOB_OSPEEDR, 0x48000408
    .equ    GPIOB_PUPDR  , 0x4800040C
    .equ    GPIOB_ODR    , 0x48000414

    .equ 	DIN,	0b100000 	//PA5
    .equ	CS,		0b1000000	//PA6
    .equ	CLK,	0b10000000	//PA7
    //max7219
    .equ	DECODE,			0x19 //decode control
    .equ	INTENSITY,		0x1A //brightness
    .equ	SCAN_LIMIT,		0x1B //how many digits to display
    .equ	SHUT_DOWN,		0x1C //shut down -- we did't use this actually
    .equ	DISPLAY_TEST,	0x1F //display test -- we did' use this actually

GPIO_init:
	//TODO: Initialize three GPIO pins as output for max7219 DIN, CS and CLK
	//RCC_AHB2ENR: enable GPIOA and B and C
	push {r0,r1,r2,lr}
    ldr r0, =RCC_AHB2ENR
    mov r1, 0b111
    str r1, [r0]
/*
	//GPIOA_MODER: PA7 6 5: output
	ldr r0, =0b010101
	lsl r0, 10
	ldr r1, =GPIOA_MODER
	ldr r2, [r1]
	and r2, 0xFFFF03FF //clear 7 6 5
	orrs r2, r2, r0 //7 6 5  --> output
	str r2, [r1]
*/
	ldr  r1, =GPIOA_MODER
	ldr  r2, [r1]
	and  r2, 0b11111111111111111111111100111111
	orr  r2, 0b00000000000000000000000010000000 //alternate function mode for PA3
	str  r2, [r1]
/*	//GPIOA_OTYPER: push-pull (reset state)
	//GPIO_OSPEEDR: high speed
	mov r0, 0b101010 //PA2,1,0: high speed
	lsl r0, 10
	ldr r1, =GPIOA_OSPEEDER
	ldr r2, [r1]
	and r2, 0xFFFF03FF
	orrs r2, r2, r0
	str r0, [r1]
*/
    /*
    //enable the port b GPIOB_MODER for INPUT MODE FOR KEYPAD
	ldr r0, =GPIOB_MODER
	ldr r1, [r0] //get originally initilized reset value 0xFFFFFEBF
	mov r2, 0x00000000 //pb7~pb0 for input mode
	//clear pb7~pb0 to zero
	and r1, r1, 0xFFFF0000 //FFFF1100000000111111 from manual p25
	orr r1, r1, r2 //get the value of
	str r1,	[r0]*/

	pop {r0,r1,r2,lr}
	BX LR
