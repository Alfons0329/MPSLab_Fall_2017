.syntax unified
.cpu cortex-m4
.thumb
.text
.global GPIOAC_init
    .equ	RCC_AHB2ENR,	0x4002104C
    .equ	GPIOA_MODER,	0x48000000
    .equ	GPIOA_OSPEEDER,	0x48000008
    .equ	GPIOA_PUPDR,	0x4800000C
    .equ	GPIOA_IDR,		0x48000010
    .equ	GPIOA_ODR,		0x48000014
    .equ	GPIOA_BSRR,		0x48000018 //set bit
    .equ	GPIOA_BRR,		0x48000028 //clear bit
    //use for
    .equ    RCC_AHB2ENR  , 0x4002104C
    .equ    GPIOB_MODER  , 0x48000400
    .equ    GPIOB_OTYPER , 0x48000404
    .equ    GPIOB_OSPEEDR, 0x48000408
    .equ    GPIOB_PUPDR  , 0x4800040C
    .equ    GPIOB_ODR    , 0x48000414

    .equ GPIOC_MODER  , 0x48000800
    .equ GPIOC_OTYPER ,	0x48000804
    .equ GPIOC_OSPEEDR,	0x48000808
    .equ GPIOC_PUPDR  ,	0x4800080c
    .equ GPIOC_IDR    , 0x48000810
    //.equ    onesec, 800000
    //.equ    interval_cnt, 200000
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

GPIO_init:
//TODO: Initialize three GPIO pins as output for max7219 DIN, CS and CLK
//RCC_AHB2ENR: enable GPIOA and B and C
    push {r0,r1,r2,lr}
    ldr r0, =RCC_AHB2ENR
    mov r1, 0b111
    str r1, [r0]

    //GPIOA_MODER: PA7 6 5: output
    ldr r0, =0b010101
    lsl r0, 10
    ldr r1, =GPIOA_MODER
    ldr r2, [r1]
    and r2, 0xFFFF03FF //clear 7 6 5
    orrs r2, r2, r0 //7 6 5  --> output
    str r2, [r1]

    ldr r0, =GPIOC_MODER
	ldr r1, [r0]
	//clear pc13 to zero
	and r1, r1, 0xf3ffffff
	str r1,	[r0]

    pop {r0,r1,r2,lr}
BX LR
