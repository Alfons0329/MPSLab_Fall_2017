    .syntax unified
    .cpu cortex-m4
    .thumb

.data
    student_id1: .byte 0, 4, 1, 0, 1, 3, 7 //TODO: put your student id here
    student_id2: .byte 0, 4, 1, 6, 3, 2, 4

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
	//RCC_AHB2ENR: enable GPIOA
	push {r0,r1,r2,lr}
	mov r0, 0b1
	ldr r1, =RCC_AHB2ENR
	str r0, [r1]

	//GPIOA_MODER: PA7 6 5: output
	ldr r0, =0b010101
	lsl r0, 10
	ldr r1, =GPIOA_MODER
	ldr r2, [r1]
	and r2, 0xFFFF03FF //clear 7 6 5
	orrs r2, r2, r0 //7 6 5  --> output
	str r2, [r1]

	//GPIOA_OTYPER: push-pull (reset state)
	//GPIO_OSPEEDR: high speed
	mov r0, 0b101010 //PA2,1,0: high speed
	lsl r0, 10
	ldr r1, =GPIOA_OSPEEDER
	ldr r2, [r1]
	and r2, 0xFFFF03FF
	orrs r2, r2, r0
	str r0, [r1]

	pop {r0,r1,r2,lr}
	BX LR
