	.syntax unified
	.cpu cortex-m4
	.thumb
.data
	//arr: 0,1,2,3,4,5,6,7,8,9,A,b,C,d,E
	arr: .byte 0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B, 0x77, 0x1F, 0x4E, 0x3D, 0x4F, 0x47 //TODO: put 0 to F 7-Seg LED pattern here

.text
	.global main
	.equ	RCC_AHB2ENR,	0x4002104C
	.equ	GPIOA_MODER,	0x48000000
	.equ	GPIOA_OTYPER,	0x48000004
	.equ	GPIOA_OSPEEDER,	0x48000008
	.equ	GPIOA_PUPDR,	0x4800000C
	.equ	GPIOA_ODR,		0x48000014

main:
    BL   GPIO_init
    BL   max7219_init
loop:
    BL   Display0toF
    B loop

GPIO_init:
	//TODO: Initialize three GPIO pins as output for max7219 DIN, CS and CLK

	BX LR

Display0toF:
	//TODO: Display 0 to F at first digit on 7-SEG LED. Display one per second.
	BX LR

MAX7219Send:
   //input parameter: r0 is ADDRESS , r1 is DATA
	//TODO: Use this function to send a message to max7219
	BX LR

max7219_init:
	//TODO: Initialize max7219 registers
	BX LR

Delay:
	//TODO: Write a delay 1sec function
	BX LR
