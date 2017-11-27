.syntax unified
.cpu cortex-m4
.thumb
.data

.text
	/*Start from manual p75 of GPIO Address data*/
	.global delay_1s
	.equ RCC_AHB2ENR  , 0x4002104C
	.equ GPIOB_MODER  , 0x48000400
	.equ GPIOB_OTYPER , 0x48000404
	.equ GPIOB_OSPEEDR, 0x48000408
	.equ GPIOB_PUPDR  , 0x4800040C
	.equ GPIOB_ODR    , 0x48000414
	.equ onesec, 200000
	.equ interval_cnt, 200000
	/*GPIOC for button*/
	.equ GPIOC_MODER  , 0x48000800
	.equ GPIOC_OTYPER ,	0x48000804
	.equ GPIOC_OSPEEDR,	0x48000808
	.equ GPIOC_PUPDR  ,	0x4800080c
	.equ GPIOC_IDR    , 0x48000810
	//.equ debounce_delay_time,24000 //800k for 1 sec moving the led, button bounce for 10-20m, hence use 800000(30/1000)=24k for interval
do_delay:
	   //TODO: Write a delay 1sec function
	sub r3, r3, #1
	cmp r3, #0
	bne do_delay
	bx lr

delay_1s:
	ldr r3, =onesec
	b do_delay
