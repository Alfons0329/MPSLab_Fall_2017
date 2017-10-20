/*gpio reference http://www.nimblemachines.com/stm32-gpio/*/
.syntax unified
.cpu cortex-m4
.thumb

.data
leds: .byte 0

.text
	.global main  /*Start from manual p75 of GPIO Address data*/
	.equ RCC_AHB2ENR  , 0x4002104C
	.equ GPIOB_MODER  , 0x48000400
	.equ GPIOB_OTYPER , 0x48000404
	.equ GPIOB_OSPEEDR, 0x48000408
	.equ GPIOB_PUPDR  , 0x4800040C
	.equ GPIOB_ODR    , 0x48000414
	.equ onesec, 800000

main:
   	BL   GPIO_init
	MOVS	R1, #1
	LDR	R0, =leds
	STRB	R1, [R0]
	bl first_led
	b Loop
//use
//use r3 for counter values	ex for moving and shift left/right
//use r2 for led data output value address in the future
//use r1 for output the led data value
first_led:
	mov r1, 0xfff3
	strh r1, [r2]
	bx lr
Loop:
	//TODO: Write the display pattern into leds variable

	//BL		DisplayLED

	switch_left:
	mov r3, 0x0
	b goleft
	switch_right:
	mov r3, 0x0
	b goright
	B		Loop

GPIO_init:
  //TODO: Initial LED GPIO pins as output
	//enable the gpio port b to do the tasks
	ldr r0, =RCC_AHB2ENR
	ldr r1, [r0]
	eor r1, 0x00000002
	str r1, [r0]

	//enable the port b GPIOB_MODER for output mode, chiech is 01 (GPOM)
	ldr r0, =GPIOB_MODER
	ldr r1, [r0] //get originally initilized reset value 0xFFFFFEBF
	mov r2, 0x00001540 //0001010101(mode6 to mode4)000000
	//clear pb6~pb3 to zero
	and r1,r1, 0xFFFFC03F //FFFF1100000000111111 from manual p25
	orr r1,r1,r2 //get the value of  FFFF|11|00000000|111111 or 0000|00|01010101|000000
	str r1,[r0]

	//otype is default to pp , no need to change
	//set the speed , defulat value is 0x00000000 low speed, now use high speed
	mov r1, 0x00002A80
	ldr r0, =GPIOB_OSPEEDR
	str r1, [r0]
	//usage r2 for led data output value address in the future
	ldr r2, =GPIOB_ODR
  	BX LR
/*
DisplayLED:
							//		  76543210
	mov r1, 0xffffffe7 //FFFF|1111111111100111
	strh r1,[r2]
	bx lr
*/
goleft:
	push {r3}
	ldr r3, =onesec
	bl Delay
	lsl r1, r1, #1
	/*cmp r1, 0xffffff38cmp r1, 0b11111111111111111111111100111000 //leftboundary*/
 	pop {r3}
 	cmp r3, #3
 	it eq
 	moveq r1,0xff3f //special case of shift logic

 	strh r1,[r2] //srote to output value

	add r3, r3, #1
	cmp r3,#4
	beq switch_right
	bne goleft
goright:
	push {r3}
	ldr r3, =onesec
	bl Delay
	lsr r1, r1, #1
	/*cmp r1, 0xffffff38cmp r1, 0b11111111111111111111111100111000 //leftboundary*/
	strh r1,[r2] //srote to output value

	pop {r3}
	add r3, r3, #1
	cmp r3,#4
	beq switch_left
	bne goright

Delay:
   //TODO: Write a delay 1sec function
	sub r3, r3, #1
	cmp r3, 0
	bne Delay
	bx lr
/*
fedbca9876543210
1111111111110011 initialize
1111111111100110 lsl 1
1111111111001100 lsl 2
1111111110011000 lsl 3
1111111100110000 lsl 4
0111111110011000 lsr 1
0011111111001100 lsr 2
0001111111100110 lsr 3
0000111111110011 lsr 4
*/
