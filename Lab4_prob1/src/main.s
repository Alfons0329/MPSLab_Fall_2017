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

main:
   	BL   GPIO_init
	MOVS	R1, #1
	LDR	R0, =leds
	STRB	R1, [R0]
	bl first_led
	b Loop
//use r2 for led data output value address in the future
//use r1 for output the led data value
first_led:
	mov r1, 0xfff7
	strh r1, [r2]
	bx lr
Loop:
	//TODO: Write the display pattern into leds variable

	BL		DisplayLED
   	BL   	Delay
	switch_left:
	b goleft
	switch_right:
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

DisplayLED:
	mov r1, 0xffffffe7 //FFFF|1111111111100111
	strh r1,[r2]
	BX LR
goleft:
	lsl r1, r1, #1
	/*cmp r1, 0xffffff38cmp r1, 0b11111111111111111111111100111000 //leftboundary*/
	strh r1,[r2]
	ite eq
	beq switch_right
	bne goleft
goright:
	lsr r1, r1, #1
	strh r1,[r2]
	cmp
Delay:
   //TODO: Write a delay 1sec function

BX LR

end:
	b end
