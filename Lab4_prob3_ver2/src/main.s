.syntax unified
.cpu cortex-m4
.thumb

.data
	leds: .byte 0
	password: .byte 0b1100 /*Lock 1down 2up 3up 4up correct, otherwise, blink, debug purpose*/
	//Problem: seems onlt lock2 has the correct signal, the rest all1(or output 0 at lock due to eor)
.text
	//Start from manual p75 of GPIO Address data
	.global main
	.equ RCC_AHB2ENR  , 0x4002104C
	.equ GPIOB_MODER  , 0x48000400
	.equ GPIOB_OTYPER , 0x48000404
	.equ GPIOB_OSPEEDR, 0x48000408
	.equ GPIOB_PUPDR  , 0x4800040C
	.equ GPIOB_ODR    , 0x48000414

	.equ quarter_sec  , 200000 //from trial and error XD
	.equ wait_for_input, 1000000
	//GPIOC for button
	.equ GPIOC_MODER  , 0x48000800
	.equ GPIOC_OTYPER ,	0x48000804
	.equ GPIOC_OSPEEDR,	0x48000808
	.equ GPIOC_PUPDR  ,	0x4800080C
	.equ GPIOC_IDR    , 0x48000810
	.equ LED_ALLON    , 0xff87
	.equ LED_ALLOFF	  , 0xffff
main:
    BL GPIO_init
	MOVS	R1, #1
	LDR	R0, =leds
	STRB	R1, [R0]
	mov r6, #0
	ldr r1, =LED_ALLON
	strh r1, [r2]
	mov r0, #0
	b Loop

Loop:
	mov r0, r0 //continue to accumulate the signal of threshold
	b check_button
	check_end:
	cmp r6, #1 //button is pressed, check lock
	beq check_lock

	blink_end:
	mov r6, #0
	B		Loop

GPIO_init:
  	//TODO: Initial LED GPIO pins as output
	//enable the gpio port ABC
	ldr r0, =RCC_AHB2ENR
	mov r1, 0b111
	str r1, [r0]

	//enable the port b GPIOB_MODER for output mode
	ldr r0, =GPIOB_MODER
	ldr r1, [r0] //get originally initilized reset value 0xFFFFFEBF
	mov r2, 0x00001540 //0001010101(pb_mode6 to pb_mode3)000000
	//clear pb6~pb3 to zero
	and r1, r1, 0xFFFFC03F //FFFF1100000000111111 from manual p25
	orr r1, r1, r2 //get the value of  FFFF|11|00000000|111111 or 0000|00|01010101|000000
	str r1,	[r0]

	//otype is default to pp , no need to change
	//set the speed , defulat value is 0x00000000 low speed, now use high speed
	mov r1, 0x00002A80
	ldr r0, =GPIOB_OSPEEDR
	str r1, [r0]
	//usage r2 for led data output value address in the future
	//ldr r2, =GPIOB_ODR

	//enable the port c GPIOC_MODER for input mode
	ldr r0, =GPIOC_MODER
	ldr r1, [r0]
	//clear pc13 to zero, also PC5432
	ldr r1, =0xf3ffff00
	//and r1, r1, 0xf3ffff00
	str r1,	[r0]

	//otype is default to pp , no need to change
	//usage r4 for button data input value address in the future

	ldr  r1, =GPIOC_PUPDR
	ldr  r0, [r1]
	ldr  r2, =0b01010101
	and  r0, 0xFFFFFF00
	orr  r0, r2
	str  r0, [r1]

	ldr r2, =GPIOB_ODR
	ldr r4, =GPIOC_IDR //button
	ldr r8, =GPIOC_IDR //DIP
  	BX LR

check_button: //check every cycle, and accumulate 1
	ldr r5, [r4] //fetch the data from button
	lsr r5, r5, #13
	and r5, r5, 0x1 //filter the signal (bit mask)
	cmp r5, #0 //FUCK DONT KNOW WHY THE PRESSED SIGNAL IS 0
	it eq
	addeq r0, r0 ,#1 //accumulate until the threshold

	cmp r5, #1 //not stable, go back to accumulate again
	it eq
	moveq r0, #0

	cmp r0, #1000 //threshold achieved BREAKDOWN! //use #1 for slow motion debug
	it eq
	moveq r6, #1 //r6 0 not pressed, 1 pressed

	b check_end
	//use r8 to get the address of lock
	//use r7 to get data from lock
	//use r6 for button press flag
	//use r5 to get data from button
	//use r4 to get the address of button
	//use r3 for LED delay counter
	//use r2 to get the address of LED
	//use r1 to output the led data value
	//use r0 for check_button counter
check_lock:
	ldr r3, =wait_for_input //if slow motion debug, comment this line
	bl delay_quarter_sec //if slow motion debug, comment this line
	ldr  r7, [r8]
	and  r7, 0b1111
	eor  r7, 0b1111
	ldr  r9, =password
	ldrb r9, [r9]

	cmp r7, r9
	beq led_blink_three
	b led_blink_once

led_blink_three:
	ldr r3, =quarter_sec
	bl delay_quarter_sec //delay for sometime
	ldr r3, =quarter_sec
	bl delay_quarter_sec
	ldr r1, =LED_ALLON//ff|1000|0111|
	strh r1, [r2]
	ldr r3, =quarter_sec
	bl delay_quarter_sec

	ldr r1, =LED_ALLOFF //ff|1111|1111|
	strh r1, [r2]
	ldr r3, =quarter_sec
	bl delay_quarter_sec

	ldr r1, =LED_ALLON
	strh r1, [r2]
	ldr r3, =quarter_sec
	bl delay_quarter_sec

	ldr r1, =LED_ALLOFF
	strh r1, [r2]
	ldr r3, =quarter_sec
	bl delay_quarter_sec

	ldr r1, =LED_ALLON
	strh r1, [r2]
	ldr r3, =quarter_sec
	bl delay_quarter_sec

	ldr r1, =LED_ALLOFF
	strh r1, [r2]
	ldr r3, =quarter_sec
	bl delay_quarter_sec


	delay_end:

	b blink_end

led_blink_once:
	ldr r3, =quarter_sec
	bl delay_quarter_sec //delay for sometime
	ldr r3, =quarter_sec
	bl delay_quarter_sec
	ldr r1, =LED_ALLON //ff|1000|0111|
	strh r1, [r2]
	ldr r3, =quarter_sec
	bl delay_quarter_sec

	ldr r1, =LED_ALLOFF  //ff|1111|1111|
	strh r1, [r2]

	b blink_end

delay_quarter_sec:
	subs r3, r3, #1
	cmp r3, #0
	bne delay_quarter_sec
	bx lr
