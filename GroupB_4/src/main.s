.syntax unified
	.cpu cortex-m4
	.thumb
.data

.text
	.global main
	.equ thousand, 1000
	.equ million, 250000
	.equ hundred, 1000
	.equ ten,10
	.equ A,	0xF3FFFFFF
	.equ GPIOC,			0x48000800
	.equ GPIOC_MODER,	0x48000800
	.equ GPIOC_OTYPER, 	0x48000804
	.equ GPIOC_OSPEEDR, 0x48000808
	.equ GPIOC_PUPDR, 	0x4800080C
	.equ GPIOC_IDR, 	0x48000810
	.equ GPIOC_ODR, 	0x48000814
	.equ GPIO_BSRR,		0x18
	.equ GPIO_BRR,		0x28

	//GPIO
	.equ	RCC_AHB2ENR,	0x4002104C
	.equ	GPIOA,	0x48000000
	.equ	GPIOA_MODER,	0x48000000
	.equ	GPIOA_OTYPER,	0x48000004
	.equ	GPIOA_OSPEEDER,	0x48000008
	.equ	GPIOA_PUPDR,	0x4800000C
	.equ	GPIOA_IDR,		0x48000010
	.equ	GPIOA_ODR,		0x48000014
	//.equ	GPIOA_BSRR,		0x48000018 //set bit -> 1
	//.equ	GPIOA_BRR,		0x48000028 //clear bit -> 0

	//Din, CS, CLK offset
	.equ 	DIN,	0b100000 	//PA5
	.equ	CS,		0b1000000	//PA6
	.equ	CLK,	0b10000000	//PA7

	//max7219
	.equ	DECODE,			0x19 //decode control
	.equ	INTENSITY,		0x1A //brightness
	.equ	SCAN_LIMIT,		0x1B //how many digits to display
	.equ	SHUT_DOWN,		0x1C //shut down -- we did't use this
	.equ	DISPLAY_TEST,	0x1F //display test -- we did' use this

main:
    BL   GPIO_init
    BL   max7219_init
    bl	 stopwatch_init
    movs r0,#0
    push {r0}
	b User_button

GPIO_init:
	//TODO: Initialize three GPIO pins as output for max7219 DIN, CS and CLK
		//TODO: Initialize three GPIO pins as output for max7219 DIN, CS and CLK
	//RCC_AHB2ENR: enable GPIOA
	mov r0, 0b101
	ldr r1, =RCC_AHB2ENR
	str r0, [r1]

	//GPIOA_MODER: PA7,6,5: output
	ldr r0, =0b010101
	lsl r0, 10
	ldr r1, =GPIOA_MODER
	ldr r2, [r1]
	and r2, 0xFFFF03FF //clear 7 6 5
	orrs r2, r2, r0 //7 6 5  --> output
	str r2, [r1]

	//GPIOA_OTYPER: push-pull (reset state)
	//GPIO_OSPEEDR: high speed
	mov r0, 0b101010 //PA7,6,5: high speed
	lsl r0, 10
	ldr r1, =GPIOA_OSPEEDER
	ldr r2, [r1]
	and r2, 0xFFFF03FF
	orrs r2, r2, r0
	str r0, [r1]

	ldr r0, =GPIOC_MODER
	ldr r1, [r0]
	//clear pc13 to zero
	and r1, r1, 0xF3FFFFFF
	str r1,	[r0]

	BX LR

User_button:
	//push {r0,r1,r2,lr}

	ldr r0,=GPIOC_IDR
	ldr r3,[r0]
	movs r4,#1
	lsl r4,#13
	ands r3,r4
	beq debounce//0=push,1=unpush
	pop {r0}
	push {r0}
	cmp r0,#1
	beq short_pushed
	b User_button
	//pop {r0,r1,r2,lr}
debounce://for unpush->push
	bl Delay
	ldr r0,=GPIOC_IDR
	ldr r3,[r0]
	movs r4,#1
	lsl r4,#13
	ands r3,r4
	beq do_pushed //pushed
	b User_button //unpush
debounce_2://for short push->unpush
	bl Delay
	ldr r0,=GPIOC_IDR
	ldr r3,[r0]
	movs r4,#1
	lsl r4,#13
	ands r3,r4
	bne short_pushed //unpush
	beq do_loop //push
do_pushed:
//this loop is one sec
	pop	{r0}
	cmp r0,#1
	//if press button twice
	beq stop
	movs r0,#1
	push {r0}
do_loop:
	ldr r0,=GPIOC_IDR
	ldr r3,[r0]
	movs r4,#1
	lsl r4,#13
	ands r3,r4
	bne debounce_2
	//loop until unpush
	b do_loop
//program end
stop:
	//movs r0,#0
	b stop
//counting time here
//r10 for msec
//r11 for sec
//r12 for min
short_pushed:
	bl Delay_msec
	adds r10,r10,#1//1cycle
	cmp r10,#1000
	beq sec
	b Display
sec:
	movs r10,#0
	add r11,r11,#1
	cmp r11,#60
	beq min
	b Display
min:
	movs r11,#0
	add r12,r12,#1
Display:
	//TODO: Display time
//msec
	movs r3,#10
	//if 312
	udiv r6,r10,r3//r6=31
	mul r7,r6,r3//r7=310
	subs r5,r10,r7//r5=312-310 = 2
	udiv r4,r6,r3 //r4=3
	mul r7,r4,r3//r7=30
	subs r6,r6,r7//r5=1

	push {r5,r6}
	movs r0,#0x3
	movs r1,r4
	bl MAX7219Send
	pop  {r5,r6}
	push {r5}
	movs r0,#0x2
	movs r1,r6
	bl MAX7219Send
	pop {r5}
	movs r0,#0x1
	movs r1,r5
	bl MAX7219Send
//sec
	//if 55
	movs r3,#10
	udiv r5,r11,r3//r5十位數=5
	mul r7,r5,r3
	subs r6,r11,r7//r6個位數
	movs r0,#0x4
	movs r1,r6
	push {r5}
	bl MAX7219Send
	pop {r5}
	movs r0,#0x5
	movs r1,r5
	bl MAX7219Send
//min
	movs r3,#10
	udiv r5,r12,r3//r5十位數
	mul r7,r5,r3
	subs r6,r12,r7//r6個位數
	movs r0,#0x6
	movs r1,r6
	push {r5}
	bl MAX7219Send
	pop {r5}
	movs r0,#0x7
	movs r1,r5
	bl MAX7219Send

	b User_button
MAX7219Send:
   //input parameter: r0 is ADDRESS , r1 is DATA
   //occupy r0~r7
	//TODO: Use this function to send a message to max7219
	lsl r0, r0, #8
	add r0, r0, r1//merge r0,r1 to a 16bit data
	ldr r1, =GPIOA
	ldr r2, =CS
	ldr r3, =DIN
	ldr r4, =CLK
	ldr r5, =GPIO_BSRR
	ldr r6, =GPIO_BRR
	mov r7, #16//r7 = i
max7219send_loop:
//occupy r8~r9
	mov r8, #1 //for selecting 'the bit' we want to deliver
	sub r9, r7, #1
	lsl r8, r8, r9 // r8 = mask
	str r4, [r1,r6]//set clk=0
	tst r0, r8 //1=not equal 1, 0=equal 1
	beq bit_not_set//bit not set
	str r3, [r1,r5] //deliver '1' to DIN
	b if_done
bit_not_set:
	str r3, [r1,r6] //deliver '0' to DIN
if_done:
	str r4, [r1,r5]//set clk=1
	subs r7, r7, #1//i--
//	cmp r7,#-1
	bgt max7219send_loop
	str r2, [r1,r6] //set cs=1
	str r2, [r1,r5] //set cs=0
	BX LR
max7219_init:
	//TODO: Initialize max7219 registers
	push {r0, r1, r2, lr}
	ldr r0, =DECODE
	ldr r1, =#0xFF//open codeB decode mode
	BL MAX7219Send
	ldr r0, =DISPLAY_TEST
	ldr r1, =#0x0//close display_test
	BL MAX7219Send
	ldr r0, =SCAN_LIMIT
	ldr r1, =#0x6//show digit0~6
	BL MAX7219Send
	ldr r0, =INTENSITY
	ldr r1, =#0x9//brightness 19/32
	BL MAX7219Send
	ldr r0, =SHUT_DOWN
	ldr r1, =#0x1//normal operation
	BL MAX7219Send
	pop {r0, r1, r2, pc}

	BX LR
Delay:
	//TODO: Write a delay 20msec function for debounce
	ldr r5,=thousand//2
d_loop:
	subs r5,r5,#1//1
	//cmp r5,#0//1
	bgt d_loop//1 or 3

	BX LR

Delay_msec:
	//TODO: Write a delay 1msec function fot time counting
	ldr r5,=hundred//2
dmsec_loop:
	subs r5,r5,#1//1
//	cmp r5,#0//1
	bgt dmsec_loop//1 or 3

	BX LR
stopwatch_init:
//r10=0(initial time=0)
	push {lr}
	movs r10,#0
	movs r11,#0
	movs r12,#0
	//print the initial value
	movs r0, 1
	movs r1, 0xF
	bl MAX7219Send
	movs r0, 2
	movs r1, 0xF
	bl MAX7219Send
	movs r0, 3
	movs r1, 0xF
	bl MAX7219Send
	movs r0, 4
	movs r1, 0xF
	bl MAX7219Send
	movs r0, 5
	movs r1, 0xF
	bl MAX7219Send
	movs r0, 6
	movs r1, 0xF
	bl MAX7219Send
	movs r0, 7
	movs r1, 0xF
	bl MAX7219Send
	pop {lr}
	bx lr
