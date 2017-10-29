	.syntax unified
	.cpu cortex-m4
	.thumb

.data
	student_id1: .byte 0, 4, 1, 0, 1, 3, 7 //TODO: put your student id here
	student_id2: .byte 0, 4, 1, 6, 3, 2, 4

.text
	.global main
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

main:
    BL   GPIO_init
    BL   max7219_init
    //TODO: display your student id on 7-Seg LED
    BL	Display_student_ID
    BX LR

Display_student_ID:
	mov r0, 0x8 //init digit = 8
	mov r2, 0x0 //arr index
	ldr r3, =student_id1
display_loop:
	subs r0, r0, 1 //digit -1
	ldrb r1, [r3,r2] //student_id1[r2]
	bl MAX7219Send
	adds r2, r2, 1 //arr index +1
	cmp r0, 1 //digit == 1
	bne display_loop
	b Display_student_ID

GPIO_init:
	//TODO: Initialize three GPIO pins as output for max7219 DIN, CS and CLK
	//RCC_AHB2ENR: enable GPIOA
	mov r0, 0b1
	ldr r1, =RCC_AHB2ENR
	str r0, [r1]

	//GPIOA_MODER: PA2,1,0: output
	ldr r0, =0b010101
	lsl r0, 10
	ldr r1, =GPIOA_MODER
	ldr r2, [r1]
	and r2, 0xFFFF03FF //clear 2 1 0
	orrs r2, r2, r0 //2 1 0 --> output
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

	BX LR

MAX7219Send:
//input parameter: r0 is ADDRESS , r1 is DATA
	//TODO: Use this function to send a message to max7219
	push {r0, r1, r2, r3, r4, r5, r6, r7, LR}
	lsl	r0, 8 //move to D15-D8
	add r0, r1 //r0 == din
	ldr r1, =DIN
	ldr r2, =CS
	ldr r3, =CLK
	ldr r4, =GPIOA_BSRR //-> 1
	ldr r5, =GPIOA_BRR //-> 0
	ldr r6, =0xF //now sending (r6)-th bit
	//b send_loop

send_loop:
	mov r7, 1
	lsl r7, r6
	str r3, [r5] //CLK -> 0
	tst r0, r7 //same as ANDS but discard the result (just update condition flags)
	beq bit_not_set //the sending bit(r0) != 1
	str r1, [r4] //din -> 1
	b if_done

bit_not_set: //send clear bit
	str r1, [r5] //din -> 0

if_done:
	str r3, [r4] //CLK -> 1
	subs r6, 0x1
	bge send_loop
	str r2, [r5] //CS -> 0
	str r2, [r4] //CS -> 1
	pop {r0, r1, r2, r3, r4, r5, r6, r7, PC}
	BX LR

max7219_init:
	//TODO: Initialize max7219 registers
	push {r0, r1, LR}
	ldr r0, =DECODE
	ldr r1, =0xFF //CODE B decode for digit 0-7
	bl MAX7219Send

	ldr r0, =DISPLAY_TEST
	ldr r1, =0x0 //normal operation
	bl MAX7219Send

	ldr r0, =INTENSITY
	ldr r1, =0xA //brightness (21/32)
	bl MAX7219Send

	ldr r0, =SCAN_LIMIT
	ldr r1, =0x6 //light up digit 0-6
	bl MAX7219Send

	ldr r0, =SHUT_DOWN
	ldr r1, =0x1 //normal operation
	bl MAX7219Send

	pop {r0, r1, PC}
	BX LR
