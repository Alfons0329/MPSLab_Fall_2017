    .syntax unified
    .cpu cortex-m4
    .thumb

.data


.text
    .global max7219_send
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

max7219_send:
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
