.syntax unified
.cpu cortex-m4
.thumb

.data

    hamm_ans: .asciz "012345678910111213141516"
    ans_digit: .byte 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x2, 0x2, 0x2, 0x2, 0x2, 0x2


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
        .equ	DECODE,			0x19 //�ѽX����
        .equ	INTENSITY,		0x1A //�G�ױ���
        .equ	SCAN_LIMIT,		0x1B //�]�w���ܽd��
        .equ	SHUT_DOWN,		0x1C //����
        .equ	DISPLAY_TEST,	0x1F //���ܴ���

        //button
        /*GPIOC for button*/
        .equ GPIOC_MODER  , 0x48000800
        .equ GPIOC_OTYPER ,	0x48000804
        .equ GPIOC_OSPEEDR,	0x48000808
        .equ GPIOC_PUPDR  ,	0x4800080c
        .equ GPIOC_IDR    , 0x48000810

        //
        .equ one_sec, 10000
        .equ point_one_sec, 1000

        .equ X, 0xabcd
    	.equ Y, 0xdcba
hamm:
    //TODO
    eor R0, R0, R1
    add R4, R0, #0

    whileloop:
    	cmp R4, #0
        beq return
    	and R5, R4, #1 //R5 for increment value
		add R3, R3, R5

    	lsr R4 ,R4 ,#1
    	b whileloop

    //vcnt result, R4
    return:
    bx lr
main:
    BL   GPIO_init
    BL   max7219_init
    //ldr R8, =X //This code will cause assemble error. Why? And how to fix.
    ldr R0, =X
    //ldr R9, =Y
    ldr R1, =Y
    bl hamm
	mov r12, r3 // r3 for the result
Display_hamm_ans:
    //mov r10, 0x0

    ldr r3, =hamm_ans
    //ldr r0, =ans_digit

    //push {r0}

    cmp r12, 10
    bge twodigit
    ldr r1, =0x0
    b nottwodigit

    twodigit:
    ldr r1, =0x1
    nottwodigit:

    ldr r0, =SCAN_LIMIT
    bl MAX7219Send

    //pop {r0}

    cmp r12, 0
    it eq
    moveq r4, 0

    cmp r12, 1
    it eq
    moveq r4, 1

    cmp r12, 2
    it eq
    moveq r4, 2

    cmp r12, 3
    it eq
    moveq r4, 3

    cmp r12, 4
    it eq
    moveq r4, 4

    cmp r12, 5
    it eq
    moveq r4, 5

    cmp r12, 6
    it eq
    moveq r4, 6

    cmp r12, 7
    it eq
    moveq r4, 7

    cmp r12, 8
    it eq
    moveq r4, 8

    cmp r12, 9
    it eq
    moveq r4, 9

    cmp r12, 10
    it eq
    moveq r4, 10

    cmp r12, 11
    it eq
    moveq r4, 12

    cmp r12, 12
    it eq
    moveq r4, 14

    cmp r12, 13
    it eq
    moveq r4, 16

    cmp r12, 14
    it eq
    moveq r4, 18

    cmp r12, 15
    it eq
    moveq r4, 20

    cmp r12, 16
    it eq
    moveq r4, 22

    cmp r12, 10
    bge twodigits
    ldr r0, =0x1
    b nottwodigits

    twodigits:
    ldr r0, =0x2
    nottwodigits:

    adds r0, r0, 1

display_loop:
    subs r0, r0, 1
    ldrb r1, [r3,r4] //fibo_ans[r9+r10] ex: 144 then r9 at 1 r10 in [0,2] to get 1, 4 and 4 from MSB to LSB
    sub r1, r1, #48

    bl MAX7219Send

    adds r4, r4, 1 //arr idex +1
    cmp r0, 1
    bne display_loop

    b Display_hamm_ans
GPIO_init:
    //TODO: Initialize three GPIO pins as output for max7219 DIN, CS and CLK
    //RCC_AHB2ENR: enable GPIOA and GPIOC
    mov r0, 0b101
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


    //enable the port c GPIOC_MODER for input mode
	ldr r0, =GPIOC_MODER
	ldr r1, [r0]
	//clear pc13 to zero
	and r1, r1, 0xf3ffffff

	str r1,	[r0]

	//otype is default to pp , no need to change

	//usage r4 for button data input value address in the future

	ldr r8, =GPIOC_IDR
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
    lsl r7, r6 //left shift to get the data in current digit
    str r3, [r5] //CLK -> 0
    tst r0, r7 //同ANDS但不存結果 (update condition flags)
    beq bit_not_set //r0要送的那位!=1
    str r1, [r4] //din -> 1
    b if_done

bit_not_set: //send clear bit
    str r1, [r5] //din -> 0

if_done:
    str r3, [r4] //CLK -> 1 r3 use as clock
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

    ldr r0, =SCAN_LIMIT
    ldr r1, =0x0 //dynamically light up the digit
    bl MAX7219Send

    ldr r0, =INTENSITY
    ldr r1, =0xA //�G�� 21/32
    bl MAX7219Send


    ldr r0, =SHUT_DOWN
    ldr r1, =0x1 //normal operation
    bl MAX7219Send

    pop {r0, r1, PC}
    BX LR
