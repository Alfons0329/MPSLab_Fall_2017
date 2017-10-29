.syntax unified
.cpu cortex-m4
.thumb

.data
    student_id1: .byte 0, 4, 1, 0, 1, 3, 7 //TODO: put your student id here
    student_id2: .byte 0, 4, 1, 6, 3, 2, 4
    fib_ans: .asciz "01123581321345589144233377610987159725844181676510946177112865746368750251213931964183178115142298320401346269217830935245785702887922746514930352241578173908816963245986:1"
    ans_digit: .byte 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x2, 0x2, 0x2, 0x2, 0x2, 0x3, 0x3, 0x3, 0x3, 0x3, 0x4, 0x4, 0x4, 0x4, 0x5, 0x5, 0x5, 0x5, 0x5, 0x6, 0x6, 0x6, 0x6, 0x6, 0x7, 0x7, 0x7, 0x7, 0x7, 0x8, 0x8, 0x8, 0x8, 0x2

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
main:
    BL   GPIO_init
    BL   max7219_init
	mov r9, 0x0
    mov r12, 0x0
    BL	Display_fibo_number
    BX LR
//use r0 for digit of current number, in ans_digit, or the accumulation in fibonacci
//use r1 for get data in fibonacci array

//use r3 for the address of fibonacci array
//use r4 for get the current digit
//use r5 to get the button data
//use r6 for press flag
//use r9 for the extend address of current fibo MSB
//use r8 to get data from button
//use r10 as current digit counter
//use r11 for accumulation the pointer in fibonacci array, current MSB starting point
//use r12 for debouncing OMG!! SO MANY REGISTERS
Display_fibo_number:
    mov r10, 0x0

    ldr r3, =fib_ans
    ldr r0, =ans_digit
    ldrb r0, [r0,r4] //get current fibonacci digit this r0 will decrease
    adds r0, r0, 1
display_loop:
    subs r0, r0, 1

	mov r9, r7 //keep oroginal position
    adds r9, r9, r10
    ldrb r1, [r3,r9] //fibo_ans[r9+r10] ex: 144 then r9 at 1 r10 in [0,2] to get 1, 4 and 4 from MSB to LSB
    sub r1, r1, #48

    push {r0}
    mov r12, r12

    b check_button
    check_end:

    pop {r0}
    is_reset:
    bl MAX7219Send

    adds r10, r10, 1 //arr idex +1
    cmp r0, 1
    bne display_loop

    b Display_fibo_number

check_button: //check every cycle, and accumulate 1
    ldr r5, [r8] //fetch the data from button
    lsr r5, r5, #13
    and r5, r5, 0x1 //filter the signal

    cmp r5, #0 //FUCK DONT KNOW WHY THE PRESSED SIGNAL IS 0
    it eq
    addseq r12, r12 ,#1 //accumulate until the threshol

    cmp r5, #1 //not stable, go back to accumulate again
    it eq
    moveq r12, #0

	ldr r11, =ans_digit

    push {r10}
    ldr r10, =point_one_sec //trial and error
    cmp r12, r10 //threshold achieved BREAKDOWN!, r6 flag rises use 1 for slo mo debug
	it eq
	movseq r6, #1
    pop {r10}

    cmp r6, #1
    it eq
    ldrbeq r11, [r11,r4] //get current fibonacci digit this r0 will decrease

    cmp r6, #1
    it eq
    addeq r4, r4, 0x1//go to next fibonacci digit

    cmp r6, #1
    it eq
    addeq r7, r7, r11 //move to the start of next fibonacci digit, by increment the digit of current fibonacci number

    cmp r4, #40
    it ge
    movsge r4, #40

    cmp r4, #40
    it ge
    movsge r7, #170 //for -1

	mov r6, #0

	push {r10}
    ldr r10, =one_sec
    cmp r12, r10
    bge clear_to_zero
    pop {r10}

    b check_end
clear_to_zero:
    mov r1,0x0
    mov r0,0x1
    b is_reset
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

    ldr r0, =INTENSITY
    ldr r1, =0xA //�G�� 21/32
    bl MAX7219Send

    ldr r0, =SCAN_LIMIT
    ldr r1, =0x7 //light up digit 0-7
    bl MAX7219Send

    ldr r0, =SHUT_DOWN
    ldr r1, =0x1 //normal operation
    bl MAX7219Send

    pop {r0, r1, PC}
    BX LR
