.syntax unified
.cpu cortex-m4
.thumb

.data
    result: .byte 0 //1 byte = 8 bits, 1 word = 16bits
    tmp1: .word 0
    tmp2: .word 0
.text
    .global main
    .equ X, 0x5555
    .equ Y, 0xAA55

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
    //ldr R8, =X //This code will cause assemble error. Why? And how to fix.
    ldr R0, =X
    //ldr R9, =Y
    ldr R1, =Y

	ldr R2, =result
    bl hamm

	str R3, [R2]
L: b L
