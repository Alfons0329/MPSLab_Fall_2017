	.syntax unified
	.cpu cortex-m4
	.thumb
.data
    result: .byte 0 //1 byte = 8 bits, 1 word = 16bits
    tmp1: .word 0
    tmp2: .word 0
.text
    .global main
    .equ X, 0x1000 //0x55AA
    .equ Y, 0x3F //0xAA55

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
    movs R0, #X //This code will cause assemble error. Why? And how to fix.
    movs R1, #Y
    ldr R2, =result
    ldr R3, [R2]
    bl hamm
L: b L
