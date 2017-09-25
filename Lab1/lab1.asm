---------------------------------------
-------         PART1           -------
---------------------------------------
	.syntax unified
	.cpu cortex-m4
	.thumb
.text
	.global main
	.equ AA, 0x55

main:
	movs r0, #AA
	movs r1, #20
	adds r2, r0, r1
L:	B L

---------------------------------------
-------         PART2           -------
---------------------------------------
	.syntax unified
	.cpu cortex-m4
	.thumb

.data
	X: .word 100
	str: .asciz "Hello World!"

.text
	.global main
	.equ AA, 0x55

main:
	ldr r1, =X
	ldr r0, [r1]
	movs r2, #AA
	adds r2, r2, r0
	str r2, [r1]

	ldr r1, =str
	ldr r2, [r1]
L:	B L

---------------------------------------
-------         PART3           -------
---------------------------------------
	.syntax unified
	.cpu cortex-m4
	.thumb

.data
	X: .word 5
	Y: .word 10
	Z: .word 0

.text
	.global main

main:
	ldr r0, =X
	ldr r1, [r0]
	ldr r2, =Y
	ldr r3, [r2]
	ldr r4, =Z
	ldr r5, [r4]

	movs r6, #10

	muls r1, r1, r6
	adds r1, r1, r3
	str r1, [r0]

	subs r5, r3, r1
	str r5, [r4]
L:	B L
