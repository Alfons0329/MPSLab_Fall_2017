.syntax unified
   .cpu cortex-m4
   .thumb
.data  //initilaize (gloabl variable)
   X: .word 5
   Y: .word 10
   Z: .word 0
.text //(read only) to be put in, to tell the compiler do such things, define like
   .global main
   .equ AA, 0x05
   .equ BB, 0x0A

main:

    movs r0, #BB     //r0 = 10
    ldr r1, =X
    ldr r2, [r1]     //r2 = x, load from register

    muls r0, r2 , r0   //r0 = r2 * 10

    ldr r4, =Y
    ldr r3, [r4]     //r3 = y

    adds r0 , r0 , r3  //r0 = r0 + r3

    str r0, [r1]     // x = x * 10 + y

    ldr r5, =Z
    ldr r3, [r4]
    ldr r2, [r1]

    subs r6 , r3 , r2

    str r6,[r5]

L: B L
