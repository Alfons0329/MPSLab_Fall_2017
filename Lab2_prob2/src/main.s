	.syntax unified
	.cpu cortex-m4
	.thumb

.text
    .global main
    .equ N, 47

fib:
    //TODO
    //check if N is in the range, let r4= -1 for OUT_OF_RANGE


    cmp_greater_than_1:
        cmp r0, #1
        bge cmp_less_than100
        //if OUT_OF_RANGE
        movs r4, #0
        sub r4, r4, #1
        b return

    cmp_less_than100:
        cmp r0, #100
        ble fibonacci_main
        //if OUT_OF_RANGE
        movs r4, #0
        sub r4, r4, 1
        b return //br for better manipulation??


    fibonacci_main:

        movs r4, #1 //first prototype for special-testcase judge

        cmp r0, #1 //fibonacci f(1)=1 --> 1 1 2 3 5 8 13...
        beq return
        cmp r0, #2
        beq return

		movs r1, #1 //first
        movs r2, #1 //second
		movs r4, #0 //fib(n)

		movs r5, #2 //fibonacci counnter start at 2 (modification for overflow detection)

        for_loop:
            adds r4, r1, r2  //third=fir+sec adds will update the flag!! FOR SURE
            movs r1, r2 //fir=sec
            movs r2, r4//sec=third

            add r5, r5, #1//increment the counter by 1

            bvs overflow_return //f(48)will cause overflow in 32bit integer

            cmp r5, r0 //compare if it is still in the fib range
            blt for_loop//back to loop again

    return:
        bx lr

    overflow_return:
        movs r4, #0
        subs r4, r4, #2
        bx lr
main:
    movs R0, #N
    bl fib
L: b L
