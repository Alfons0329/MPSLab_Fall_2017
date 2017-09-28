
.text
    .global main
    .equ N, 20

fib:
    //TODO
    //check if N is in the range, let r4= -1 for OUT_OF_RANGE


    cmp_greater_than_1:
        cmp r0, #1
        bge cmp_less_than100
        //if OUT_OF_RANGE
        movs r4, #0
        subs r4, r4, 1
        b return

    cmp_less_than100:
        cmp r0, #100
        ble fibonacci_main
        //if OUT_OF_RANGE
        movs r4, #0
        subs r4, r4, 1
        b return


    fibonacci_main:

        movs r4, #1 //fir
        movs r1, #1 //sec
        movs r2, #2 //third

        cmp r0, #1
        beq return
        cmp r0, #2
        beq return

        movs r5, #1 //fibonacci counnter


        for_loop:
            adds r2, r1, r4  //third=fir+sec
            r4=r1 //fir=sec
            r1=r2//sec=third
            adds r5, r5, #1

            bvs overflow_return
            
            cmp r5, r0
            blt for_loop

    return:
        bx lr

    overflow_return:
        movs r4, #0
        subs r4, r4, 2
        bx lr
main:
    movs R0, #N
    bl fib
L: b L
