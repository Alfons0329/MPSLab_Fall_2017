/*bubble sort in c
void bubble_sort(int arr[], int len) {
	int i, j, temp;
	for (i = 0; i < len - 1; i++)
		for (j = 0; j < len - 1 - i; j++)
			if (arr[j] > arr[j + 1]) {
				temp = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = temp;
			}
}
*/

    .syntax unified
    .cpu cortex-m4
    .thumb

.data
    arr1: .byte 0x19, 0x34, 0x14, 0x32, 0x52, 0x23, 0x61, 0x29
    arr2: .byte 0x18, 0x17, 0x33, 0x16, 0xFA, 0x20, 0x55, 0xAC
.text
    .global main
do_sort:
    //TODO arr from 0 to 7

    movs r3, #7 //i<len-1-i part
    movs r1, #0 //int i=0
    for_loop_outer: //i=[0,6]

    movs r2, #0 //int j=0
    sub r9, r3, r1//len-1-i
        for_loop_inner:
            add r4, r0, r2 //offset in byte get arr+j address, store in r4
            ldrb r5, [r4]//dereference to get value r5 as arr[j]

            add r6, r4, #1 //offset in byte get arr+j address ,store in r6
            ldrb r7, [r6]

            cmp r5,r7
            bgt swap
            swap_is_done:
            //j++ then j<len-1-i
            add r2, r2, #1
             //r9 as len-1-i r9 = r3(7 which is len-1 )- r1 (which is i)
            cmp r2,r9
            blt for_loop_inner

    //i++ then i< len -1
    add r1, r1, #1
    cmp r1, #7
    blt for_loop_outer

    b return //job is done

    swap:  //using r8 as temp value for swapping use STRB for storing in memory
        movs r8, r5
        movs r5, r7
        movs r7, r8

        //store the fucking value back
        strb r5, [r4]
        strb r7, [r6]
        b swap_is_done


	return:
    	bx lr

//swap in this place will cause the fucking error of back to the main function, or add swap_is_done tag

main:
    ldr r0, =arr1
    bl do_sort
    ldr r0, =arr2
    bl do_sort
L: b L
