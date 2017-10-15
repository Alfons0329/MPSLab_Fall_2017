	.syntax unified
	.cpu cortex-m4
	.thumb
.data
	result: .word 0
	max_size: .word 0

.text
	m: .word 0x5E //94
	n: .word 0x60 //96
	.global main

main:
	ldr r0, m
	ldr r1, n
	push {r0, r1} //sp+4=r0(m), sp=r1(n)
	movs r6, #0 //how many times should we << (both even)
	movs r8, #0 //initialize recursive as zero
	bl GCD
	//r7=result, r8=max_size
	lsl r7, r6
	ldr r0, =result
	ldr r1, =max_size
	str r7, [r0]
	str r8, [r1]

L: B L

GCD:
	//TODO: Implement your GCD function
	ldr r2, [sp, #4] //r2=m
	ldr r3, [sp] //r3=n
	pop {r1, r0} //r1=n, r0=m
	push {lr}

	//m==n
	cmp r2, r3
	beq re_m
	//m==0
	cmp r2, #0
	beq re_n
	//n==0
	cmp r3, #0
	beq re_m

	//m is even
	and r4, r2, #1
	cmp r4, #0
	beq m_even
	//n is even and m is odd
	and r4, r3, #1
	cmp r4, #0
	beq n_even

	//both odd
	//case1: if m>n
	cmp r2, r3
	bgt m_greater
	blt n_greater

	re_m:
		movs r7, r2
		//movs r8, #0
		bx lr
	re_n:
		movs r7, r3
		//movs r8, #0
		bx lr

	m_even:
		and r4, r3, #1
		cmp r4, #0
		beq both_even
		//gcd(m>>1,n)
		lsr r2, #1 //m>>1
		b recursive

	both_even:
		//gcd(m>>1, n>>1)<<1
		lsr r2, #1 //m>>1
		lsr r3, #1 //n>>1
		add r6, r6, #1 //<< n times
		b recursive

	n_even:
		//gcd(m,n>>1)
		lsr r3, #1
		b recursive

	m_greater:
		//gcd((m-n)>>1,n)
		sub r2, r2, r3 //m-n
		lsr r2, #1 //(m-n)>>1
		b recursive
	n_greater:
		//gcd((n-m)>>1,m)==gcd(m,(n-m)>>1)
		sub r3, r3, r2 //n-m
		lsr r3, #1 //(n-m)>>1
		b recursive

	recursive:
		//store m, n back to r0, r1
		movs r0, r2
		movs r1, r3
		push {r0, r1} //push to stk just like first call GCD in main
		bl GCD
		add r8, r8, #1
		pop {lr} //end a recursive loop then pop out lr
		bx lr


