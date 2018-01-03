.syntax unified
.cpu cortex-m4
.thumb
.global GPIO_init
.global max7219_init
.global max7219_send
.global delay_1s
.global FPU_init
.equ RCC_AHB2ENR, 0x4002104C
.equ GPIOA_MODER, 0x48000000
.equ GPIOA_OTYPER, 0x48000004
.equ GPIOA_OSPEEDR, 0x48000008
.equ GPIOB_MODER, 0x48000400
.equ GPIOB_OTYPER, 0x48000404
.equ GPIOB_OSPEEDR, 0x48000408
.equ GPIOB_PUPDR, 0x4800040C
.equ GPIOC_MODER, 0x48000800
.func GPIO_init
GPIO_init:
//PA4.5.6.7 & PB10
	//Enable GPIOA.B.C clock
	movs r0, #0b111
	ldr r1, =RCC_AHB2ENR
	str r0, [r1]
	//Set PA0.1.4~9 as output mode(01)
	ldr r0, =#0x55505
	ldr r1, =GPIOA_MODER
	ldr r2, [r1]
	ldr r3, =#0xFFF000F0
	and r2, r3		//Mask MODER
	orrs r2, r2, r0
	str r2, [r1]
	//Default PA is Pull-up output, no need to set
	//Set PA0.1.4~9 as medium speed mode(01)
	ldr r0, =#0x55505
	ldr r1, =GPIOA_OSPEEDR
	strh r0, [r1]
	//Set PB3~6 as output mode(01)
	ldr r0, =#0x1540
	ldr r1, =GPIOB_MODER
	ldr r2, [r1]
	ldr r3, =#0xFFFFC03F
	and r2, r3		//Mask MODER
	orrs r2, r2, r0
	str r2, [r1]
	//Default PB is Pull-up output, no need to set
	//Set PB3~6 as medium speed mode(01)
	ldr r0, =#0x1540
	ldr r1, =GPIOB_OSPEEDR
	strh r0, [r1]
	//Set PB10 as output mode(01)
	movs r0, #0x100000
	ldr r1, =GPIOB_MODER
	ldr r2, [r1]
	and r2, #0xFFCFFFFF		//Mask MODER10
	orrs r2, r2, r0
	str r2, [r1]
	// PB Pull-up output(01)
	movs r0, #0x100000
	ldr r1, =GPIOB_PUPDR
	ldr r2, [r1]
	and r2, #0xFFCFFFFF		//Mask MODER10
	orrs r2, r2, r0
	str r2, [r1]
	//Set PB10 as high speed mode(10)
	movs r0, #0x200000
	ldr r1, =GPIOB_OSPEEDR
	strh r0, [r1]
	//Set GPIOC Pin13 as input mode(00)
	ldr r1, =GPIOC_MODER
	ldr r0, =#0xF3FFFFFF
	str r0, [r1]
	BX lr
.endfunc

.equ DECODE_MODE, 0x09
.equ DISPLAY_TEST, 0x0F
.equ SCAN_LIMIT, 0x0B
.equ INTENSITY, 0x0A
.equ SHUTDOWN, 0x0C
.func max7219_init
max7219_init:
	push {r0, r1, r2, lr}
	ldr r0, =#DECODE_MODE
	ldr r1, =#0xFF
	BL max7219_send
	ldr r0, =#DISPLAY_TEST
	ldr r1, =#0x0
	BL max7219_send
	ldr r0, =#SCAN_LIMIT
	ldr r1, =#0x7
	BL max7219_send
	ldr r0, =#INTENSITY
	ldr r1, =#0xA
	BL max7219_send
	ldr r0, =#SHUTDOWN
	ldr r1, =#0x1
	BL max7219_send
	pop {r0, r1, r2, pc}
.endfunc

.equ GPIOA_BASE, 0x48000000
.equ DATA, 0x20 //PA5
.equ LOAD, 0x40 //PA6
.equ CLOCK, 0x80 //PA7
.equ BSRR, 0x18
.equ BRR, 0x28
.func max7219_send
//input parameter: r0 is address, r1 is data
max7219_send:
	push {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, lr}
	lsl r0, r0, #8
	add r0, r0, r1
	ldr r1, =#GPIOA_BASE
	ldr r2, =#LOAD
	ldr r3, =#DATA
	ldr r4, =#CLOCK
	ldr r5, =#BSRR
	ldr r6, =#BRR
	mov r7, #16			//r7 = i
.max7219send_loop:
	mov r8, #1
	sub r9, r7, #1
	lsl r8, r8, r9		//r8 = mask
	str r4, [r1,r6]		//CLOCK = 0
	tst r0, r8
	beq .bit_not_set
	str r3, [r1,r5]
	b .if_done
.bit_not_set:
	str r3, [r1,r6]
.if_done:
	str r4, [r1,r5]		//CLOCK = 1
	subs r7, #1
	bgt .max7219send_loop
	str r2, [r1,r6]		//CS = 1
	str r2, [r1,r5]		//CS = 0
	pop {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, pc}
.endfunc

.equ GPIOC_IDR, 0x48000810
.equ X, 51
.equ Y, 7041
.func delay_1s
delay_1s:
	ldr r3, =X
D1:
	ldr r4, =Y
D2:
	ldr r1, =GPIOC_IDR
	ldr r0, [r1]
	movs r2, #1
	lsl r2, #13
	ands r0, r2
	beq .break		// if 1 ignore button
	subs r4, #1
	bne D2
	subs r3, #1
	bne D1
.break:
	bx lr
.endfunc

FPU_init:
	push {r0, r1, lr}
	LDR r0, =0xE000ED88
	LDR r1, [R0]
	ORR r1, r1, #(0xF << 20)
	STR r1, [r0];
	pop {r0, r1, pc}
