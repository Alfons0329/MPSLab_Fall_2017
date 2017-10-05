	// ref http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.100976_0000_00_en/isc1490370287635.html
	//Q:parallel execution or sequential ?
	.syntax unified
	.cpu cortex-m4
	.thumb
.text
	.global main
	.equ RCC_BASE,0x40021000
	.equ RCC_CR,0x0
	.equ RCC_CFGR,0x08
	.equ RCC_PLLCFGR,0x0c
	.equ RCC_CCIPR,0x88
	.equ RCC_AHB2ENR,0x4C
	.equ RNG_CLK_EN,18

	// Register address for RNG (Random Number Generator)
	.equ RNG_BASE,0x50060800 //RNG BASE Address
	.equ RNG_CR_OFFSET,0x00 //RNG Control Register
	.equ RNGEN,2 // RNG_CR bit 2

	.equ RNG_SR_OFFSET,0x04 //RNG Status Register (Knowing the RNG status)
	.equ DRDY,0 // RNG_SR bit 0
	.equ RNG_DR_OFFSET,0x08 //RNG Data Register (Generated random number!)
	//Data Settings for 3.4.4
	.equ SAMPLE,1000000
	.equ INT_MAX,2147483647 //Default datatype is signed integer register,if use(2^32)
	 //it will be treated as minus since exceeded 2147483647(0x7FFFFFFF)
	//overflow to minus ,-2147483648 (cant use abs since it exceeds INT_MAX)

	.equ FOUR,4

set_flag:
	ldr r2,[r0,r1]
	orr r2,r2,r3
	str r2,[r0,r1]
	bx lr

enable_fpu:
	//Your code in 3.4.1
	//Your code start from here

	LDR.W R0, =0xE000ED88 //ldr vs ldr.w ? why ldr.w??
	//ref: https://stackoverflow.com/questions/9800526/difference-between-ldr-and-ldr-w

	LDR R1, [R0]
	ORR R1, R1, #(0xF << 20)
	STR R1, [R0]

	bx lr

enable_rng:
	ldr r0, =RNG_BASE
	ldr r1, =RNG_CR_OFFSET
	ldr r3, =4  //bit 2 is RNGEN (ref p.814 of manual)
	ldr r2,[r0]
	orr r2,r2,r3 //open it, enablt it
	str r2,[r0]
	bx lr
get_rand:
	//Your code start from here
	//read RNG_SR
	//use r2 to store value, whether it is RNG_SR or RNG_DR
	ldr r0, =RNG_BASE
	ldr r1, =RNG_SR_OFFSET //r1 gets RNG_CR_OFFSET for offset
	ldr r2, [r0,r1] //r2=0x50060800+0x04(memory) get value of RNG_SR

	and r2, r2, #1//DRDY is at the last bit (ref p.815 of manual)
	cmp r2, #1 //check DRDY bit, wait until to 1
	bne get_rand //if still not 1, branch back if non eq

	//read RNG_DR for random number and store into a register for later usage
	ldr r1, =RNG_DR_OFFSET
	ldr r2, [r0,r1] //r2=0x50060800+0x08(memory) get the value of RNG_DR
	bx lr

main:
	//RCC Settings
	ldr r0,=RCC_BASE
	ldr r1,=RCC_CR
	ldr r3,=#(1<<8) //HSION
	bl set_flag
	ldr r1,=RCC_CFGR
	ldr r3,=#(3<<24) //HSI16 selected
	bl set_flag
	ldr r1,=RCC_PLLCFGR
	ldr r3,=#(1<<24|1<<20|1<<16|10<<8|2<<0)
	bl set_flag
	ldr r1,=RCC_CCIPR
	ldr r3,=#(2<<26)
	bl set_flag
	ldr r1,=RCC_AHB2ENR
	ldr r3,=#(1<<RNG_CLK_EN)
	bl set_flag
	ldr r1,=RCC_CR
	ldr r3,=#(1<<24) //PLLON
	bl set_flag
chk_PLLON:
	ldr r2,[r0,r1]
	ands r2,r2,#(1<<25)
	beq chk_PLLON

monte_carlo_main:
	//type conversion
	//use r2 to get value of U32_MAX
	//should use s32 or it will generate error of infinite loop since if use 2^32-1 and store it in r2 but use unsigned, does not fbits
	bl enable_rng
	bl enable_fpu
	ldr r2, =INT_MAX //r2 gets INT_MAX
	/*vmov.f32 s2, r2 //mov it into the floating point type which will cause error see
	NOTE: http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0489g/Bcfhcbhb.html
	Directly use vcvt float, int will cause CE, should use the following pseudo conversion first then do real conversion*/


	/*
	register usage
	r5 as the counter of for loop ,which will eventually be the total sample point (modifiable)
	r6 as the counter of monte carlo of how many points are inside the unit circle
	in short r5 as i, r6 as inner_cnt
	*/

	vmov.f32 s2, r2

	vcvt.f32.s32 s2, s2 //convert it to f32 type
	vabs.f32 s2, s2//abs of 0x80000000(signed-->unsigned)

	vmov.f32 s7, #1.000 //since only f32 zero is allowed in vcmp,we use indirect method to compare<1.0
	//vcvt.f32.s32 s5, s5

	//counter initialization, a habit (??!)
	mov r5, #0
	mov r6, #0
	//set the sample number in register
	ldr r7, =SAMPLE

	for_loop: //for(int i=0 i<1000000 i++)

		b generating_random_numbers
		generating_random_numbers_is_finished_wow_cool_oh_yes_lololol:
		vcvt.f32.s32 s3, s3 //s3 to x but in float type
		vcvt.f32.s32 s4, s4 //s2 to y but in float type

		//Map x,y in unit range [0,1] using FPU
		//using div
		vdiv.f32 s3, s3, s2
		vdiv.f32 s4, s4, s2

		//Calculate the z=sqrt(x^2+y^2) using FPU
		vmul.f32 s3, s3, s3 //s3*=s3=>x^2
		vmul.f32 s4, s4, s4	//s4*=s4=>y^2

		vadd.f32 s5, s3, s4
		vsqrt.f32 s5, s5

		vcmp.f32 s5, s7//indirect as aforementioned
		VMRS APSR_nzcv, FPSCR //Movs the floating point status register to APSR ver
		//ref: http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0801a/CIAEHGBA.html

		it lt  //if z<1.0 ,inner_cnt++
		addlt r6, r6, #1

		add r5, r5, #1
		cmp r5, r7 //r7 as 1000000
		blt for_loop

	//for loop is finished, ready for pi calculation, pi= 4*inner_cnt/1000000
	// --> this is wrong mul r6, r6, #4 (4*inner_cnt)
	//ARM DOES NOT SUPPORT IMMEDIATE VALUE IN MULTIPLY

	movs r4, #4 //for multiplication
	mul r6, r6, r4 //4*inner_cnt

	//pseudo type conversion again as following steps
	ldr r5, =SAMPLE  //r5 get 1000000
	vmov.f32 s5, r5 //s5 get 1000000.0
	vcvt.f32.s32 s5, s5//type conversion

	vmov.f32 s6, r6 //s6 get inner count but in f32 type
	vcvt.f32.s32 s6, s6 //type conversion

	vdiv.f32 s6, s6, s5 // 1000000.0 (f32) and finally, pi=s6
	b finished

generating_random_numbers:
	//Enable FPU,RNG , converse order is not true

	//r2 stores the value of generated number
	//use r3 for x and r4 for y
	//Generate 2 random U32 number x,y
	bl get_rand
	mov r3, r2 //temp store for inspection
	vmov.f32 s3, r2  //should use vmov to stroe from normal register to floating point register
	//vmov is for pseudo type conversion , which will mentioned later
	bl get_rand
	mov r4, r2 //temp store for inspection
	vmov.f32 s4, r2  //should use vmov to stroe from normal register to floating point register
	//vmov is for pseudo type conversion , which will mentioned later

	/*bx lr cannot use this since bx lr the lr return register will just store one reg(only the recent used one),
	such machanism will cause an infinite loop from line 194 till here
	so if we need to return, we can only use b something
	to the main for loop ,b for loop back main
	*/
	b generating_random_numbers_is_finished_wow_cool_oh_yes_lololol

finished:
L: 	b L
