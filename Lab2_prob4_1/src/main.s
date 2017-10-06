		.syntax unified
	.cpu cortex-m4
	.thumb
.data
	x: .float 0.123
	y: .float 0.456
	z: .word 20
.text
	.global main

enable_fpu:
	//Your code start from here

	/*LDR.W R0, =0xE000ED88

	LDR R1, [R0]
	ORR R1, R1, #(0xF << 20)
	STR R1, [R0]
	bx lr*/

main:
	bl enable_fpu
	ldr r0,=x
	vldr.f32 s0,[r0]
	ldr r0,=y
	vldr s1,[r0]
	vadd.f32 s2,s0,s1
	// Your code start from here
	//Calculate the following values using FPU instructions
	//and show the register result in your report


    //r0 as x, and r1 as y
	// s2=x-y
	vsub.f32 s2, s0, s1
	// s2=x*y
	vmul.f32 s2, s0, s1
	// s2=x/y
	vdiv.f32 s2, s0, s1
	// load z into r0,
	ldr r0,=z
	// copy z from r0 to s2,
	vldr.f32 s2, [r0]

	//u32 a word(4bytes), actually uint32_t (POSIX Standard)
	// convert z from U32 to float representation F32 in s2
	//conversion reference to: http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0489c/CIHFFGJG.html
	//usage VCVT{cond}.type Dd, Dm {, #fbits}
	//now as a self-type conversion
	vcvt.f32.u32 s2, s2 //(from right src to left dst)
	// calculate s3=z+x+y (s2+s0+s1)
	//s3=x+y
	vadd.f32 s3, s0, s1
	//s3+=z
	vadd.f32 s3, s3, s2
L: b L
