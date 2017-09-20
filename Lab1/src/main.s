	.syntax unified
	.cpu cortex-m4
	.thumb
.text
	.global main
	.equ AA, 0x55 // #define AA=0x55(macro definition in hex)
main:
	movs r0, #AA //assign the var AA to r0(register 0) which is 85_dec
	movs r1, #20 //assign the 20_dec to r1
	adds r2, r0, r1 //add up and save into r2
L: B L //L as a label and back to L to serve as a halt state
