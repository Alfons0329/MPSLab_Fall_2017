	.syntax unified
	.cpu cortex-m4
	.thumb

.data
	user_stack: .zero 128
	expr_result: .word 0

.text
	.global main
	postfix_expr: .asciz  "-100 10 20 + - 10 +" //result should be -120


main:
	LDR	R0, =postfix_expr
	ldr sp, =user_stack
	add sp, sp, #128 //open a stack sp, r13
	b postfix_expr_eval
	//TODO: Setup stack pointer to end of user_stack and calculate the expression using PUSH, POP operators, and store the result into expr_result


program_end:
	B		program_end



strlen:
	//r0 now has postfix address
	//r1 for the increment i (byte addressable, asciiz a byte for char)

	//push {r0} //temporarily store in stack
	mov r1, #0 //int i=0

	strlen_for_loop: // for(int i=0 arr[i]!='\0' i++)
	//expr_result array, an element possess 8 bit or say 1 byte, byte addressable
		add r2, r0, 0  //copy r0 address to r2 DONT WRITE add r0, r0, r1 or will increment permanently
		add r2, r2, r1 //extendded the address where addris now r2+r1 for get value in future
		ldrb r2, [r2] // r2=*(r2+r1)->dereference dereference a byte only

		cmp r2, #0 //empty ascii is 0
		beq strlen_quit //not

		add r1, r1, #1 //i++
		b strlen_for_loop

	b strlen_quit

strlen_quit:
	//push {r1} //now r1 has the value of how long the string is, stack be like back[r0(addr),r1(strlen)]top
	bx lr

atoi: //r7 for atoi value r8 for signed_bool_value
    //TODO: implement a “convert string to integer” function
	/*ex: 1 2 5
		i ---------> cnt=2
	    while(cnt)
		    {
		        sum+=str[i+cnt]*pow;
		        pow*=10;
		        cnt--;
		    }
		}

		return sum;

	*/
	//r9 as counter
	mov r9, #0
	for_loop_get_digit:

		mov r3, r0
		add r3, r3, r2
		add r3, r3, r9
		ldrb r3, [r3]
		cmp r3, #32
		beq atoi_get_value
		add r9, r9, #1

		b for_loop_get_digit

		atoi_done:
			cmp r8, #1
			bne ret_positive
			mov r8, #0
			sub r8, r8, #1
			mul r7, r7, r8

		ret_positive:
			push {r7}
			b getvalue_done

atoi_get_value:
	movs r4, #1 //pow
	movs r5, #10 //ten
	movs r7, #0 //sum, or say atoi value
	add r6, r2, 0 //temp store
	sub r6, r6, #1
	add r2, r2, r9//update i in i<len for iterator
	while_cnt:
		mov r3, r0
		add r3, r3, r6
		add r3, r3, r9
		ldrb r3, [r3]
		sub r3, r3, #48
		mul r3, r3, r4 //str[i+cnt]*pow;
		add r7, r7, r3 //sum+=str[i+cnt]*pow;
		mul r4, r4, r5 //pow*=10
		sub r9, r9, #1
		cmp r9, #0
		bne while_cnt
	b atoi_done

postfix_expr_eval:
	bl strlen
	/*
	r0 has the value of where the address of postfix_expr is
	r1 gets the value of string length
	r2 serve as counter i
	r3 to be used as string iterator
	r4 serves as the current value of the expression evaluation
	r5, r6 use as two operands for calculation
	*/
	//start the for_loop, now r2 is 0
	LDR	R0, =postfix_expr //regain
	mov r2, #0
	expr_eval_for_loop: //for(int i=0 i<len i++)
		mov r3, r0
		mov r8, #0 //default neg_flag=0
		add r3, r3, r2
		ldrb r3, [r3]

		cmp r3, #32
		bne is_not_space
		cmp r3, #32
		beq is_space_increment_one

		is_not_space:
			cmp r3, #48
			bge get_value

			cmp r3, #45
			beq is_negative_sign

			cmp r3, #43
			beq is_plus_operation

		is_negative_sign:
			add r2, r2, #1 //test if next is a space or not
			mov r3, r0
			add r3, r3, r2
			ldrb r3, [r3]
			sub r2, r2, #1
			cmp r3, #32
			beq is_minus_operation
			mov r8, #1 //neg_flag=1
			add r2, r2, #1
			bl atoi
			b getvalue_done

		is_plus_operation:
			pop {r4,r5}
			add r4, r4, r5
			push {r4}
			add r2, r2, #1
			cmp r2, r1
			bge all_done_store_ans_in_expr_value //end and leave

			b expr_eval_for_loop
		is_minus_operation:
			pop {r4,r5}
			sub r4, r5, r4
			push {r4}
			add r2, r2, #1
			cmp r2, r1
			bge all_done_store_ans_in_expr_value //end and leave

			b expr_eval_for_loop

		get_value:
			bl atoi
			b getvalue_done
		getvalue_done:
			cmp r2, r1
			blt expr_eval_for_loop
		is_space_increment_one:
			add r2, r2, #1
			cmp r2, r1
			blt expr_eval_for_loop

all_done_store_ans_in_expr_value:
	ldr r2, =expr_result
	str r4, [r2]
	b program_end
