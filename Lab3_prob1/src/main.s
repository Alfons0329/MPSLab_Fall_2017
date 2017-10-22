	.syntax unified
	.cpu cortex-m4
	.thumb

.data
	user_stack: .zero 128
	expr_result: .word 0

.text
	.global main
	postfix_expr: .asciz  "70 0 -1 + 50 + -" //result should be -120
	.align 4

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

	strlen_quit:
	bx lr

atoi: //r7 for atoi value r8 for signed_bool_value
    //TODO: implement a �onvert string to integer�� function

	//r9 as counter
	mov r9, #0
	for_loop_get_digit:
		/*r3 as base addr, r6 as i where integer starts and r9 as cnt, count how many digit of such integer*/
		mov r3, r0
		add r3, r3, r2
		add r3, r3, r9
		ldrb r3, [r3]
		cmp r3, #32 //ex: 125_ now see the white space after 5, then we have cnt=9 (3-digit num)
		//the digit is guranteed, go get the value of number
		beq atoi_get_value
		add r9, r9, #1

		b for_loop_get_digit //if not reach _ after 5, still loop to ensure the digit of number is right

	atoi_done:
		cmp r8, #1 //check if this integer is negative, neg_flag at r8
		bne ret_positive
		mov r8, #0 //make atoi value negative
		sub r8, r8, #1
		mul r7, r7, r8

	ret_positive:
		push {r7} //push the value into stack
		b getvalue_done

atoi_get_value:
/*
	PESUDO CODE OF GET VALUE
	ex: 1 2 5
	i ---------> cnt=3
	while(cnt)
		{
			sum+=str[i+cnt]*pow;
			pow*=10;
			cnt--;
		}
	}

	return sum;

*/
	movs r4, #1 //pow
	movs r5, #10 //ten
	movs r7, #0 //sum, or say atoi value
	add r6, r2, 0 //temp store i, where the integer starts
	sub r6, r6, #1 //ex. 125_ after get digit r9 will be 3 but the argument point possess the address of first integer
	//is at 1 rather than the space before 1,  using 3 will get the space, thus minus 1
	add r2, r2, r9//update i in i<len for iterator, now r2, or say i stop at next space
	while_cnt:
		/*r3 as base addr, r6 as i where integer starts and r9 as cnt, increment from lsb to msb*/
		mov r3, r0 //get base addr
		add r3, r3, r6 // r2 is where the pointer points to integer
		add r3, r3, r9 // r3-1+r9 , r9 is 3 r3 points to1 so now r3 points to 5 of 125_
		ldrb r3, [r3] //r3=*(r3-1+r9)
		sub r3, r3, #48 // -'0'
		mul r3, r3, r4 //str[i+cnt]*pow;
		add r7, r7, r3 //sum+=str[i+cnt]*pow;
		mul r4, r4, r5 //pow*=10
		sub r9, r9, #1 //increment bit
		cmp r9, #0
		bne while_cnt
	b atoi_done //quit atoi

postfix_expr_eval:
	bl strlen
	/*
	r0 has the value of where the address of postfix_expr is
	r1 gets the value of string length
	r2 serve as counter i
	r3 to be used as string iterator
	r4 serves as the current value of the expression evaluation
	r4, r5 use as two operands for calculation (pop and calculate)
	*/
	//start the for_loop, now r2 is 0
	LDR	R0, =postfix_expr //regain
	mov r2, #0
	expr_eval_for_loop: //for(int i=0 i<len i++)
		mov r3, r0
		mov r8, #0 //default neg_flag=0
		add r3, r3, r2
		ldrb r3, [r3]
		//filtering the char
		cmp r3, #32
		bne is_not_space
		cmp r3, #32
		beq is_space_increment_one

		is_not_space:
			cmp r3, #48 //is a integer , ASCII>48
			bge get_value

			cmp r3, #45 //is a negative sign(but not sure if is an opt or sign)
			beq is_negative_sign

			cmp r3, #43 //is plus operation
			beq is_plus_operation

		is_negative_sign:
			add r2, r2, #1 //test if next is a space or not
			mov r3, r0 //get base addr
			add r3, r3, r2 //test the next char after
			ldrb r3, [r3]
			sub r2, r2, #1
			cmp r3, #32
			beq is_minus_operation
			cmp r3, #0 //if reach end, still evaluate last minus operator,
			//the case is minus at last position before the 0\
			beq is_minus_operation //still go evaluate

			mov r8, #1 //neg_flag=1
			add r2, r2, #1 //back to the minus signed position, where before the integer starts, and go get value
			bl atoi //go get value
			b getvalue_done

		is_plus_operation:
			pop {r4,r5}
			add r4, r4, r5
			push {r4}
			add r2, r2, #1
			cmp r2, r1
			bge all_done_store_ans_in_expr_value //end and leave

			b expr_eval_for_loop //still in expr, keep going

		is_minus_operation:
			pop {r4,r5}
			sub r4, r5, r4
			push {r4}
			add r2, r2, #1
			cmp r2, r1
			bge all_done_store_ans_in_expr_value //end and leave

			b expr_eval_for_loop //still in expr, keep going

		get_value:
			bl atoi
			b getvalue_done

		getvalue_done:
			cmp r2, r1 //value will definitely in the expr since it is postfix expression
			blt expr_eval_for_loop

		is_space_increment_one: //encounter a space, just move to next char for tokenizing
			add r2, r2, #1
			cmp r2, r1
			blt expr_eval_for_loop

all_done_store_ans_in_expr_value:
	ldr r2, =expr_result
	str r4, [r2]
	b program_end
