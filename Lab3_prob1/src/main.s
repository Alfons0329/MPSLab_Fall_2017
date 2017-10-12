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
	bl strlen
	//TODO: Setup stack pointer to end of user_stack and calculate the expression using PUSH, POP operators, and store the result into expr_result


program_end:
	B		program_end



strlen:
	//r0 now has postfix address
	//r1 for the increment i (byte addressable, asciiz a byte for char)

	push {r0} //temporarily store in stack
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
	push {r1} //now r1 has the value of how long the string is, stack be like back[r0(addr),r1(strlen)]top
	bx lr

atoi:
    //TODO: implement a “convert string to integer” function
	//back[r0(addr),r1(strlen)]top
	pop{r0-r1} //regain



    bx lr
/*
//i pass into atoi
for(int i=0;i<len;i++)
{
    if(!isnotspace(str[i]))//pass into
    {
        if(str[i]=='-')
        {
            i++;
            neg_flag=true;
        }
        if(neg_flag)
        {
            for(int cnt=1;str[i+cnt]!=' ';cnt++);
        }
        else
        {
            for(int cnt=0;str[i+cnt]!=' ';cnt++);
        }

        break; //goto next_down
    }
    break;
}
/*cnt=dist(msb,lsb) to check len of a number
ex: - 1 2 5
i ---------> cnt =3
ex: 1 2 5
i ---------> cnt=2
*/
int sum=0,pow=0;
if(neg_flag)
{
    while(cnt>=1)
    {
        sum+=str[i+cnt]*pow;
        pow*=10;
        cnt--;
    }
}
else
{
    while(cnt)
    {
        sum+=str[i+cnt]*pow;
        pow*=10;
        cnt--;
    }
}

return sum;

*/
postfix_expr_eval:
	strlen:
