#include <stdio.h>
#include <stdlib.h>
#define ID_LEN 7
extern void GPIO_init();
extern void max7219_init();
extern void max7219_send(unsigned char address, unsigned char data);
int display(int data, int num_digs)
{
    //getting the value from LSB to MSB which is right to left
    //7 seg panel from 1 to 7 (not zero base)
    int i=0,dig;
    for(i=1;i<=num_digs;i++)
    {
        max7219_send(i,data%10);
        dig=data%10;
        data/=10; //get the next digit
        if(i==num_digs)
        {
        	max7219_send(i,0); //send 0
        }
    }
    if(data>99999999 || data<-9999999)
        return -1; //out of range error
    else
        return 0; //end this function
}
int display_clr(int num_digs)
{
	for(int i=1;i<=num_digs;i++)
	{
		max7219_send(i,0xF);
	}
	return 0;
}
int main()
{
    int student_id = 416324;
    GPIO_init();
    max7219_init();
    display_clr(8);
    display(student_id, 7);
    return 0;
}
