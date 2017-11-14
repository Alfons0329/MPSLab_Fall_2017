//These functions inside the asm file
#include <stdio.h>
#include <stdlib.h>
//#include "stm32l476xx.h"
//#include "core_cm4.h"
#define ID_LEN 7
extern void GPIO_init();
extern void max7219_init();
extern void max7219_send(unsigned char address, unsigned char data);
/**
* TODO: Show data on 7-seg via max7219_send
* Input:
*
data: decimal value
*
num_digs: number of digits will show on 7-seg
* Return:
*
0: success
*
-1: illegal data range(out of 8 digits range)
*/
int display(int data, int num_digs)
{
    //getting the value from LSB to MSB which is right to left
    //7 segpanel from 1 to 7 (not zero base)
    int i=0,dig;
    for(i=1;i<=num_digs;i++)
    {
        max7219_send(i,data%10);
        dig=data%10;
        //printf("data is now %d and send %d ",data,dig);
        data/=10; //get the next digit 
        if(i==num_digs)
        {
        	max7219_send(i,0); //send 0
        }
    }
    for(i=1;i<=8;i++)
    {
        max7219_send(i,15); //clear the screen
    }
    if(data>99999999 || data<99999999)
        return -1; //out of range error
    else
        return 0; //end this function
}
int main()
{
    int student_id = 416324;
    GPIO_init();
    max7219_init();
    display(student_id, 7);
}
