#include <stdio.h>
#include <stdlib>
#include “stm32l4xx.h”
//TODO: define your gpio pin
//WTF Sequence order?
#define X0 GPIO_PA3
#define X1 GPIO_PA2
#define X2 GPIO_PA1
#define X3 GPIO_PA0
#define Y0 GPIO_PA7
#define Y1 GPIO_PA6
#define Y2 GPIO_PA5
#define Y3 GPIO_PA4
unsigned int x_pin = {X0, X1, X2, X3}; //COL 0 1 2 3
unsigned int y_pin = {Y0, Y1, Y2, Y3}; //ROW 0 1 2 3
//PA76543210
/* TODO: initial keypad gpio pin, X as output and Y as input
*/
void keypad_init()
{
    GPIO_init(); //have initialized in arm
}
/* TODO: scan keypad value
* return:
* >=0: key pressed value
* -1: no key press
*/
char keypad_scan()
{
    //key get get 255 for no key press
    //if pressed , keypad return the value of that key, otherwise, return 255 for no pressed

}
int main()
{

}
