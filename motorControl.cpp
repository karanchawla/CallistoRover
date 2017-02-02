#include"mbed.h"
#include "Sabertooth.h"
#include "QEI.h"

//Quadrature Encoder constructor initialization
QEI wheelRight(p29, p30, NC, 624); 
QEI wheelLeft(p27, p28, NC, 624);

DigitalOut myLED(p19);

//Initialize Sabertooth serial connection
Sabertooth sb(p13,129,9600); //DIP switch configuration - check

//Driver program
int main()
{
    sb.InitializeCom();
    sb.SetSpeedMotorA((int)50);
    sb.SetSpeedMotorB((int)50);
    myLED = 1 ;
}    
