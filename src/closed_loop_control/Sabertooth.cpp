/**
 * @author Birdy
 *
 * @section LICENSE
 *
 * Copyright (c) 2010 ARM Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 * 
 * Sabertooth motor drivers  
 *
 * Website:
 * http://www.dimensionengineering.com/
 * 
 */

/**
 * Includes
 */
#include "Sabertooth.h"

Sabertooth::Sabertooth(PinName Tx, int address, int baudrate) : _sabertoothserial(Tx, NC) {
  
    _Address = address;
    _sabertoothserial.baud(baudrate);
    // Initialize Current Speed
    _CurrentSpeed[A] = 0;
    _CurrentSpeed[B] = 0;

}

void Sabertooth::InitializeCom(void){
    wait(2); // wait for sabertooth to initialized
    _sabertoothserial.putc(START_CHARACTER);
}

// Valid speed is between -127 and +127
void Sabertooth::SetSpeedMotorA(int speed)
{
    SetMotorSpeed(A, speed);
}

// Valid speed is between -127 and +127
void Sabertooth::SetSpeedMotorB(int speed)
{
    SetMotorSpeed(B, speed);
}


void Sabertooth::SetMotorSpeed(int motor,int speed){
   
    speed = ClipSpeed(speed);
    _CurrentSpeed[motor] = speed;
    int motorCommandStart = (motor == 0 ? 0 : 4);
    if (speed >= 0)
    {
        // forward
        Send(motorCommandStart + 0, speed);
    }
    else
    {
        // backwards
        Send(motorCommandStart + 1, -speed);
    }

}

// Valid speed is between -127 and +127
int Sabertooth::GetCommandedSpeedA()
{
    return _CurrentSpeed[A];
}

    // Valid speed is between -127 and +127
int Sabertooth::GetCommandedSpeedB()
{
    return _CurrentSpeed[B];
}

void Sabertooth::ShutdownMotors(void){
    _CurrentSpeed[A] = 0;
    _CurrentSpeed[B] = 0;
    SetSpeedMotorA(_CurrentSpeed[A]);
    SetSpeedMotorB(_CurrentSpeed[B]);
}


int Sabertooth::ClipSpeed(int speed)
{
    if (speed < -127)
    {
        return -127;
    }
    if (speed > 127)
    {
        return 127;
    }
    return speed;
}


void Sabertooth::Send(int command, int value)
{
    int tmp;
    _sabertoothserial.putc(_Address);
    _sabertoothserial.putc(command);
    _sabertoothserial.putc(value);
    tmp = _Address+command+value;
    tmp = tmp & 0x7F;
    _sabertoothserial.putc(tmp);
}