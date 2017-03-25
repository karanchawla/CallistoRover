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

#ifndef SABERTOOTH_H
#define SABERTOOTH_H

/**
 * Includes
 */
#include "mbed.h"

/**
 * Defines
 */
#define START_CHARACTER 170


class Sabertooth {

public:
      /**
     * Constructor.
     *
     * @param
     */
     // Constructor
    Sabertooth(PinName Tx, int address, int baudrate);

    // Initialize Sabertooth
    void InitializeCom(void);
    
    // Valid speed is between -127 and +127
    void SetSpeedMotorA(int speed);
    
    // Valid speed is between -127 and +127
    void SetSpeedMotorB(int speed);
    
    void ShutdownMotors(void);
    
    // Valid speed is between -127 and +127
    int GetCommandedSpeedA();
    
    // Valid speed is between -127 and +127
    int GetCommandedSpeedB();
    
private:
    enum Motor { A, B };
    int _Address;
    int _CurrentSpeed[2];
    Serial _sabertoothserial; // tx, rx
        
    int ClipSpeed(int speed);
    
    void Send(int command, int value);
    
    void SetMotorSpeed(int motor,int speed);
};

#endif /* SABERTOOTH_H */