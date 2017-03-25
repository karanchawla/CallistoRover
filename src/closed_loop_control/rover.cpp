/*
 * rover.cpp
 *
 *  Created on: Mar 12, 2017
 *      Author: karan
 */

#include "rover.h"



rover::rover(float left, float right, int encResoltn, PinName sbtx , int addr, int baud, float kp, float ki, float kd, float cmd_max , float cmd_min, float vel_max, float vel_min, float deltaT)
:L(left),R(right),Sb(sbtx,addr,baud),lWheel(kp,ki,kd,cmd_max,cmd_min,vel_max,vel_min), rWheel(kp,ki,kd,cmd_max,cmd_min,vel_max,vel_min)
{
	// TODO Auto-generated constructor stub
	encoderRes = encResoltn;
	lWheelAngularVelEnc = 0;
	rWheelAngularVelEnc = 0;
	dt = deltaT;
	V = 0;
	W = 0;
	Sb.InitializeCom();
}

rover::~rover() {
	// TODO Auto-generated destructor stub
}

void rover::computeEncoderVelocity(int leftAngle, int rightAngle)
{
	lWheelAngularVelEnc = (float)(leftAngle - prevAngleL)/(encoderRes*dt);
	rWheelAngularVelEnc = (float)(rightAngle - prevAngleR)/(encoderRes*dt);
	prevAngleL = leftAngle;
	prevAngleR = rightAngle;
}


void rover::setTarget(float lTarget, float rTarget)
{
	lWheelAngularVelTarget = lTarget;
	rWheelAngularVelTarget = rTarget;
}

void rover::updateState()
{
	lWheel.Update(lWheelAngularVelEnc,lWheelAngularVelTarget);
	rWheel.Update(rWheelAngularVelEnc,rWheelAngularVelTarget);
}


void rover::cmdVel()
{
	Sb.SetSpeedMotorA(rWheel.getCmd());
	Sb.SetSpeedMotorB(lWheel.getCmd());
}
