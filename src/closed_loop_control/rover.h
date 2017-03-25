/*
 * rover.h
 *
 *  Created on: Mar 12, 2017
 *      Author: karan
 */

#ifndef ROVER_H_
#define ROVER_H_



#include "mbed.h"
#include "pid.h"
#include "Sabertooth.h"
#include "QEI.h"
#include "math.h"
#include <cstdlib>


class rover
{
public:
	rover(float left,float right,int encResoltn, PinName sbtx,int addr,int baud,float kp,float ki,float kd,float cmd_max,float cmd_min,float vel_max,float vel_min,float deltaT);
	virtual ~rover();
	void computeEncoderVelocity(int leftAngle, int rightAngle);
	void updateState();
	void setTarget(float lTarget, float rTarget);
	void cmdVel();
	float rWheelAngularVelEnc;
	float lWheelAngularVelEnc;
	float rWheelAngularVelTarget;
	float lWheelAngularVelTarget;

private:
	float L;
	float R;
	Sabertooth Sb;
	int encoderRes;
	pid lWheel;
	pid rWheel;
	float dt;
	float V;
	float W;
	int prevAngleL;
	int prevAngleR;


};

#endif /* ROVER_H_ */
