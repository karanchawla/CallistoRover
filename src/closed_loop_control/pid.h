/*
 * pid.h
 *
 *  Created on: Mar 25, 2017
 *      Author: karan
 */

#ifndef PID_H_
#define PID_H_

class pid {
public:
	pid(float PGain, float IGain, float DGain, float cmd_max , float cmd_min, float vel_max, float vel_min);
	virtual ~pid();
	void Update(float currValue, float refValue);
	float velToCmd(float angularVelTarget);
	float getCmd();
	float getError();
	float setGains(float PGain, float IGain, float DGain);

private:
	float Kp;
	float Kd;
	float Ki;
	float motorCmdMax;
	float motorCmdMin;
	float motorMaxAngularVel;
	float motorMinAngularVel;
	float prevError;
	float intError;
	float cmd;
	float diffError;
};

#endif /* PID_H_ */
