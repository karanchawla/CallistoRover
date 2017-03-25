#include "mbed.h"
#include "QEI.h"
#include <cstdlib>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "Sabertooth.h"
#include <cmath>

//Defining the pins

#ifdef TARGET_LPC1768
#define MOTORSHIELD_IN1     p23
#define MOTORSHIELD_IN2     p24
#define MOTORSHIELD_IN3     p29
#define MOTORSHIELD_IN4     p30
#define SPEEDPIN_A          p13
#define SPEEDPIN_B          p9
#define SPEEDPIN_C          p10
#endif

//Encoder and Sabertooth object intializations
QEI wheelL(MOTORSHIELD_IN3, MOTORSHIELD_IN4, NC, 624);
QEI wheelR(MOTORSHIELD_IN1, MOTORSHIELD_IN2, NC, 624);

//Geometric parameters
float L=0.28;
float R=0.03;

// Max and min motor commands
int motor_cmd_max = 127;
int motor_cmd_min = -127;

//Mapping motor commands and angular velocities
float motor_max_angular_vel = 4233.334;
float motor_min_angular_vel = -666.667;

int encoderRes = 20000;

//Time interval for PID controller
float dt = 0.01;

// PID Gains
float Kp = 20;
float Ki = 0;
float Kd = 5;

float lwheel_tangent_vel_target = 0;
float rwheel_tangent_vel_target = 0;

//Subscribing to left wheel and right wheel velocities
void lwheel_tangent_vel_target_callback( const std_msgs::Float32& float_msg)
{
    lwheel_tangent_vel_target = (float)float_msg.data;
}

void rwheel_tangent_vel_target_callback( const std_msgs::Float32& float_msg)
{
    rwheel_tangent_vel_target = (float)float_msg.data;
}

//Subscriber initialization
ros::Subscriber<std_msgs::Float32> lwheel_tangent_vel_target_sub("lwheel_tangent_vel_target", lwheel_tangent_vel_target_callback);
ros::Subscriber<std_msgs::Float32> rwheel_tangent_vel_target_sub("rwheel_tangent_vel_target", rwheel_tangent_vel_target_callback);

float tangentToAngular(float tangent_vel)
{
	//V = omega * R

    float angularVel = tangent_vel/R;
    return angularVel;
}

//TO DO:
// Add publisher defintions and maybe a method in class to get the required variables
// for publishing to ros out.

ros::NodeHandle nh;

rover Callisto(L,R,encoderRes,SPEEDPIN_A,129,9600,Kp,Ki,Kd,motor_cmd_max,motor_cmd_min,motor_max_angular_vel,motor_min_angular_vel,dt);


int main()
{
	nh.initNode();
	nh.subscribe(lwheel_tangent_vel_target_sub);
	nh.subscribe(rwheel_tangent_vel_target_sub);

	while(1)//Change to ros::ok() later on
	{
		nh.spinOnce();

		wait_ms(1);
		int lTarget, rTarget;
		lTarget = tangentToAngular(lwheel_tangent_vel_target);
		rTarget = tangentToAngular(rwheel_angular_vel_target);

		Callisto.setTarget(lTarget,rTarget);

		int pulseL, pulseR;
		pulseL = wheelL.getPulses();
		pulseR = wheelR.getPulses();

		Callisto.computeEncoderVelocity(pulseL,pulseR);
		Callisto.updateState();
		Callisto.cmdVel();

		nh.spinOnce();
		wait_ms(1);
	}


}
























