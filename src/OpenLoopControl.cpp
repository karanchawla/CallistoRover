#include "mbed.h"
#include "QEI.h"
#include <cstdlib>
#include <ros.h>
#include <std_msgs/Float32.h>
#include "Sabertooth.h"
#include <cmath>

 
#ifdef TARGET_LPC1768
#define MOTORSHIELD_IN1     p23
#define MOTORSHIELD_IN2     p24
#define MOTORSHIELD_IN3     p29
#define MOTORSHIELD_IN4     p30
#define SPEEDPIN_A          p13
#define SPEEDPIN_B          p26
#elif defined(TARGET_KL25Z) || defined(TARGET_NUCLEO_F401RE)
#define MOTORSHIELD_IN1     D8
#define MOTORSHIELD_IN2     D11
#define MOTORSHIELD_IN3     D12
#define MOTORSHIELD_IN4     D13
#define SPEEDPIN_A          D9
#define SPEEDPIN_B          D10
#else
#error "You need to specify a pin for the sensor"
#endif

Ticker flipper;

QEI wheelL(MOTORSHIELD_IN3, MOTORSHIELD_IN4, NC, 624);
QEI wheelR(MOTORSHIELD_IN1, MOTORSHIELD_IN2, NC, 624);
Sabertooth Sb(SPEEDPIN_A,129,9600);

float L=0.28;
float R=0.03;


float lwheel_tangent_vel_target = 0;
float rwheel_tangent_vel_target = 0;

float lwheel_angular_vel_target = 0;
float rwheel_angular_vel_target = 0;
    
// Angular velocity encoder readings
float lwheel_angular_vel_enc = 0;
float rwheel_angular_vel_enc = 0;

int motor_cmd_max = 127;
int motor_cmd_min = -127;
float motor_max_angular_vel = 4233.334;
float motor_min_angular_vel = -666.667;

/*
ros::Publisher lwheel_angular_vel_target_pub("lwheel_angular_vel_target", &float_msg);
ros::Publisher rwheel_angular_vel_target_pub("rwheel_angular_vel_target", &float_msg);

ros::Publisher lwheel_angular_vel_control_pub("lwheel_angular_vel_control", &float_msg);
ros::Publisher rwheel_angular_vel_control_pub("rwheel_angular_vel_control", &float_msg);

ros::Publisher lwheel_angular_vel_motor_pub("lwheel_angular_vel_motor", &int_msg);
ros::Publisher rwheel_angular_vel_motor_pub("rwheel_angular_vel_motor", &int_msg);
*/

//Subscribing to left wheel and right wheel velocities
void lwheel_tangent_vel_target_callback( const std_msgs::Float32& float_msg)
{
    lwheel_tangent_vel_target = (float)float_msg.data;
}

void rwheel_tangent_vel_target_callback( const std_msgs::Float32& float_msg)
{
    rwheel_tangent_vel_target = (float)float_msg.data;
}

//Read in encoder readings for PID
void lwheel_angular_vel_enc_callback(const std_msgs::Float32& float_msg)
{
	lwheel_angular_vel_enc = float_msg.data;
}

void rwheel_angular_vel_enc_callback(const std_msgs::Float32& float_msg)
{
	rwheel_angular_vel_enc = float_msg.data;
}

//Subscriber initialization
ros::Subscriber<std_msgs::Float32> lwheel_tangent_vel_target_sub("lwheel_tangent_vel_target", lwheel_tangent_vel_target_callback);
ros::Subscriber<std_msgs::Float32> rwheel_tangent_vel_target_sub("rwheel_tangent_vel_target", rwheel_tangent_vel_target_callback);

ros::Subscriber<std_msgs::Float32> lwheel_angular_vel_enc_sub("lwheel_angular_vel_enc", lwheel_angular_vel_enc_callback);
ros::Subscriber<std_msgs::Float32> rwheel_angular_vel_enc_sub("rwheel_angular_vel_enc", rwheel_angular_vel_enc_callback);

float tangentvel_2_angularvel(float tangent_vel)
{
	/*
	V = omega * R
	*/
    float angular_vel = tangent_vel/R;
    return angular_vel;
}


/*
Mapping angular velocity targets to motor commands 
Note: Moto commands are ints between -127 to 127
*/
int angularvel_2_motorcmd(float angular_vel_target)
{
    int motor_cmd;

    float slope = (motor_cmd_max - motor_cmd_min)/(motor_max_angular_vel - motor_min_angular_vel);
    float intercept = 20;
    if (angular_vel_target ==0)
        return 0;

    if (angular_vel_target > 0)
    {

        motor_cmd = slope*(angular_vel_target) + intercept;
        if(motor_cmd > motor_cmd_max)
            motor_cmd = motor_cmd_max;
        if(motor_cmd < motor_cmd_min)
            motor_cmd = motor_cmd_min;
    }
    else 
    {
        motor_cmd = slope*(angular_vel_target) + intercept;
        if(motor_cmd > motor_cmd_max)
            motor_cmd = motor_cmd_max;
        if(motor_cmd < motor_cmd_min)
            motor_cmd = motor_cmd_min;  

    }

    return motor_cmd;
}

//Send motor commands to robot
void motorcmd_2_robot( char wheel, int motor_command, Sabertooth &Sb)
{

    if(wheel == 'r')
    {
        Sb.SetSpeedMotorA(motor_command);
    }

    if(wheel == 'l')
    {
        Sb.SetSpeedMotorB(motor_command);
    }
}

ros::NodeHandle nh;

int main()
{
    nh.initNode();
    nh.subscribe(lwheel_tangent_vel_target_sub);
    nh.subscribe(rwheel_tangent_vel_target_sub);
    nh.subscribe(lwheel_angular_vel_enc_sub);
    nh.subscribe(rwheel_angular_vel_enc_sub);

    Sb.InitializeCom();

    while(1)
    {
        nh.spinOnce();
        wait_ms(1);

        rwheel_angular_vel_target = tangentvel_2_angularvel(rwheel_tangent_vel_target);
        //rwheel_angular_vel_target_pub.publish(rwheel_angular_vel_target);

        //rwheel_angular_vel_control_pub.publish(rwheel_angular_vel_target);
        int rwheel_motor_cmd = angularvel_2_motorcmd(rwheel_angular_vel_target);
        //rwheel_angular_vel_motor_pub.publish(rwheel_motor_cmd); 

        motorcmd_2_robot('r', rwheel_motor_cmd, Sb);

        lwheel_angular_vel_target = tangentvel_2_angularvel(lwheel_tangent_vel_target);
        //lwheel_angular_vel_target_pub.publish(lwheel_angular_vel_target);

        //lwheel_angular_vel_control_pub.publish(lwheel_angular_vel_target);

        //Compute motor command
        int lwheel_motor_cmd = angularvel_2_motorcmd(lwheel_angular_vel_target);
        //lwheel_angular_vel_motor_pub.publish(lwheel_motor_cmd); 

        motorcmd_2_robot('l',lwheel_motor_cmd, Sb);
        wait_ms(500);
    }

}
