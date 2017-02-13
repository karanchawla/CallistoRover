/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#include "mbed.h"
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "Sabertooth.h"
#include "QEI.h"

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

QEI EncdrL(MOTORSHIELD_IN3, MOTORSHIELD_IN4, NC, 624);
QEI EncdrR(MOTORSHIELD_IN1, MOTORSHIELD_IN2, NC, 624);
Sabertooth sb(SPEEDPIN_A,129,9600);

float lwheel_tangent_vel_target = 0;
float rwheel_tangent_vel_target = 0;

float lwheel_angular_vel_target = 0;
float rwheel_angular_vel_target = 0;
    
// Angular velocity encoder readings
float lwheel_angular_vel_enc = 0;
float rwheel_angular_vel_enc = 0;

int motor_cmd_max = 127;
int motor_cmd_min = -127;

float R = 0.03;

ros::NodeHandle nh;
std_msgs::Float32 float_msg;
std_msgs::Int32 int_msg;

ros::Publisher lwheel_angular_vel_target_pub("lwheel_angular_vel_target", &float_msg);
ros::Publisher rwheel_angular_vel_target_pub("rwheel_angular_vel_target", &float_msg);

ros::Publisher lwheel_angular_vel_control_pub("lwheel_angular_vel_control", &float_msg);
ros::Publisher rwheel_angular_vel_control_pub("rwheel_angular_vel_control", &float_msg);

ros::Publisher lwheel_angular_vel_motor_pub("lwheel_angular_vel_motor", &int_msg);
ros::Publisher rwheel_angular_vel_motor_pub("rwheel_angular_vel_motor", &int_msg);

void lwheel_tangent_vel_callback( const std_msgs::Float32& msg)
{
    lwheel_tangent_vel_target = msg.data;
}

void rwheel_tangent_vel_callback( const std_msgs::Float32& msg)
{
    rwheel_tangent_vel_target = msg.data;
}

void lwheel_angular_vel_callback( std_msgs::Int32& msg)
{
    lwheel_angular_vel_target = msg.data;
}

void rwheel_angular_vel_callback( std_msgs::Int32& msg)
{
    rwheel_angular_vel_target = msg.data;
}

//Read in tangential velocity targets
ros::Subscriber<std_msgs::Float32> lwheel_tangent_vel_target_sub("lwheel_tangent_vel_target",lwheel_tangent_vel_callback);
ros::Subscriber<std_msgs::Float32> rwheel_tangent_vel_target_sub("rwheel_tangent_vel_target",rwheel_tangent_vel_callback);

float tangentvel_2_angularvel(float tangent_vel)
{
    float angular_vel = tangent_vel/R;
    return angular_vel;
}

int angularvel_2_motorcmd(float angular_vel_target)
{
    int motor_cmd;

    if (angular_vel_target ==0)
        return 0;

    if (angular_vel_target > 0)
    {
        motor_cmd = 0; //Set this at the lab
        if(motor_cmd > motor_cmd_max)
            motor_cmd = motor_cmd_max;
        if(motor_cmd < motor_cmd_min)
            motor_cmd = motor_cmd_min;
    }
    else 
    {
        motor_cmd = 0; //Set this later
        if(motor_cmd > motor_cmd_max)
            motor_cmd = motor_cmd_max;
        if(motor_cmd < motor_cmd_min)
            motor_cmd = motor_cmd_min;  

        motor_cmd = -motor_cmd; 
    }

    return motor_cmd;
}

void motorcmd_2_robot( char wheel, int motor_command, Sabertooth &Sb)
{
    int motor_command_raw = (int) abs(motor_command);

    if(wheel == 'l')
    {
        if(motor_command>=0)
        {
            Sb.SetSpeedMotorA(motor_command_raw);
        }
        else if(motor_command < 0)
        {
            Sb.SetSpeedMotorA(-motor_command_raw);
        }
    }

    if(wheel == 'r')
    {
        if(motor_command>=0)
        {
            Sb.SetSpeedMotorB(motor_command_raw);
        }
        else if(motor_command < 0)
        {
            Sb.SetSpeedMotorB(-motor_command_raw);
        }
    }
}

void lwheel_update(Sabertooth &Sb)
{
    lwheel_angular_vel_target = tangentvel_2_angularvel(lwheel_tangent_vel_target);
    lwheel_angular_vel_target_pub.publish(lwheel_angular_vel_target);

    lwheel_angular_vel_control_pub.publish(lwheel_angular_vel_target);

    //Compute motor command
    int lwheel_motor_cmd = angularvel_2_motorcmd(lwheel_angular_vel_target);
    lwheel_angular_vel_motor_pub.publish(lwheel_motor_cmd); 

    motorcmd_2_robot('l',lwheel_motor_cmd, Sb);

}

void rwheel_update(Sabertooth &Sb)
{
    rwheel_angular_vel_target = tangentvel_2_angularvel(rwheel_tangent_vel_target);
    rwheel_angular_vel_target_pub.publish(rwheel_angular_vel_target);

    rwheel_angular_vel_control_pub.publish(rwheel_angular_vel_target);

    //Compute motor command
    int rwheel_motor_cmd = angularvel_2_motorcmd(rwheel_angular_vel_target);
    rwheel_angular_vel_motor_pub.publish(rwheel_motor_cmd); 

    motorcmd_2_robot('r', rwheel_motor_cmd, Sb);

}

DigitalOut myled(LED1);

void messageCb(const std_msgs::Empty& toggle_msg){
    myled = !myled;   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb);

int main() {
    nh.initNode();
    nh.subscribe(sub);
    sb.InitializeCom();

    while (1) {
    	sb.SetSpeedMotorA((int)50);
        sb.SetSpeedMotorB((int)-50);
        nh.spinOnce();
        wait_ms(1);
    }
}
