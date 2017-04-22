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

float cm_to_m(float x){return x/100;}


//Encoder and Sabertooth object intializations

QEI wheelR(MOTORSHIELD_IN3, MOTORSHIELD_IN4, NC, 624);
QEI wheelL(MOTORSHIELD_IN1, MOTORSHIELD_IN2, NC, 624);

Sabertooth Sb(SPEEDPIN_A,129,9600);

//Geometric parameters

float L = 0.28;
float R = 0.03;

//Target Wheel velocities <- received from diff_drive controller
float lwheel_tangent_vel_target = 0;
float rwheel_tangent_vel_target = 0;

//Target angular velocities <- updated by PID controller
float lwheel_angular_vel_target = 0;
float rwheel_angular_vel_target = 0;
    
// Angular velocity encoder readings
float lwheel_angular_vel_enc = 0;
float rwheel_angular_vel_enc = 0;

// Max and min motor commands
int motor_cmd_max = 127;
int motor_cmd_min = -127;

//Mapping motor commands and angular velocities
float motor_max_angular_vel = 3.95;
float motor_min_angular_vel = -3.95;

// Variables for deriving angular velocity using right encoder

int thetaCurrR = 0;
int thetaCurrL = 0;

int thetaPrevR = 0;
int thetaPrevL = 0;

float errorCurrL = 0;
float errorPrevL = 0;

float errorCurrR = 0;
float errorPrevR = 0;

float controlSignal = 0;
float targetNew = 0;

int encoderRes = 20000;

//Time interval for PID controller

float dt = 0.01;

// PID Gains
float Kp = 2.0;
float Kd = 0.1;

//Sotftware interrupt controller definition
Ticker t;

//Subscribing to left wheel and right wheel velocities

void lwheel_tangent_vel_target_callback( const std_msgs::Float32& float_msg)
{
    lwheel_tangent_vel_target = (float)float_msg.data;
}

void rwheel_tangent_vel_target_callback( const std_msgs::Float32& float_msg)
{
    rwheel_tangent_vel_target = (float)float_msg.data;
}

//////////////////////////////////////////////////////////////////////////////////

//Subscriber initialization

//////////////////////////////////////////////////////////////////////////////////

ros::Subscriber<std_msgs::Float32> lwheel_tangent_vel_target_sub("lwheel_tangent_vel_target", lwheel_tangent_vel_target_callback);
ros::Subscriber<std_msgs::Float32> rwheel_tangent_vel_target_sub("rwheel_tangent_vel_target", rwheel_tangent_vel_target_callback);

static inline float tangentvel_2_angularvel(float tangent_vel)
{
    /*
    V = omega * R
    */
    float angular_vel = tangent_vel/R;
    return angular_vel;
}

/////////////////////////////////////////////////////////////////////////////////////////////////

//Mapping angular velocity targets to motor commands 
//Note: Moto commands are ints between -127 to 127

/////////////////////////////////////////////////////////////////////////////////////////////////

static inline int angularvel_2_motorcmd(float angular_vel_target)
{
    int motor_cmd = 0;

    float slope = (motor_cmd_max - motor_cmd_min)/(motor_max_angular_vel - motor_min_angular_vel);
    float intercept = 20;

    if (angular_vel_target ==0)
        return 0;

    motor_cmd = -(slope*(angular_vel_target) + intercept); //For direction change the sign or wiring
 
    //Saturation
    if(motor_cmd > motor_cmd_max)
        motor_cmd = motor_cmd_max;
    else if(motor_cmd < motor_cmd_min)
        motor_cmd = motor_cmd_min;  

    return motor_cmd;
}


//Send motor commands to robot

static inline void motorcmd_2_robot( char wheel, int motor_command, Sabertooth &Sb)
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

float lwheel_angular_vel_target_new = 0;
float rwheel_angular_vel_target_new = 0;

static inline void controller()
{
    //Right Wheel Controller
    rwheel_angular_vel_target_new = 0;

    rwheel_angular_vel_target = tangentvel_2_angularvel(rwheel_tangent_vel_target);
    // Get right wheel angular velocity using encoder

    thetaCurrR = wheelR.getPulses();
    rwheel_angular_vel_enc = (thetaCurrR - thetaPrevR)/(encoderRes*dt);
    thetaPrevR = thetaCurrR;

    //PID controller
    errorCurrR = rwheel_angular_vel_target - rwheel_angular_vel_enc ;
    controlSignal = Kp * errorCurrR + Kd * (-errorPrevR + errorCurrR);
    rwheel_angular_vel_target_new = controlSignal;
    errorPrevR = errorCurrR;

    int rwheel_motor_cmd = angularvel_2_motorcmd(rwheel_angular_vel_target_new);
    motorcmd_2_robot('r', rwheel_motor_cmd, Sb);
    //Left wheel controller
    lwheel_angular_vel_target_new = 0;

    lwheel_angular_vel_target = tangentvel_2_angularvel(lwheel_tangent_vel_target);

    // Get left wheel angular velocity using encoder
    thetaCurrL = wheelL.getPulses();
    lwheel_angular_vel_enc = (thetaCurrL - thetaPrevL)/(encoderRes*dt);
    thetaPrevL = thetaCurrL;

    // PID Controller for left wheel
    errorCurrL = lwheel_angular_vel_target - lwheel_angular_vel_enc;
    controlSignal = Kp * errorCurrL + Kd * (-errorPrevL + errorCurrL);
    lwheel_angular_vel_target_new = controlSignal;
    errorPrevL = errorCurrL;

    //Compute motor command
    int lwheel_motor_cmd = angularvel_2_motorcmd(lwheel_angular_vel_target_new);
    motorcmd_2_robot('l',lwheel_motor_cmd, Sb);
}

/////////////////////////////////////////////////////////////////////////////

//Publisher definitions

/////////////////////////////////////////////////////////////////////////////
std_msgs::Float32 msgL;
std_msgs::Float32 msgR;

ros::Publisher lwheel_angular_vel_target_pub("lwheel_angular_vel_target", &msgL);
ros::Publisher rwheel_angular_vel_target_pub("rwheel_angular_vel_target", &msgR);

std_msgs::Float32 lwheel_angular_vel_control_msg;
std_msgs::Float32 rwheel_angular_vel_control_msg;


ros::Publisher lwheel_angular_vel_control_pub("lwheel_angular_vel_control", &lwheel_angular_vel_control_msg);
ros::Publisher rwheel_angular_vel_control_pub("rwheel_angular_vel_control", &rwheel_angular_vel_control_msg);

std_msgs::Float32 lwheel_angular_vel_enc_msg;
std_msgs::Float32 rwheel_angular_vel_enc_msg;

ros::Publisher lwheel_angular_vel_enc_pub("lwheel_angular_vel_enc", &lwheel_angular_vel_enc_msg);
ros::Publisher rwheel_angular_vel_enc_pub("rwheel_angular_vel_enc", &rwheel_angular_vel_enc_msg);

ros::NodeHandle nh;

static inline void publishData(void)
{
    msgR.data = rwheel_angular_vel_target;
    rwheel_angular_vel_target_pub.publish(&msgR);

    rwheel_angular_vel_control_msg.data = rwheel_angular_vel_target_new;
    rwheel_angular_vel_control_pub.publish(&rwheel_angular_vel_control_msg);
    
    rwheel_angular_vel_enc_msg.data =  rwheel_angular_vel_enc;        
    rwheel_angular_vel_enc_pub.publish(&rwheel_angular_vel_enc_msg);         
    
    msgL.data = lwheel_angular_vel_target;
    lwheel_angular_vel_target_pub.publish( &msgL );

    lwheel_angular_vel_control_msg.data = lwheel_angular_vel_target_new;
    lwheel_angular_vel_control_pub.publish(&lwheel_angular_vel_control_msg);

    lwheel_angular_vel_enc_msg.data = lwheel_angular_vel_enc;
    lwheel_angular_vel_enc_pub.publish(&lwheel_angular_vel_enc_msg);

    return;
}

//Driver function
int main()
{
    //Initialize the node
    nh.initNode();

    while (!nh.connected()) { nh.spinOnce(); }

    nh.loginfo("MBED ready.");

    //Pubs intializations
    nh.advertise(lwheel_angular_vel_target_pub);
    nh.advertise(rwheel_angular_vel_target_pub);
    nh.advertise(lwheel_angular_vel_control_pub);
    nh.advertise(rwheel_angular_vel_control_pub);
    nh.advertise(lwheel_angular_vel_enc_pub);
    nh.advertise(rwheel_angular_vel_enc_pub);

    //Subs intialization
    nh.subscribe(lwheel_tangent_vel_target_sub);
    nh.subscribe(rwheel_tangent_vel_target_sub);

    //Motor driver initialization
    Sb.InitializeCom();
    t.attach(&controller,dt);
    while(1)
    {
        publishData();
         
        nh.spinOnce();

        wait_ms(1);
    }

}