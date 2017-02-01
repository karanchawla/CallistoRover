

    Developer Resources
    Partners
    Cloud

    Hardware
    Documentation
    Code
    Questions
    Forum
    |
    karanchawla
    Compiler

Users » mitchelljones » Code » ROSRobot2
Mitchell Jones / ROSRobot2

controller for 2 motor robot with ROS

Dependencies:   mbed

    Home History Graph API Documentation Wiki Pull Requests 

    file
    revisions
    annotate
    diff
    raw

main.cpp

Committer:
    mitchelljones
Date:
    9 months ago
Revision:
    0:67bc01af349e

File content as of revision 0:67bc01af349e:

#include "mbed.h"
#include "QEI.h"
#include <cstdlib>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include "Sabertooth.h"
 
Ticker flipper;
/*Serial pc(USBTX, USBRX);//Serial port*/
/*Serial xbee(p9, p10); //used to connected the mbed to the xbee over Serial UART comm.*/
//PwmOut wheel1(p21); //signal for speed/direction to right wheels
//PwmOut wheel2(p22); //singal for speed/direction to left wheels
QEI wheelR (p29, p30, NC, 624); // Signal from encoder in right wheel
QEI wheelL (p27, p28, NC, 624); // Signal from encoder in left wheel
DigitalOut led1(p20);  // LED for xbee connection verification
DigitalOut led2(p19);  // LED for W+ verification
DigitalOut led3(p18);  // LED for W- verification
 
float L=0.28; //Distance between left and right wheel
float R=0.03;//Radius of wheel
int encoderResolution=20000;// This is including the gear ratio
float vR = 0.00;
float vL = 0.00; //Due to the type of Pwm input the Sabertooth v1 takes, this is the signal for the robot to not move
float V=0;//Desired linear velocity of robot
float W=0;//Desired angular velocity of robot
int thetaprevR=0; // Prev angular rotation of motor
int thetacurrR=0; // Current angular rotation of motor
int thetaprevL=0; // Prev angular rotation of motor
int thetacurrL=0; // Current angular rotation of motor
float omegaR=0; // Angular velocity of the motor of right motor
float omegaL=0; // Angular velocity of the motor of left motor
float errR=0; // Error
float errL=0; // Error
float preverrR=0;//Error in the last time step
float interrR=0; //Integral of the error
float differrR=0;//Derivative of error
float preverrL=0;//Error in the last time step
float interrL=0; //Integral of the error
float differrL=0;//Derivative of error
float Kp=3.0;// Propotional controller constant
float Ki=0.001;//0.0000000001;//Integral controller constant
float Kd=0;//0.001;//Differential controller constant
float Kpw=0.1;// Propotional cotroller for omega
float Kiw=0.0001;// Propotional cotroller for omega
float errOmega=0;// Error in angular velocity
float errOmegaprev=0;// Error in angular velocity in last step
float interrOmega=0;// Integral of error in angular velocity
float dt=0.01; // Time step
float omegaDesiredR=((2*V-W*L)/(2*R*6.28)); // Desired angular velocity of right wheel
float omegaDesiredL=((2*V+W*L)/(2*R*6.28)); //Desired angular velocity of left wheel 
float thetaTotal=0;
char w[32];
 
 
 void checkVelocity(){
    
    thetacurrR=wheelR.getPulses();
    thetacurrL=wheelL.getPulses();
    omegaR=(float)(thetacurrR-thetaprevR)/(encoderResolution*dt);
    omegaL=(float)(thetacurrL-thetaprevL)/(encoderResolution*dt);
    thetaprevR=thetacurrR;
    thetaprevL=thetacurrL;
    errR=omegaDesiredR-omegaR;
    errL=omegaDesiredL-omegaL;
    interrR=interrR+dt*errR;
    interrL=interrL+dt*errL;
    differrR=(errR-preverrR)/dt;
    differrL=(errL-preverrL)/dt;
    errOmega=W-(((omegaL-omegaR)*2*3.14*R)/L);
    interrOmega=interrOmega+dt*errOmega;
 
   if((errL>0.01||errL<-0.01)){
            //vL+=(Kp*(errL)+Ki*interrL);//-Kpw*errOmega);//-Kiw*interrOmega);
            vL+=(Kp*(errL)+Kd*(differrL)+Ki*interrL);//-Kpw*errOmega);//-Kiw*interrOmega);
   }
   
   
   if((errR>0.01||errR<-0.01)){
            //vR+=(Kp*(errR)+Ki*interrR);//-Kpw*errOmega);//-Kiw*interrOmega);
            vR+=(Kp*(errR)+Kd*(differrR)+Ki*interrR);//-Kpw*errOmega);//-Kiw*interrOmega);
   } 
   
   if(interrR>20){
    interrR = 20;
}
if(interrL>20){
    interrL = 20;
}
if(interrR<-20){
    interrR = -20;
}
if(interrL<-20){
    interrL = -20;
}
    preverrR=errR;
    preverrL=errL;
    errOmegaprev=errOmega;
    
    
/* if(xbee.readable()){*/
        //xbee.scanf("%s\n",w);
        //pc.printf("%s   ",w);
        
        //if((atof(w)/1000.0)>=-0.7 && (atof(w)/1000.0)<0.7)
        //{W=atof(w)/1000.0;}
        
         omegaDesiredR=((2*V-W*L)/(2*R*6.28));
         omegaDesiredL=((2*V+W*L)/(2*R*6.28));  
/*}*/
 
} 
 
void cmd(const geometry_msgs::Vector3 &cmd_msg) {
     V = (float) cmd_msg.x;
     W = (float) cmd_msg.y;  
     led3 = 1;
}
 
ros::NodeHandle nh(p9,p10);
ros::Subscriber<geometry_msgs::Vector3> sub("cmd_vel_w", cmd);
Sabertooth sb(p13,129,9600);
 
 
int main() {
    
    nh.initNode();
    nh.subscribe(sub);
    sb.InitializeCom();
 
    //wheel1.period(0.20);
    //wheel2.period(0.20);
      
    flipper.attach(&checkVelocity, dt);
    while(1){
        nh.spinOnce();
    wait_ms(1);
    if (V>0)
    {
    led1 = 1;
    }
    
        sb.SetSpeedMotorA((int)vR);
    sb.SetSpeedMotorB((int)vL);
    
        //wheel1.pulsewidth(vR); //output the current speed value to right motor
        //wheel2.pulsewidth(vL); //output the current speed value to left motor  
/*        wheel1.pulsewidth(0.00155);*/
        //wheel2.pulsewidth(0.001);
       
    }
}
 
            

Repository toolbox

Embed:
Import into Compiler
Export to desktop IDE
Build repository
Follow
Clone repository to desktop:
Repository details
Type: 	Program
Created: 	26 Apr 2016
Imports: 	1
Forks: 	0
Commits: 	1
Dependents: 	0
Dependencies: 	1
Followers: 	1

    © mbed blog we're hiring! support service status privacy policy terms and conditions 

