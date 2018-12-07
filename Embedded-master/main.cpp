#include "wheelchair.h"

QEI wheelS(D0, D1, NC, 1200);        //Initializes right encoder
DigitalIn pt3(D1, PullUp);          //Pull up resistors to read analog signals into digital signals
DigitalIn pt4(D0, PullUp);

QEI wheel (D6, D3, NC, 1200);    //Initializes Left encoder
DigitalIn pt1(D6, PullUp);          //Pull up resistors to read analog signals into digital signals
DigitalIn pt2(D3, PullUp);

AnalogIn x(A0);                     //Initializes analog axis for the joystick
AnalogIn y(A1);

DigitalOut up(D2);                  //Turn up speed mode for joystick 
DigitalOut down(D8);                //Turn down speed mode for joystick 
DigitalOut on(D12);                 //Turn Wheelchair On
DigitalOut off(D11);                //Turn Wheelchair Off
bool manual = false;                //Turns chair joystic to automatic and viceverza


Serial pc(USBTX, USBRX, 57600);     //Serial Monitor
Timer t;                            //Initialize time object t
EventQueue queue;                   //Class to organize threads

ros::NodeHandle nh;
nav_msgs::Odometry odom;
geometry_msgs::Twist commandRead;

void handlerFunction(const geometry_msgs::Twist& command){
    commandRead = command;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &handlerFunction);
ros::Publisher chatter("odom", &odom);
ros::Publisher chatter2("cmd_vel", &commandRead);

Wheelchair smart(xDir,yDir, &pc, &t, &wheel, &wheelS); //Initialize wheelchair object
Thread compass;                      //Thread for compass
Thread velosity;                      //Thread for velosity
Thread ros_com;                      //Thread for velosity


void rosCom_thread();

int main(void)
{   
    nh.initNode();
    nh.advertise(chatter);
    nh.advertise(chatter2);
    nh.subscribe(sub);
    
    queue.call_every(SAMPLEFREQ, &smart, &Wheelchair::compass_thread);  //Sets up sampling frequency of the compass_thread
    queue.call_every(SAMPLEFREQ, &smart, &Wheelchair::velosity_thread); //Sets up sampling frequency of the velosity_thread
    queue.call_every(200, rosCom_thread); //Sets up sampling frequency of the velosity_thread
    t.reset();
    compass.start(callback(&queue, &EventQueue::dispatch_forever));      //Starts running the compass thread
    velosity.start(callback(&queue, &EventQueue::dispatch_forever));     //Starts running the velosity thread
    ros_com.start(callback(&queue, &EventQueue::dispatch_forever));     //Starts running the velosity thread
    while(1) {
        //pc.printf("lin x: %f, lin y %f, lin z %f, ang z %f", commandRead.linear.x, commandRead.linear.y, commandRead.linear.z, commandRead.angular.z);
        if(commandRead.linear.x != 0) {                                        
            smart.pid_twistV();
        } else if(commandRead.angular.z != 0){                                        //Sends command to go to the kitchen
                smart.pid_twistA();
        } else {
            smart.stop();                                              //If nothing else is happening stop the chair
        }

        wait(process);
    }
}
void rosCom_thread()
{
        smart.linearV = commandRead.linear.x;
        smart.angularV = commandRead.angular.z*180/3.1415926;
        odom.pose.pose.position.x = smart.x_position;
        odom.pose.pose.position.y = smart.y_position;
        odom.pose.pose.position.z = 0;
        //set the orientation
        odom.pose.pose.orientation.z = sin(smart.z_angular*.5*3.1415926/180);
        odom.pose.pose.orientation.x = 0;
        odom.pose.pose.orientation.y = 0;
        odom.pose.pose.orientation.w = cos(smart.z_angular*.5*3.1415926/180);
        odom.twist.twist.linear.x = smart.vel;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.linear.z = 0;
        odom.twist.twist.angular.x = 0;
        odom.twist.twist.angular.y = 0;
        odom.twist.twist.angular.z = smart.getTwistZ()*3.1415926/180;
        chatter.publish(&odom);
        //chatter2.publish(&commandRead);
        nh.spinOnce();
}    