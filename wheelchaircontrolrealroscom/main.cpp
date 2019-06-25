#include "wheelchair.h"

/* Intializes the right encoder */
QEI wheelS(D9, D10, NC, 1200);  
/* Set pull-up resistors to read analog signals into digital signals */
DigitalIn pt3(D10, PullUp);         
DigitalIn pt4(D9, PullUp);

/* Initializes Left encoder */
QEI wheel (D7, D8, NC, 1200);    
/* Set pull-up resistors to read analog signals into digital signals */
DigitalIn pt1(D7, PullUp);         
DigitalIn pt2(D8, PullUp);

/* Initializes analog axis for the joystick */
AnalogIn x(A0);                    
AnalogIn y(A1);

double test1;
double test2;

/* Initializing more pins for wheelchair control */
DigitalOut up(D2);                                                  //Turn up speed mode for joystick 
DigitalOut down(D8);                                                //Turn down speed mode for joystick 
DigitalOut on(D12);                                                 //Turn Wheelchair On
DigitalOut off(D11);                                                //Turn Wheelchair Off


VL53L1X sensor1(PB_11, PB_10, D0);         //initializes ToF sensors
VL53L1X sensor2(PB_11, PB_10, D1);
VL53L1X sensor3(PB_11, PB_10, D2);
VL53L1X sensor4(PB_11, PB_10, D3);
VL53L1X sensor5(PB_11, PB_10, D4);
VL53L1X sensor6(PB_11, PB_10, D5);
VL53L1X sensor7(PB_11, PB_10, PE_14);
VL53L1X sensor8(PB_11, PB_10, PE_12);
VL53L1X sensor9(PB_11, PB_10, PE_10);
VL53L1X sensor10(PB_11, PB_10, PE_15);
VL53L1X sensor11(PB_11, PB_10, D6);
VL53L1X sensor12(PB_11, PB_10, D11);

VL53L1X* ToF[12] = {&sensor1, &sensor2, &sensor3, &sensor4, &sensor5, &sensor6, 
&sensor7, &sensor8, &sensor9, &sensor10, &sensor11, &sensor12};                 //puts ToF sensor pointers into an array
VL53L1X** ToFT = ToF;

/* Changes the control mode of wheelchair: Automatic or Manual */
bool manual = false;                                        

Serial pc(USBTX, USBRX, 57600);                                     //Serial Monitor
Timer t;                                                            //Initialize time object t
EventQueue queue;                                                   //Class to organize threads

ros::NodeHandle nh;
nav_msgs::Odometry odom;
geometry_msgs::Twist commandRead;

void handlerFunction(const geometry_msgs::Twist& command){
    commandRead = command;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &handlerFunction);
ros::Publisher chatter("odom", &odom);
ros::Publisher chatter2("cmd_vel", &commandRead);

/* Initialize Wheelchair objects and threads */
Wheelchair smart(xDir,yDir, &pc, &t, &wheel, &wheelS, ToFT); //Initialize wheelchair object
Thread compass;                      
Thread velocity;                      
Thread ros_com;    
Thread assistSafe;        

/* This thread continues the communication with ROS and Mbed */
void rosCom_thread();

int main(void)
{   
    /* Initialize ROS commands to publish and subscribe to nodes */
    nh.initNode();
    nh.advertise(chatter);
    nh.advertise(chatter2);
    nh.subscribe(sub);
    
    /* Sets up sampling frequency of threads */
    queue.call_every(SAMPLEFREQ, &smart, &Wheelchair::compass_thread);      
    queue.call_every(SAMPLEFREQ, &smart, &Wheelchair::velocity_thread);  
    queue.call_every(SAMPLEFREQ, &smart, &Wheelchair::assistSafe_thread); //Sets up sampling frequency of the velosity_thread
    queue.call_every(200, rosCom_thread);                              
    
    t.reset();                                                            //resets the time
    
    /* Start running threads */
    compass.start(callback(&queue, &EventQueue::dispatch_forever));          
    velocity.start(callback(&queue, &EventQueue::dispatch_forever));  
    assistSafe.start(callback(&queue, &EventQueue::dispatch_forever));     //Starts running the velosity thread
    ros_com.start(callback(&queue, &EventQueue::dispatch_forever));     
    
    
    while(1) {
        /* If Ros comands the wheelchair to move fowards or backwards*/
        if(commandRead.linear.x != 0) 
        {                                        
            smart.pid_twistV();                                           //Updates the twist linear velocity of chair
            test1 = 3.14159;
        } 
        /* If Ros comands the wheelchair to turn at a certain speed*/
        else if(commandRead.angular.z != 0)                               
        {                         
                smart.pid_twistA();                                       //Updates the twist angular velocity of chair
                test2 = 2.782;
        } 
        /* If Ros does not give veloc
        ity comands*/
        else
        {
            smart.stop();                                                 //Stops the chair
            test2 = 0;
            test1 = 0;
        }
        wait(process);                                                    //Delay
    }
    
}

/* This thread allows for continuous update and publishing of ROS Odometry/Twist message  */
void rosCom_thread()
{
        /*Determines linear and angular velocity */
        smart.linearV = commandRead.linear.x;
        smart.angularV = commandRead.angular.z*180/3.1415926;
    
        /* Publishes the position of the Wheelchair for Odometry */
        odom.pose.pose.position.x = smart.x_position;
        odom.pose.pose.position.y = smart.y_position;
        odom.pose.pose.position.z = 0;
    
        /* Publishes the orientation of the Wheelchair for Odometry */
        odom.pose.pose.orientation.z = sin(smart.z_angular*.5*3.1415926/180);
        odom.pose.pose.orientation.x = 0;
        odom.pose.pose.orientation.y = 0;
        odom.pose.pose.orientation.w = cos(smart.z_angular*.5*3.1415926/180);
    
        /* Publishes Twist linear velocity of Wheelchair */
        odom.twist.twist.linear.x = smart.vel;
        odom.twist.twist.linear.y = commandRead.linear.x;
        odom.twist.twist.linear.z = commandRead.angular.z;
    
        /* Publishes Twist angular velocity of Wheelchair */
        odom.twist.twist.angular.x = test1;
        odom.twist.twist.angular.y = test2;
        odom.twist.twist.angular.z = smart.getTwistZ()*3.1415926/180;
    
        /* Allows for Odometry to be published and sent to ROS */
        chatter.publish(&odom);
        //chatter2.publish(&commandRead);
        
        /*Checks for incoming messages */
        nh.spinOnce();                
}    