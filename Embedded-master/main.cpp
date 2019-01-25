#include "wheelchair.h"

/* Intializes the right encoder */
QEI wheelS(D0, D1, NC, 1200);  
/* Set pull-up resistors to read analog signals into digital signals */
DigitalIn pt3(D1, PullUp);         
DigitalIn pt4(D0, PullUp);

/* Initializes Left encoder */
QEI wheel (D6, D3, NC, 1200);    

/* Set pull-up resistors to read analog signals into digital signals */
DigitalIn pt1(D6, PullUp);         
DigitalIn pt2(D3, PullUp);

DigitalIn e_button(D4, PullUp);     //set pin for emergency button

/* Initializes analog axis for the joystick */
AnalogIn x(A0);                    
AnalogIn y(A1);

/* Initializing more pins for wheelchair control */
DigitalOut up(D2);                                                  //Turn up speed mode for joystick 
DigitalOut down(D8);                                                //Turn down speed mode for joystick 
DigitalOut on(D12);                                                 //Turn Wheelchair On
DigitalOut off(D11);                                                //Turn Wheelchair Off

/* Changes the control mode of wheelchair: Automatic or Manual */
bool manual = false;                                        

Serial pc(USBTX, USBRX, 57600);                                     //Serial Monitor
Timer t;                                                            //Initialize time object t
EventQueue queue;                                                   //Class to organize threads

/* Import ROS commands to establish communication with ROS and Mbed */
ros::NodeHandle nh;                                                 //Creates publisher and subscriber and takes care of serial port communication
nav_msgs::Odometry odom;                                            //Create a topic odom in ROS
geometry_msgs::Twist commandRead;                                   //Create a topic commandRead in ROS

/* This function allows for receiving ROS Twist messages */
void handlerFunction(const geometry_msgs::Twist& command){
    commandRead = command;
}

/*Establishes a subcriber and publisher to respective node/topic */
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &handlerFunction);   //Creates Subscriber to cmd_vel ROS topic
ros::Publisher chatter("odom", &odom);                                    //Creates Publisher node to odom ROS topic
ros::Publisher chatter2("cmd_vel", &commandRead);                         //Creates Publisher node to cmd_vel ROS topic

/* Initialize Wheelchair objects and threads */
Wheelchair smart(xDir,yDir, &pc, &t, &wheel, &wheelS); 
Thread compass;                      
Thread velocity;                      
Thread ros_com;            

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
    queue.call_every(200, rosCom_thread);                              
    
    t.reset();                                                            //resets the time
    
    /* Start running threads */
    compass.start(callback(&queue, &EventQueue::dispatch_forever));          
    velocity.start(callback(&queue, &EventQueue::dispatch_forever));             
    ros_com.start(callback(&queue, &EventQueue::dispatch_forever));     
    
    
    while(1) {
        /* If Ros comands the wheelchair to move fowards or backwards*/
        if(commandRead.linear.x != 0) 
        {                                        
            smart.pid_twistV();                                           //Updates the twist linear velocity of chair
        } 
        /* If Ros comands the wheelchair to turn at a certain speed*/
        else if(commandRead.angular.z != 0)                               
        {                         
                smart.pid_twistA();                                       //Updates the twist angular velocity of chair
        } 
        /* If Ros does not give veloc
        ity comands*/
        else
        {
            smart.stop();                                                 //Stops the chair
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
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.linear.z = 0;
    
        /* Publishes Twist angular velocity of Wheelchair */
        odom.twist.twist.angular.x = 0;
        odom.twist.twist.angular.y = 0;
        odom.twist.twist.angular.z = smart.getTwistZ()*3.1415926/180;
    
        /* Allows for Odometry to be published and sent to ROS */
        chatter.publish(&odom);
        //chatter2.publish(&commandRead);
        
        /*Checks for incoming messages */
        nh.spinOnce();                
}    
