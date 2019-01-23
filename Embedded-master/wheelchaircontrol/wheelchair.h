#ifndef wheelchair
#define wheelchair

/* Importing libraries into wheelchair.h */
#include "chair_BNO055.h"
#include "PID.h"
#include "QEI.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>


/*
* Joystick has analog out of 200-700, scale values between 1.3 and 3.3
*/             
#define def (2.5f/3.3f)                 //Default axis on joystick to stay neutral; used on x and y axis
#define high 3.3f/3.3f                  //High power on joystick; used on x and y axis
#define low (1.7f/3.3f)                 //Low power on joystick; used on x and y axis
#define offset .035f                    //Joystick adjustment to be able to go straight. Chair dependent on manufactoring precision
#define process .1                      //Defines default time delay in seconds

/* Pin plug in for Nucleo-L432KC */
#define xDir D9                         //* PWM Pins */
#define yDir D10
#define Encoder1 D7                     //*Digital In Pull Up Pin */
#define Encoder2 D8
#define Diameter 31.75                  //Diameter of encoder wheel

/* Define pin for emergency stop button*/

/** Wheelchair class
 * Used for controlling the smart wheelchair
 */

class Wheelchair
{
public:
    /** Create Wheelchair Object with x,y pin for analog dc output
     * serial for printout, and timer
     */
    Wheelchair(PinName xPin, PinName yPin, Serial* pc, Timer* time, QEI* wheel, QEI* wheelS);
    
    /** Move using the joystick */
    void move(float x_coor, float y_coor);
    
    /* Turn right a certain amount of degrees using PID*/
    void pid_right(int deg);
    
    /* Turn left a certain amount of degrees using PID*/
    void pid_left(int deg);
    
    /* Drive the wheelchair forward */
    void forward();
    
    /* Drive the wheelchair in reverse*/
    void backward();
    
    /* Turn the wheelchair right*/
    void right();
    
    /* Turn the wheelchair left*/
    void left();
    
    /* Stop the wheelchair*/
    void stop();
    
    /* Functions to get IMU data*/
    void compass_thread();
    void velocity_thread();
    void rosCom_thread();
    
    /* Move x millimiters foward using PID*/
    void pid_forward(double mm);
    
    /* Obtain angular position for Twist message */
    double getTwistZ();
    
    /*  Gets the encoder distance moved since encoder reset*/
    float getDistance();
    
    /* Resets encoder*/
    void resetDistance();
    
    /* Function to determine whether we are turning left or right*/
    void pid_turn(int deg);
    
    /* Function to obtain angular and linear velocity */
    void pid_twistA();
    void pid_twistV();
    
    /* Function to publish and show Odometry message */
    void odomMsg();
    void showOdom();
    
    /* Functions with a predetermined path (demo) */
    void desk();
    void kitchen();
    void desk_to_kitchen();
    
    /* Variables for postion, angle, and velocity */
    double x_position;
    double y_position;
    double z_angular;   
    double curr_vel;
    double z_twistA;
    double linearV;
    double angularV;
    double vel;
    double test1, test2;  
    
private:
    /* Pointers for the joystick speed */
    PwmOut* x;
    PwmOut* y;
    
    chair_BNO055* imu;                  // Pointer to IMU
    Serial* out;                        // Pointer to Serial Monitor
    Timer* ti;                          // Pointer to the timer
    QEI* wheel;                         // Pointer to encoder
    QEI* wheelS;                        // Pointer to encoder
};
#endif
