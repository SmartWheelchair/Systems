#include "wheelchair.h"
 
bool manual_drive = false;                                                             // Variable changes between joystick and auto drive
double curr_yaw, curr_velS;                                                            // Variable that contains current relative angle
double encoder_distance;                                                               // Keeps distanse due to original position
 
volatile double Setpoint, Output, Input, Input2;                                       // Variables for PID
volatile double pid_yaw, Distance, Setpoint2, Output2, encoder_distance2;              // Variables for PID
volatile double vIn, vOut, vDesired;
volatile double vInS, vOutS, vDesiredS;
volatile double yIn, yOut, yDesired;

double dist_old, curr_pos;

 
PID myPID(&pid_yaw, &Output, &Setpoint, 5.5, .00, 0.0036, P_ON_E, DIRECT);             // Angle PID object constructor
PID myPIDDistance(&Input, &Output, &Setpoint, 5.5, .00, 0.002, P_ON_E, DIRECT);        // Distance PID object constructor
PID PIDVelosity(&vIn, &vOut, &vDesired, 5.5, .00, .002, P_ON_E, DIRECT);
PID PIDSlaveV(&vInS, &vOutS, &vDesiredS, 5.5, .00, .002, P_ON_E, DIRECT);
PID PIDAngularV(&yIn, &yOut, &yDesired, 5.5, .00, .002, P_ON_E, DIRECT);
 

/* Thread measures current angular position */
void Wheelchair::compass_thread() 
{    
     curr_yaw = imu->yaw();
     z_angular = curr_yaw;
}

/* Thread measures velocity of wheels and distance traveled */
void Wheelchair::velosity_thread() 
{
    curr_vel = wheel->getVelosity();
    curr_velS = wheelS->getVelosity();
    curr_pos = wheel->getDistance(53.975);
}

/* Constructor for Wheelchair class */
Wheelchair::Wheelchair(PinName xPin, PinName yPin, Serial* pc, Timer* time, QEI* qei, QEI* qeiS)
{
    x_position = 0;
    y_position = 0;
    /* Initializes X and Y variables to Pins */
    x = new PwmOut(xPin);                                                               
    y = new PwmOut(yPin);
    /* Initializes IMU Library */
    imu = new chair_BNO055(pc, time);
    Wheelchair::stop();                                                                 // Wheelchair is initially stationary
    imu->setup();                                                                       // turns on the IMU
    out = pc;                                                                           // "out" is called for serial monitor
    wheelS = qeiS;                                                                      // "wheel" is called for encoder
    wheel = qei;          
    out->printf("wheelchair setup done \r\n");                                          // Make sure it initialized; prints in serial monitor
    ti = time;
    myPID.SetMode(AUTOMATIC);                                                           // set PID to automatic mode
}

/* Move wheelchair with joystick on manual mode */
void Wheelchair::move(float x_coor, float y_coor)                                
{
  /* Scales one joystick measurement to the chair's joystick measurement */
    float scaled_x = ((x_coor * 1.6f) + 1.7f)/3.3f;
    float scaled_y = (3.3f - (y_coor * 1.6f))/3.3f;

  /* Sends the scaled joystic values to the chair */
    x->write(scaled_x);                                                         
    y->write(scaled_y);
}
 
/* Automatic mode: move forward and update x,y coordinate sent to chair */
void Wheelchair::forward()                                                              
{
    x->write(high);
    y->write(def+offset);
}
 
/* Automatic mode: move in reverse and update x,y coordinate sent to chair */
void Wheelchair::backward()                    
{
    x->write(low);
    y->write(def);
}
 
/* Automatic mode: move right and update x,y coordinate sent to chair */
void Wheelchair::right()                                                             
{
    x->write(def);
    y->write(low);
}

 /* Automatic mode: move left and update x,y coordinate sent to chair */
void Wheelchair::left()                                                               
{
    x->write(def);
    y->write(high);
}
 
/* Stop the wheelchair */
void Wheelchair::stop()                                                       
{
    x->write(def);
    y->write(def);
}

/* Counter-clockwise is -
 * Clockwise is +
 * Range of deg: 0 to 360
 * This constructor takes in an angle from user and adjusts for turning right 
 */
void Wheelchair::pid_right(int deg)
{
    bool overturn = false;                                                              //Boolean if angle over 360˚
    
    out->printf("pid right\r\r\n");                                
    x->write(def);                                                                      // Update x sent to chair to be stationary
    Setpoint = curr_yaw + deg;                                                          // Relative angle we want to turn
    pid_yaw = curr_yaw;                                                                 // Sets pid_yaw to angle input from user
    
    /* Turns on overturn boolean if setpoint over 360˚ */
    if(Setpoint > 360) 
    {                                                               
        overturn = true;
    }
    
    myPID.SetTunings(5.5,0, 0.0035);                                                    // Sets the constants for P and D
    myPID.SetOutputLimits(0, def-low-.15);                                              // Limit is set to the differnce between def and low
    myPID.SetControllerDirection(DIRECT);                                               // PID mode: Direct
    
    /* PID stops when approaching a litte less than desired angle */ 
    while(pid_yaw < Setpoint - 3)
    {                                       
        /* PID is set to correct angle range if angle greater than 360˚*/
        if(overturn && curr_yaw < Setpoint-deg-1)
        {
            pid_yaw = curr_yaw + 360;  
        }
        else 
        {
            pid_yaw = curr_yaw;
        }
            
        myPID.Compute();                                                                // Does PID calculations
        double tempor = -Output+def;                                                    // Temporary value with the voltage output
        y->write(tempor);                                                               // Update y sent to chair 
        
        /* Prints to serial monitor the current angle and setpoint */
        out->printf("curr_yaw %f\r\r\n", curr_yaw);                              
        out->printf("Setpoint = %f \r\n", Setpoint);
 
        wait(.05);                                                                      // Small delay (milliseconds)
    }
 
    /* Saftey stop for wheelchair */
    Wheelchair::stop();                                                                 
    out->printf("done \r\n");
}
 
/* Counter-clockwise is -
 * Clockwise is +
 * Range of deg: 0 to 360
 * This constructor takes in an angle from user and adjusts for turning left
 */
void Wheelchair::pid_left(int deg)                                                      
{
    bool overturn = false;                                                              //Boolean if angle under 0˚
    
    out->printf("pid Left\r\r\n");                                                     
    x->write(def);                                                                      // Update x sent to chair to be stationary
    Setpoint = curr_yaw - deg;                                                          // Relative angle we want to turn
    pid_yaw = curr_yaw;                                                                 // Sets pid_yaw to angle input from user
 
    /* Turns on overturn boolean if setpoint less than 0˚ */
    if(Setpoint < 0) 
    {                                                                 
        overturn = true;
    }
 
    myPID.SetTunings(5,0, 0.004);                                                       // Sets the constants for P and D
    myPID.SetOutputLimits(0,high-def-.12);                                              //Limit is set to the differnce between def and low
    myPID.SetControllerDirection(REVERSE);                                              // PID mode: Reverse
 
    /* PID stops when approaching a litte more than desired angle */
    while(pid_yaw > Setpoint+3)
    {                                                        
       /* PID is set to correct angle range if angle less than 0˚ */                                                                          
       if(overturn && curr_yaw > Setpoint+deg+1) 
       {
          pid_yaw = curr_yaw - 360;
       }
       else 
       {
          pid_yaw = curr_yaw;
       }
     
        myPID.Compute();                                                                // Does PID calculations
        double tempor = Output+def;                                                     // Temporary value with the voltage output
        y->write(tempor);                                                               // Update y sent to chair
     
        /* Prints to serial monitor the current angle and setpoint */
        out->printf("curr_yaw %f\r\n", curr_yaw);
        out->printf("Setpoint = %f \r\n", Setpoint);
     
        wait(.05);                                                                      // Small delay (milliseconds)
    }
 
   /* Saftey stop for wheelchair */
    Wheelchair::stop();                                                                 
    out->printf("done \r\n");

}
 
/* This constructor determines whether to turn left or right */
void Wheelchair::pid_turn(int deg) 
{    
 
   /* Sets angle to coterminal angle for left turn if deg > 180
    * Sets angle to coterminal angle for right turn if deg < -180
    */
    if(deg > 180)
    {                                                                   
        deg -= 360;
    }
    else if(deg < -180)
    {                                    
        deg +=360;
    }  
    
    /* Makes sure angle inputted to function is positive */
    int turnAmt = abs(deg);
 
    /* Calls PID_right if deg > 0, else calls PID_left if deg < 0 */
    if(deg >= 0)
    {
        Wheelchair::pid_right(turnAmt);                                                
    }
    else
    {
        Wheelchair::pid_left(turnAmt);                        
    }

}

/* This constructor takes in distance to travel and adjust to move forward */
void Wheelchair::pid_forward(double mm)
{
    mm -= 20;                                                                           // Makes sure distance does not overshoot
    Input = 0;                                                                          // Initializes input to zero: Test latter w/o
    wheel->reset();                                                                     // Resets encoders so that they start at 0
 
    out->printf("pid foward\r\n");
 
    double tempor;                                                                      // Initializes Temporary variable for x input
    Setpoint = mm;                                                                      // Initializes the setpoint to desired value
 
    myPIDDistance.SetTunings(5.5,0, 0.0015);                                            // Sets constants for P and D
    myPIDDistance.SetOutputLimits(0,high-def-.15);                                      // Limit set to difference between high and def 
    myPIDDistance.SetControllerDirection(DIRECT);                                       // PID mode: Direct
 
    y->write(def+offset);                                                               // Update y to make chair stationary
    
    /* Chair stops moving when Setpoint is reached */
    while(Input < Setpoint){      
     
        if(out->readable())                                                             // Emergency Break
        {                                                    
            break;
        }

        Input = wheel->getDistance(53.975);                                             // Gets distance from Encoder into PID
        wait(.05);                                                                      // Slight Delay: *****Test without
        myPIDDistance.Compute();                                                        // Compute distance traveled by chair
 
        tempor = Output + def;                                                          // Temporary output variable                    
        x->write(tempor);                                                               // Update x sent to chair
     
        /* Prints to serial monitor the distance traveled by chair */
        out->printf("distance %f\r\n", Input);
        }
    
}   

/* This constructor returns the relative angular position of chair */
double Wheelchair::getTwistZ()
{
    return imu->gyro_z();
}   

/* This constructor computes the relative angle for Twist message in ROS */
void Wheelchair::pid_twistA()
{
    /* Initialize variables for angle and update x,y sent to chair */
    char c;
    double temporA = def;
    y->write(def); 
    x->write(def); 
 
    PIDAngularV.SetTunings(.00015,0, 0.00);                                             // Sets the constants for P and D
    PIDAngularV.SetOutputLimits(-.1, .1);                                               // Limit set to be in range specified
    PIDAngularV.SetControllerDirection(DIRECT);                                         // PID mode: Direct
 
    /* Computes angular position of wheelchair while turning */
    while(1)
    {
        yDesired = angularV;
     
        /* Update and set all variable so that the chair is stationary
         * if the desired angle is zero
         */
        if(yDesired == 0)
        {
            x->write(def);
            y->write(def);
            yDesired = 0;
            return;
        }
         
        /* Continuously updates with current angle measured by IMU */
        yIn = imu->gyro_z(); 
        PIDAngularV.Compute();
        temporA += yOut;                                                                // Temporary value with the voltage output
        y->write(temporA);                                                              // Update y sent to chair
     
        //out->printf("temporA: %f, yDesired %f, angle: %f\r\n", temporA, yDesired, imu->gyro_z());
        wait(.05);                                                                      // Small delay (milliseconds)
    }
 
}    

/* This constructor computes the relative velocity for Twist message in ROS */
void Wheelchair::pid_twistV()
{
    /* Initializes variables as default */
    double temporV = def;
    double temporS = def;
    vDesiredS = 0;
    x->write(def);
    y->write(def);
    wheel->reset();                                                                     // Resets the encoders
 
    PIDVelosity.SetTunings(.00005,0, 0.00);                                             // Sets the constants for P and D
    PIDSlaveV.SetTunings(.004,0.000001, 0.000001);                                      // Sets the constants for P and D
    PIDVelosity.SetOutputLimits(-.005, .005);                                           // Limits to the range specified
    PIDSlaveV.SetOutputLimits(-.002, .002);                                             // Limits to the range specified
 
    /* PID mode: Direct */
    PIDVelosity.SetControllerDirection(DIRECT); 
    PIDSlaveV.SetControllerDirection(DIRECT); 
 
    while(1)
    {
        test1 = linearV*100;
        vel = curr_vel;
        vDesired = linearV*100;
     
         /* Update and set all variable so that the chair is stationary
         * if the velocity is zero
         */
        if(linearV == 0)
        {
            x->write(def);
            y->write(def);

            vel = 0;
            vDesired = 0;
            dist_old = 0;
            return;
        }
     
        if(vDesired >= 0)
        {
            PIDVelosity.SetTunings(.000004,0, 0.00);                                    // Sets the constants for P and D
            PIDVelosity.SetOutputLimits(-.002, .002);                                   // Limits to the range specified
        }
        else
        {
            PIDVelosity.SetTunings(.000015,0, 0.00);                                    // Sets the constants for P and D
            PIDVelosity.SetOutputLimits(-.0005, .0005);                                 // Limits to range specified
        }    
        
        /* Sets maximum value of variable to 1 */
        if(temporV >= 1)
        {
            temporV = 1;
        }
     
        /* Scales and makes some adjustments to velocity */
        vIn = curr_vel*100;
        vInS = curr_vel-curr_velS;
        PIDVelosity.Compute();
        PIDSlaveV.Compute();
        temporV += vOut;
        temporS += vOutS;
     
        /* Updates x,y sent to Wheelchair and for Odometry message in ROS */
        x->write(temporV);
        test2 = temporV;
        y->write(temporS);
        //out->printf("Velosity: %f, Velosity2: %f, temporV %f, temporS %f\r\n", curr_vel, curr_velS, temporV, temporS);
        Wheelchair::odomMsg();
        wait(.01);                                                                      // Small delay (milliseconds)
    }
}

/* This constructor calculates the relative position of the chair everytime the encoders reset
 * by setting its old position as the origin to calculate the new position
 */
void Wheelchair::odomMsg()
{
    double dist_new = curr_pos;
    double dist = dist_new-dist_old;
    double temp_x = dist*sin(z_angular*3.14159/180);
    double temp_y = dist*cos(z_angular*3.14159/180);
    
    x_position += temp_x;
    y_position += temp_y;
    
    dist_old = dist_new;
 } 

/* This constructor prints the Odometry message to the serial monitor */
 void Wheelchair::showOdom()
 {
     out->printf("x %f, y %f, angle %f", x_position, y_position, z_angular);
 }

/* This constructor returns the approximate distance based on the wheel diameter */
float Wheelchair::getDistance()
{
    return wheel->getDistance(Diameter);
}

/* This constructor resets the wheel encoder's */
void Wheelchair::resetDistance()
{
    wheel->reset();
}


/*Predetermined paths For Demmo*/    
void Wheelchair::desk() 
{
    Wheelchair::pid_forward(5461);
    Wheelchair::pid_right(87);
    Wheelchair::pid_forward(3658);
    Wheelchair::pid_right(87);
    Wheelchair::pid_forward(3658);
}
 
void Wheelchair::kitchen() 
{
    Wheelchair::pid_forward(5461);
    Wheelchair::pid_right(87);
    Wheelchair::pid_forward(3658);
    Wheelchair::pid_left(90);
    Wheelchair::pid_forward(305);
}
 
void Wheelchair::desk_to_kitchen()
{
    Wheelchair::pid_right(180);
    Wheelchair::pid_forward(3700);
}
 
 
