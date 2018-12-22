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
 
/* Automatic mode: move forward and update x,y coordinate */
void Wheelchair::forward()                                                              
{
    x->write(high);
    y->write(def+offset);
}
 
/* Automatic mode: move in reverse and update x,y coordinate */
void Wheelchair::backward()                    
{
    x->write(low);
    y->write(def);
}
 
/* Automatic mode: move right and update x,y coordinate */
void Wheelchair::right()                                                             
{
    x->write(def);
    y->write(low);
}

 /* Automatic mode: move left and update x,y coordinate */
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
 * This function takes in an angle from user and adjusts for turning right 
 */
void Wheelchair::pid_right(int deg)
{
    bool overturn = false;                                                              //Boolean if we have to determine coterminal angle
    
    out->printf("pid right\r\r\n");                                
    x->write(def);                                                                      // Update x for not moving foward or reverse
    Setpoint = curr_yaw + deg;                                                          // Relative angle we want to turn
    pid_yaw = curr_yaw;                                                                 // Sets pid_yaw input to current angle
    
    if(Setpoint > 360) {                                                                //Turns on overturn boolean if setpoint over 360˚
        overturn = true;
    }
    
    myPID.SetTunings(5.5,0, 0.0035);                                                    // Sets the constants for P and D
    myPID.SetOutputLimits(0, def-low-.15);                                              // Limit is set to the differnce between def and low
    myPID.SetControllerDirection(DIRECT);                                               // PID mode Direct
    
    while(pid_yaw < Setpoint - 3){                                                      // Tells PID to stop when reaching
                                                                                        // a little less than desired angle
        if(overturn && curr_yaw < Setpoint-deg-1)                                       // Sets PID yaw to coterminal angle if necesary
        {
            pid_yaw = curr_yaw + 360;
        }   
        else
            pid_yaw = curr_yaw;
            
        myPID.Compute();                                                                // Does PID calculations
        double tempor = -Output+def;                                                    // Temporary value with the voltage output
        y->write(tempor);                                                               // Sends to chair y output command
        
        out->printf("curr_yaw %f\r\r\n", curr_yaw);                              
        out->printf("Setpoint = %f \r\n", Setpoint);
 
        wait(.05);                                                                      // Small delay
        }
    Wheelchair::stop();                                                                 // Safety Stop
    out->printf("done \r\n");
}
 

/* This function takes in an angle from user and adjusts for turning left */
void Wheelchair::pid_left(int deg)                                                      
{
    bool overturn = false;                                                              //Boolean if we have to turn under relative 0˚
    
    out->printf("pid Left\r\r\n");                                                     
    x->write(def);                                                                      // Not moving fowards or reverse
    Setpoint = curr_yaw - deg;                                                          // Relative angle we want to turn
    pid_yaw = curr_yaw;                                                                 // Sets input to current angle(pid_yaw = input)
    if(Setpoint < 0) 
    {                                                                  //Turns on overturn boolean if setpoint under 0˚
        overturn = true;
    }
    myPID.SetTunings(5,0, 0.004);                                                       // Sets the constants for P and D
    myPID.SetOutputLimits(0,high-def-.12);                                              // Limits to the differnce between High and Def
    myPID.SetControllerDirection(REVERSE);                                              // PID mode Reverse
    while(pid_yaw > Setpoint+3){                                                        // Tells PID to stop when reaching
                                                                                        // a little more than desired angle
       if(overturn && curr_yaw > Setpoint+deg+1)                                        // Sets PID yaw to coterminal angle if necesary
       {
          pid_yaw = curr_yaw - 360;
        }   
        else
            pid_yaw = curr_yaw;
            
        myPID.Compute();                                                                // Does PID calculations
        double tempor = Output+def;                                                     // Temporary value with the voltage output
        y->write(tempor);                                                               // Sends to chair y output command
        
        out->printf("curr_yaw %f\r\n", curr_yaw);
        wait(.05);                                                                      // Small Delay
        }
    Wheelchair::stop();                                                                 // Safety Stop
}
 
void Wheelchair::pid_turn(int deg) {                                                    // Determine wether we are turn right or left
 
    if(deg > 180) {                                                                     // If deg > 180 turn left: coterminal angle
        deg -= 360;
    }
 
    else if(deg < -180) {                                                               // If deg < -180 turn right: coterminal angle
        deg+=360;
    }  
    
    int turnAmt = abs(deg);                                                             // Makes sure input angle is positive
 
    if(deg >= 0){
        Wheelchair::pid_right(turnAmt);                                                 // Calls PID right if positive degree
    }
    else {
        Wheelchair::pid_left(turnAmt);                                                  // Calls PID left if negative degree
    }
}
void Wheelchair::pid_forward(double mm)
{
    mm -= 20;                                                                           // Makes sure distance does not overshoot
    Input = 0;                                                                          // Initializes imput to cero: Test latter w/o
    wheel->reset();                                                                     // Resets encoders so that they start at 0
    out->printf("pid foward\r\n");
 
    double tempor;                                                                      // Initializes Temporary variable for x input
    Setpoint = mm;                                                                      // Initializes the setpoint to desired value
 
    myPIDDistance.SetTunings(5.5,0, 0.0015);                                            // Sets constants for P and D
    myPIDDistance.SetOutputLimits(0,high-def-.15);                                      // Limits to the differnce between High and Def 
    myPIDDistance.SetControllerDirection(DIRECT);                                       // PID to Direct
    y->write(def+offset);                                                               // Sets chair to not turn
    
    while(Input < Setpoint){                                                            // Stop moving when reaching setpoint
    
        if(out->readable())                                                             // Emergency Break
            break;
            
        Input = wheel->getDistance(53.975);                                             // Gets Distance from Encoder onto PID
        wait(.05);                                                                      // Slight Delay: *****Test without
        myPIDDistance.Compute();                                                        // Compute Output for chair
 
        tempor = Output + def;                                                          // Temporary output variable                    
        x->write(tempor);                                                               // Sends to chair x output
        out->printf("distance %f\r\n", Input);
        }
    
}   
double Wheelchair::getTwistZ()
{
    return imu->gyro_z();

}   
void Wheelchair::pid_twistA()
{
    char c;
    double temporA = def;
    y->write(def); 
    x->write(def); 
 
    PIDAngularV.SetTunings(.00015,0, 0.00);                                                    // Sets the constants for P and D
    PIDAngularV.SetOutputLimits(-.1, .1);                                              // Limits to the differnce between def and low
    PIDAngularV.SetControllerDirection(DIRECT);                                               // PID mode Direct
    while(1)
    {
        yDesired = angularV;
        if(yDesired == 0)
        {
            x->write(def);
            y->write(def);
            yDesired = 0;
            return;
        }
          
        yIn = imu->gyro_z(); 
        PIDAngularV.Compute();
        temporA += yOut;                                                     // Temporary value with the voltage output
        y->write(temporA); 
        //out->printf("temporA: %f, yDesired %f, angle: %f\r\n", temporA, yDesired, imu->gyro_z());
        wait(.05);
    }
}    
void Wheelchair::pid_twistV()
{
    double temporV = def;
    double temporS = def;
    vDesiredS = 0;
    x->write(def);
    y->write(def);
    wheel->reset();
    PIDVelosity.SetTunings(.00005,0, 0.00);                                                    // Sets the constants for P and D
    PIDSlaveV.SetTunings(.004,0.000001, 0.000001);                                                    // Sets the constants for P and D
    PIDVelosity.SetOutputLimits(-.005, .005);                                              // Limits to the differnce between def and low
    PIDSlaveV.SetOutputLimits(-.002, .002);                                              // Limits to the differnce between def and low
    PIDVelosity.SetControllerDirection(DIRECT); 
    PIDSlaveV.SetControllerDirection(DIRECT); 
    while(1)
    {
        test1 = linearV*100;
        vel = curr_vel;
        vDesired = linearV*100;
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
            PIDVelosity.SetTunings(.000004,0, 0.00);                                                    // Sets the constants for P and D
            PIDVelosity.SetOutputLimits(-.002, .002);                                              // Limits to the differnce between def and low
        }
        else
        {
            PIDVelosity.SetTunings(.000015,0, 0.00);                                                    // Sets the constants for P and D
            PIDVelosity.SetOutputLimits(-.0005, .0005);                                              // Limits to the differnce between def and low
        }    
        if(temporV >= 1)
            temporV = 1;
        vIn = curr_vel*100;
        vInS = curr_vel-curr_velS;
        PIDVelosity.Compute();
        PIDSlaveV.Compute();
        temporV += vOut;
        temporS += vOutS;
        x->write(temporV);
        test2 = temporV;
        y->write(temporS);
        //out->printf("Velosity: %f, Velosity2: %f, temporV %f, temporS %f\r\n", curr_vel, curr_velS, temporV, temporS);
        Wheelchair::odomMsg();
        wait(.01);
    }
}
void Wheelchair::odomMsg(){
    double dist_new = curr_pos;
    double dist = dist_new-dist_old;
    double temp_x = dist*sin(z_angular*3.14159/180);
    double temp_y = dist*cos(z_angular*3.14159/180);
    
    x_position += temp_x;
    y_position += temp_y;
    
    dist_old = dist_new;
 } 
 void Wheelchair::showOdom(){
     out->printf("x %f, y %f, angle %f", x_position, y_position, z_angular);
}
float Wheelchair::getDistance() {
    return wheel->getDistance(Diameter);
    }
    
void Wheelchair::resetDistance(){
    wheel->reset();
    }
/*Predetermined paths For Demmo*/    
void Wheelchair::desk() {
    Wheelchair::pid_forward(5461);
    Wheelchair::pid_right(87);
    Wheelchair::pid_forward(3658);
    Wheelchair::pid_right(87);
    Wheelchair::pid_forward(3658);
    }
 
void Wheelchair::kitchen() {
    Wheelchair::pid_forward(5461);
    Wheelchair::pid_right(87);
    Wheelchair::pid_forward(3658);
    Wheelchair::pid_left(90);
    Wheelchair::pid_forward(305);
    }
 
void Wheelchair::desk_to_kitchen(){
    Wheelchair::pid_right(180);
    Wheelchair::pid_forward(3700);
    }
 
 
