***************************************************************
* Arduino PID Library - Version 1.2.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under the MIT License
***************************************************************
#ifndef PID_H
#define PID_H
 
#include "mbed.h"

class PID 
{ 
 
public: 
      #define AUTOMATIC    1    // To select Automatic PID mode
      #define MANUAL    0    // To select Manual PID mode
      #define DIRECT    0    // To select Direct PID mode
      #define REVERSE    1    // To select Direct PID mode
      #define P_ON_M    0    // To select proportional on m mode
      #define P_ON_E    1    // To select proportional on m mode
 
    // Sets up clock to be used in the library
    Timer PIDtimer;
 
    // Constructor that takes in the input, output, setpoint, Proportional constant,
    // integral constant, derivative constant, proportional mode, direction of controller
    PID(volatile double* Input, volatile double* Output, volatile double* Setpoint,
        double Kp, double Ki, double Kd, int POn, int ControllerDirection); 
 
    // * sets PID to either Manual (0) or Auto (non-0)
    void SetMode(int Mode);               

    // * performs the PID calculation.  it should be
    //   called every time loop() cycles. ON/OFF and
    //   calculation frequency can be set using SetMode
    //   SetSampleTime respectively
    bool Compute();                       

    // * clamps the output to a specific range. 0-255 by default, but
    //   it's likely the user will want to change this depending on
    //   the application
    void SetOutputLimits(double, double); 
    


    //available but not commonly used functions ********************************************************
 
    // * While most users will set the tunings once in the 
    //   constructor, this function gives the user the option
    //   of changing tunings during runtime for Adaptive control
    void SetTunings(double, double, double);               
                                          
    // * overload for specifying proportional mode
    void SetTunings(double, double, double, int);             

    // * Sets the Direction, or "Action" of the controller. DIRECT
    //   means the output will increase when error is positive. REVERSE
    //   means the opposite.  it's very unlikely that this will be needed
    //   once it is set in the constructor.*/
    void SetControllerDirection(int);     
                                                                                
    // * sets the frequency, in Milliseconds, with which           
    //   the PID calculation is performed.  default is 100
    void SetSampleTime(int);              
                                          
                                                                                
    //Display functions ****************************************************************
    double GetKp();                       // These functions query the pid for interal values.
    double GetKi();                       //  they were created mainly for the pid front-end,
    double GetKd();                       // where it's important to know what is actually 
    int GetMode();                        //  inside the PID.
    int GetDirection();                   //

  private:
    //Initialize the PID Calculations
    void Initialize();
    
    double dispKp;              // * we'll hold on to the tuning parameters in user-entered 
    double dispKi;              //   format for display purposes
    double dispKd;              //
    
    double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

    int controllerDirection;    // * What direction should the PID controlls follow
    int pOn;                    // * Variable that keeps the proporcional mode

    volatile double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    volatile double *myOutput;             //   This creates a hard link between the variables and the 
    volatile double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                           //   what these values are.  with pointers we'll just know.
              
    double lastTime;                       // keeps track of the previews measurement
    double outputSum, lastInput;           // Output sum is used for internal calculation
                                           // Last imput is used to tell the difference between imputs

    double SampleTime;                     // How often are you doing the compute function
    double outMin, outMax;                 // variables of the output limits
    bool inAuto, pOnE;                     // keeps track of what modes the PID is in
    
};

#endif

