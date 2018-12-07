#ifndef CHAIR_BNO055_H
#define CHAIR_BNO055_H

#include "filter.h"
#include  "mbed.h"
#include  "math.h"
#include  "BNO055.h"

#define PI 3.141593
/*#define SDA D14
#define SCL D15*/
#define SDA D4
#define SCL D5
#define SAMPLEFREQ 50
#define CAL_TIME 3

class chair_BNO055
{
public:
  chair_BNO055(Serial* out, Timer* time);
  chair_BNO055(PinName sda_pin, PinName scl_pin, Serial* out, Timer* time);
  void setup();
  double accel_x();
  double accel_y();
  double accel_z();
  double gyro_x();
  double gyro_y();
  double gyro_z();
  double angle_north();
  double yaw();
  double pitch();
  double roll();
  
  BNO055* imu;
private:
  //BNO055* imu;
  Serial* usb;
  Timer* t;
  bool start;
  void setStart();
  void calibrate_yaw();
};

#endif
