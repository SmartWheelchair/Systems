#include "chair_BNO055.h"
/* wrapper class for BNO055 imu for the wheelchair*/
static float total_yaw = 0;
static float minGyroZ;
static float x_speed = 0;
static float x_displacement = 0;

/*
 * constructs chair_BNO055 to specify timer and serial output location
 * uses default pins set in chair_BNO055.h
 * Params: 
 * 	Serial* out: serial to output
 * 	Timer* time: timer used for delta time calculation
 */
chair_BNO055::chair_BNO055(Serial* out, Timer* time)
{
    imu = new BNO055(SDA, SCL); 
    usb = out;
    t = time;
    start = false;
}

/*
 * constructs chair_BNO055 to specify timer and serial output location
 * sets imu pins with given parameters
 * Params: 
 *	PinName sda_pin: sda pin
 *	PinName scl_pin: scl pin
 * 	Serial* out: serial to output
 * 	Timer* time: timer used for delta time calculation
 */
chair_BNO055::chair_BNO055(PinName sda_pin, PinName scl_pin, Serial* out, Timer* time)
{
    imu = new BNO055(sda_pin, scl_pin);
    usb = out;
    t = time;
    start = false;
}

/*runs imu initialization to set appropriate units and check that imu is working*/
void chair_BNO055::setup()
{
    imu->reset();
    usb->printf("Bosch Sensortec BNO055 test program on \r\n");//" __DATE__ "/" __TIME__ "\r\n");
    while (imu->check() == 0) {
        usb->printf("Bosch BNO055 is NOT available!!\r\n");
        wait(.5);
    }
    usb->printf("BNO055 found\r\n\r\n");
    usb->printf("Chip          ID: %0z\r\n",imu->ID.id);
    usb->printf("Accelerometer ID: %0z\r\n",imu->ID.accel);
    usb->printf("Gyroscope     ID: %0z\r\n",imu->ID.gyro);
    usb->printf("Magnetometer  ID: %0z\r\n\r\n",imu->ID.mag);
    usb->printf("Firmware version v%d.%0d\r\n",imu->ID.sw[0],imu->ID.sw[1]);
    usb->printf("Bootloader version v%d\r\n\r\n",imu->ID.bootload);
    imu->set_accel_units(MPERSPERS);
    imu->setmode(OPERATION_MODE_AMG);
    imu->read_calibration_data();
    imu->write_calibration_data();
    imu->set_angle_units(DEGREES);

    t->start();
}

/* calibrates yaw*/
void chair_BNO055::calibrate_yaw() {
    //get start time to calculate delta
    float start = t->read();
    
    //yaw = delta angle/ delta time
    while( t->read()- start <= CAL_TIME ){
        total_yaw = boxcar(chair_BNO055::angle_north());
        usb->printf("total_yaw %f\n", total_yaw);
        }
    }

/* gets x component of acceleration vector*/
double chair_BNO055::accel_x()
{
    imu->get_accel();
    return (double)imu->accel.x;
}

/* gets y component of acceleration vector*/
double chair_BNO055::accel_y()
{
    imu->get_accel();
    return (double)imu->accel.y;
}

/* gets z component of acceleration vector*/
double chair_BNO055::accel_z()
{
    imu->get_accel();
    return (double)imu->accel.z;
}

/* gets x component of gyroscope*/
double chair_BNO055::gyro_x()
{
    imu->get_gyro();
    return (double)imu->gyro.x;
}

/* gets y component of gyroscope*/
double chair_BNO055::gyro_y()
{
    imu->get_gyro();
    return (double)imu->gyro.y;
}

/* gets z component of gyroscope*/
double chair_BNO055::gyro_z()
{
    imu->get_gyro();
    return lowPass(imu->gyro.z);
}

/* reads magnometer*/
double chair_BNO055::angle_north()
{
    imu->get_mag();
    float x = imu->mag.x - 520;
    float y = imu->mag.y;

    float result = x/y;

    float angleToNorth;
    if(imu->mag.y>0)
        angleToNorth = 90.0 - atan(result)*180/PI;
    else if(imu->mag.y<0)
        angleToNorth = 270.0 - atan(result)*180/PI;
    else if(y == 0 && x <= 0)
        angleToNorth = 180;
    else if(y == 0 && x > 0)
        angleToNorth = 0;

    return (double)angleToNorth;
}

/* calculates roll*/
double chair_BNO055::roll()
{
    /*gets accelerometer and gyroscope values*/
    imu->get_accel();
    imu->get_gyro();
 
    /*calculates roll*/
    float roll = atan2(-imu->accel.x ,( sqrt((imu->accel.y * imu->accel.y) +
                                        (imu->accel.z * imu->accel.z))));
    //converts from randians to degrees
    roll = roll*57.3;

    //resets timer so correct delta time may be calculated
    t->reset();

    return (double)roll;
}

/* calculates pitch*/
double chair_BNO055::pitch()
{
    /*gets accelerometer and gyroscope values*/
    imu->get_accel();
    imu->get_gyro();

    //calculates pitch in radians
    float pitch = atan2 (imu->accel.y ,( sqrt ((imu->accel.x * imu->accel.x) +
                                         (imu->accel.z * imu->accel.z))));
    //converts from radians to degrees
    pitch = pitch*57.3;
	
    //resets timer so correct delta time may be calculated
    t->reset();

    return (double)pitch;
}

/* calculates yaw*/
double chair_BNO055::yaw()
{
    // gets z component of gyroscope
    float gyroZ = chair_BNO055::gyro_z();

    //only change yaw if it is not noise
    if(abs(gyroZ) >= .3) {
        total_yaw = (total_yaw - t->read()*gyroZ);
        //usb->printf("gyroZ %f\n", gyroZ);
    }
    t->reset();

    //reset yaw for turning 
    //if yaw goes over 360, return to 0
    if(total_yaw > 360)
        total_yaw -= 360;
    //if yaw goes below 0, return to 360
    if(total_yaw < 0)
        total_yaw += 360;

     //usb->printf("yaw %f\n", total_yaw);
    return (double)total_yaw;
}
