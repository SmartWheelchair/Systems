#include "chair_BNO055.h"

static float total_yaw = 0;
static float minGyroZ;
static float x_speed = 0;
static float x_displacement = 0;
chair_BNO055::chair_BNO055(Serial* out, Timer* time)
{
    imu = new BNO055(SDA, SCL);
    usb = out;
    t = time;
    start = false;
}

chair_BNO055::chair_BNO055(PinName sda_pin, PinName scl_pin, Serial* out, Timer* time)
{
    imu = new BNO055(sda_pin, scl_pin);
    usb = out;
    t = time;
    start = false;
}

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

void chair_BNO055::calibrate_yaw() {
    float start = t->read();
    
    while( t->read()- start <= CAL_TIME ){
        total_yaw = boxcar(chair_BNO055::angle_north());
        usb->printf("total_yaw %f\n", total_yaw);
        }
    }


double chair_BNO055::accel_x()
{
    imu->get_accel();
    return (double)imu->accel.x;
}

double chair_BNO055::accel_y()
{
    imu->get_accel();
    return (double)imu->accel.y;
}

double chair_BNO055::accel_z()
{
    imu->get_accel();
    return (double)imu->accel.z;
}

double chair_BNO055::gyro_x()
{
    imu->get_gyro();
    return (double)imu->gyro.x;
}

double chair_BNO055::gyro_y()
{
    imu->get_gyro();
    return (double)imu->gyro.y;
}

double chair_BNO055::gyro_z()
{
    imu->get_gyro();
    return lowPass(imu->gyro.z);
}

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

double chair_BNO055::roll()
{
    imu->get_accel();
    imu->get_gyro();

    float roll = atan2(-imu->accel.x ,( sqrt((imu->accel.y * imu->accel.y) +
                                        (imu->accel.z * imu->accel.z))));
    roll = roll*57.3;

    t->reset();

    return (double)roll;
}

double chair_BNO055::pitch()
{
    imu->get_accel();
    imu->get_gyro();

    float pitch = atan2 (imu->accel.y ,( sqrt ((imu->accel.x * imu->accel.x) +
                                         (imu->accel.z * imu->accel.z))));
    pitch = pitch*57.3;

    t->reset();

    return (double)pitch;
}

double chair_BNO055::yaw()
{
    float gyroZ = chair_BNO055::gyro_z();

    if(abs(gyroZ) >= .3) {
        total_yaw = (total_yaw - t->read()*gyroZ);
        //usb->printf("gyroZ %f\n", gyroZ);
    }
    t->reset();

    if(total_yaw > 360)
        total_yaw -= 360;
    if(total_yaw < 0)
        total_yaw += 360;

     //usb->printf("yaw %f\n", total_yaw);
    return (double)total_yaw;
}