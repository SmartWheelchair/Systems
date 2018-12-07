#ifndef BNO055_H
#define BNO055_H

#include "mbed.h"
//
#define BNOAddress (0x28 << 1)
//Register definitions
/* Page id register definition */ 
#define BNO055_PAGE_ID_ADDR          0x07
/* PAGE0 REGISTER DEFINITION START*/ 
#define BNO055_CHIP_ID_ADDR          0x00 
#define BNO055_ACCEL_REV_ID_ADDR     0x01
#define BNO055_MAG_REV_ID_ADDR       0x02
#define BNO055_GYRO_REV_ID_ADDR      0x03
#define BNO055_SW_REV_ID_LSB_ADDR    0x04
#define BNO055_SW_REV_ID_MSB_ADDR    0x05
#define BNO055_BL_REV_ID_ADDR        0x06
/* Accel data register */ 
#define BNO055_ACCEL_DATA_X_LSB_ADDR 0x08
#define BNO055_ACCEL_DATA_X_MSB_ADDR 0x09
#define BNO055_ACCEL_DATA_Y_LSB_ADDR 0x0A
#define BNO055_ACCEL_DATA_Y_MSB_ADDR 0x0B
#define BNO055_ACCEL_DATA_Z_LSB_ADDR 0x0C 
#define BNO055_ACCEL_DATA_Z_MSB_ADDR 0x0D
/* Mag data register */ 
#define BNO055_MAG_DATA_X_LSB_ADDR   0x0E
#define BNO055_MAG_DATA_X_MSB_ADDR   0x0F
#define BNO055_MAG_DATA_Y_LSB_ADDR   0x10
#define BNO055_MAG_DATA_Y_MSB_ADDR   0x11
#define BNO055_MAG_DATA_Z_LSB_ADDR   0x12
#define BNO055_MAG_DATA_Z_MSB_ADDR   0x13
/* Gyro data registers */ 
#define BNO055_GYRO_DATA_X_LSB_ADDR  0x14
#define BNO055_GYRO_DATA_X_MSB_ADDR  0x15
#define BNO055_GYRO_DATA_Y_LSB_ADDR  0x16
#define BNO055_GYRO_DATA_Y_MSB_ADDR  0x17
#define BNO055_GYRO_DATA_Z_LSB_ADDR  0x18
#define BNO055_GYRO_DATA_Z_MSB_ADDR  0x19
/* Euler data registers */ 
#define BNO055_EULER_H_LSB_ADDR      0x1A
#define BNO055_EULER_H_MSB_ADDR      0x1B
#define BNO055_EULER_R_LSB_ADDR      0x1C
#define BNO055_EULER_R_MSB_ADDR      0x1D
#define BNO055_EULER_P_LSB_ADDR      0x1E
#define BNO055_EULER_P_MSB_ADDR      0x1F
/* Quaternion data registers */ 
#define BNO055_QUATERNION_DATA_W_LSB_ADDR  0x20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR  0x21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR  0x22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR  0x23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR  0x24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR  0x25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR  0x26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR  0x27
/* Linear acceleration data registers */ 
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR 0x28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR 0x29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR 0x2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR 0x2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR 0x2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR 0x2D
/* Gravity data registers */ 
#define BNO055_GRAVITY_DATA_X_LSB_ADDR      0x2E
#define BNO055_GRAVITY_DATA_X_MSB_ADDR      0x2F
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR      0x30
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR      0x31
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR      0x32
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR      0x33
/* Temperature data register */ 
#define BNO055_TEMP_ADDR                    0x34 
/* Status registers */ 
#define BNO055_CALIB_STAT_ADDR              0x35 
#define BNO055_SELFTEST_RESULT_ADDR         0x36 
#define BNO055_INTR_STAT_ADDR               0x37 
#define BNO055_SYS_CLK_STAT_ADDR            0x38 
#define BNO055_SYS_STAT_ADDR                0x39 
#define BNO055_SYS_ERR_ADDR                 0x3A 
/* Unit selection register */ 
#define BNO055_UNIT_SEL_ADDR                0x3B 
#define BNO055_DATA_SELECT_ADDR             0x3C 
/* Mode registers */ 
#define BNO055_OPR_MODE_ADDR                0x3D 
#define BNO055_PWR_MODE_ADDR                0x3E 
#define BNO055_SYS_TRIGGER_ADDR             0x3F 
#define BNO055_TEMP_SOURCE_ADDR             0x40 
/* Axis remap registers */ 
#define BNO055_AXIS_MAP_CONFIG_ADDR         0x41 
#define BNO055_AXIS_MAP_SIGN_ADDR           0x42 
/* Accelerometer Offset registers */ 
#define ACCEL_OFFSET_X_LSB_ADDR             0x55 
#define ACCEL_OFFSET_X_MSB_ADDR             0x56 
#define ACCEL_OFFSET_Y_LSB_ADDR             0x57
#define ACCEL_OFFSET_Y_MSB_ADDR             0x58 
#define ACCEL_OFFSET_Z_LSB_ADDR             0x59 
#define ACCEL_OFFSET_Z_MSB_ADDR             0x5A 
/* Magnetometer Offset registers */ 
#define MAG_OFFSET_X_LSB_ADDR               0x5B
#define MAG_OFFSET_X_MSB_ADDR               0x5C
#define MAG_OFFSET_Y_LSB_ADDR               0x5D
#define MAG_OFFSET_Y_MSB_ADDR               0x5E
#define MAG_OFFSET_Z_LSB_ADDR               0x5F
#define MAG_OFFSET_Z_MSB_ADDR               0x60
/* Gyroscope Offset registers*/ 
#define GYRO_OFFSET_X_LSB_ADDR              0x61
#define GYRO_OFFSET_X_MSB_ADDR              0x62
#define GYRO_OFFSET_Y_LSB_ADDR              0x63
#define GYRO_OFFSET_Y_MSB_ADDR              0x64
#define GYRO_OFFSET_Z_LSB_ADDR              0x65
#define GYRO_OFFSET_Z_MSB_ADDR              0x66
/* Radius registers */ 
#define ACCEL_RADIUS_LSB_ADDR               0x67 
#define ACCEL_RADIUS_MSB_ADDR               0x68 
#define MAG_RADIUS_LSB_ADDR                 0x69 
#define MAG_RADIUS_MSB_ADDR                 0x6A 

/* Page 1 registers */
#define BNO055_UNIQUE_ID_ADDR               0x50

//Definitions for unit selection
#define MPERSPERS   0x00
#define MILLIG      0x01
#define DEG_PER_SEC 0x00
#define RAD_PER_SEC 0x02
#define DEGREES     0x00
#define RADIANS     0x04
#define CENTIGRADE  0x00
#define FAHRENHEIT  0x10
#define WINDOWS     0x00
#define ANDROID     0x80

//Definitions for power mode
#define POWER_MODE_NORMAL   0x00 
#define POWER_MODE_LOWPOWER 0x01 
#define POWER_MODE_SUSPEND  0x02 

//Definitions for operating mode
#define OPERATION_MODE_CONFIG        0x00 
#define OPERATION_MODE_ACCONLY       0x01 
#define OPERATION_MODE_MAGONLY       0x02 
#define OPERATION_MODE_GYRONLY       0x03 
#define OPERATION_MODE_ACCMAG        0x04 
#define OPERATION_MODE_ACCGYRO       0x05 
#define OPERATION_MODE_MAGGYRO       0x06 
#define OPERATION_MODE_AMG           0x07 
#define OPERATION_MODE_IMUPLUS       0x08 
#define OPERATION_MODE_COMPASS       0x09 
#define OPERATION_MODE_M4G           0x0A 
#define OPERATION_MODE_NDOF_FMC_OFF  0x0B 
#define OPERATION_MODE_NDOF          0x0C 

typedef struct values{
    int16_t rawx,rawy,rawz;
    float x,y,z;
    }values;
    
typedef struct angles{
    int16_t rawroll,rawpitch,rawyaw;
    float roll, pitch, yaw;
    } angles;
    
typedef struct quaternion{
    int16_t raww,rawx,rawy,rawz;
    float w,x,y,z;
    }quaternion;

typedef struct chip{
    char id;
    char accel;
    char gyro;
    char mag;
    char sw[2];
    char bootload;
    char serial[16];
    }chip;

/** Class for operating Bosch BNO055 sensor over I2C **/
class BNO055 
{ 
public: 

/** Create BNO055 instance **/
    BNO055(PinName SDA, PinName SCL); 
    
/** Perform a power-on reset of the BNO055 **/
    void reset();
/** Check that the BNO055 is connected and download the software details 
and serial number of chip and store in ID structure **/
    bool check();
/** Turn the external timing crystal on/off **/
    void SetExternalCrystal(bool yn);
/** Set the operation mode of the sensor **/    
    void setmode(char mode);
/** Set the power mode of the sensor **/
    void setpowermode(char mode);
    
/** Set the output units from the accelerometer, either MPERSPERS or MILLIG **/
    void set_accel_units(char units);
/** Set the output units from the gyroscope, either DEG_PER_SEC or RAD_PER_SEC **/
    void set_anglerate_units(char units);
/** Set the output units from the IMU, either DEGREES or RADIANS **/
    void set_angle_units(char units);
/** Set the output units from the temperature sensor, either CENTIGRADE or FAHRENHEIT **/
    void set_temp_units(char units);
/** Set the data output format to either WINDOWS or ANDROID **/    
    void set_orientation(char units);
/** Set the mapping of the exes/directions as per page 25 of datasheet
    range 0-7, any value outside this will set the orientation to P1 (default at power up) **/
    void set_mapping(char orient);
    
/** Get the current values from the accelerometer **/
    void get_accel(void);
/** Get the current values from the gyroscope **/
    void get_gyro(void);
/** Get the current values from the magnetometer **/
    void get_mag(void);
/** Get the corrected linear acceleration **/
    void get_lia(void);
/** Get the current gravity vector **/
    void get_grv(void);
/** Get the output quaternion **/
    void get_quat(void);
/** Get the current Euler angles **/
    void get_angles(void);
/** Get the current temperature **/
    void get_temp(void);

/** Read the calibration status register and store the result in the calib variable **/
    void get_calib(void);
/** Read the offset and radius values into the calibration array**/
    void read_calibration_data(void);
/** Write the contents of the calibration array into the registers **/    
    void write_calibration_data(void);
    
/** Structures containing 3-axis data for acceleration, rate of turn and magnetic field.
    x,y,z are the scale floating point values and
    rawx, rawy, rawz are the int16_t values read from the sensors **/
    values accel,gyro,mag,lia,gravity;
/** Stucture containing the Euler angles as yaw, pitch, roll as scaled floating point
    and rawyaw, rawroll & rollpitch as the int16_t values loaded from the registers **/
    angles euler;
/** Quaternion values as w,x,y,z (scaled floating point) and raww etc... as int16_t loaded from the
    registers **/    
    quaternion quat;

/** Current contents of calibration status register **/
    char calib;
/** Contents of the 22 registers containing offset and radius values used as calibration by the sensor **/
    char calibration[22];
/** Structure containing sensor numbers, software version and chip UID **/
    chip ID;
/** Current temperature **/
    int temperature;
    
    private:
    
        I2C _i2c;
        char rx,tx[2],address;  //I2C variables
        char rawdata[22]; //Temporary array for input data values
        char op_mode;
        char pwr_mode;
        float accel_scale,rate_scale,angle_scale;
        int temp_scale;
        
void readchar(char location){
    tx[0] = location;
    _i2c.write(address,tx,1,true);
    _i2c.read(address,&rx,1,false);
}

void writechar(char location, char value){
    tx[0] = location;
    tx[1] = value;
    _i2c.write(address,tx,2);
}

void setpage(char value){
    writechar(BNO055_PAGE_ID_ADDR,value);
}
    };
#endif