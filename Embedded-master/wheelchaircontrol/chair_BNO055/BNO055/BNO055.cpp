#include "BNO055.h"
#include "mbed.h"

BNO055::BNO055(PinName SDA, PinName SCL) : _i2c(SDA,SCL){
    //Set I2C fast and bring reset line high
    _i2c.frequency(400000);
    address = BNOAddress;
    accel_scale = 0.001f;
    rate_scale = 1.0f/16.0f;
    angle_scale = 1.0f/16.0f;
    temp_scale = 1;
    }
    
void BNO055::reset(){
//Perform a power-on-reset
    readchar(BNO055_SYS_TRIGGER_ADDR);
    rx = rx | 0x20;
    writechar(BNO055_SYS_TRIGGER_ADDR,rx);
//Wait for the system to come back up again (datasheet says 650ms)
    wait_ms(675);
}
    
bool BNO055::check(){
//Check we have communication link with the chip
    readchar(BNO055_CHIP_ID_ADDR);
    if (rx != 0xA0) return false;
//Grab the chip ID and software versions
    tx[0] = BNO055_CHIP_ID_ADDR;
    _i2c.write(address,tx,1,true);  
    _i2c.read(address+1,rawdata,7,false); 
    ID.id = rawdata[0];
    ID.accel = rawdata[1];
    ID.mag = rawdata[2];
    ID.gyro = rawdata[3];
    ID.sw[0] = rawdata[4];
    ID.sw[1] = rawdata[5];
    ID.bootload = rawdata[6];
    setpage(1);
    tx[0] = BNO055_UNIQUE_ID_ADDR;
    _i2c.write(address,tx,1,true);  
    _i2c.read(address+1,ID.serial,16,false); 
    setpage(0);
    return true;
    }
    
void BNO055::SetExternalCrystal(bool yn){
// Read the current status from the device
    readchar(BNO055_SYS_TRIGGER_ADDR); 
    if (yn) rx = rx | 0x80;
    else rx = rx & 0x7F;
    writechar(BNO055_SYS_TRIGGER_ADDR,rx); 
}

void BNO055::set_accel_units(char units){
    readchar(BNO055_UNIT_SEL_ADDR);
    if(units == MPERSPERS){
        rx = rx & 0xFE;
        accel_scale = 0.01f;
        }
    else {
        rx = rx | units;
        accel_scale = 0.001f;
        }
    writechar(BNO055_UNIT_SEL_ADDR,rx);
}

void BNO055::set_anglerate_units(char units){
    readchar(BNO055_UNIT_SEL_ADDR);
    if (units == DEG_PER_SEC){
        rx = rx & 0xFD;
        rate_scale = 1.0f/16.0f;
        }
    else {
        rx = rx | units;
        rate_scale = 1.0f/900.0f;
        }
    writechar(BNO055_UNIT_SEL_ADDR,rx);
}    

void BNO055::set_angle_units(char units){
    readchar(BNO055_UNIT_SEL_ADDR);
    if (units == DEGREES){
        rx = rx & 0xFB;
        angle_scale = 1.0f/16.0f;
        }
    else {
        rx = rx | units;
        rate_scale = 1.0f/900.0f;
        }
    writechar(BNO055_UNIT_SEL_ADDR,rx);
}    

void BNO055::set_temp_units(char units){
    readchar(BNO055_UNIT_SEL_ADDR);
    if (units == CENTIGRADE){
        rx = rx & 0xEF;
        temp_scale = 1;
        }
    else {
        rx = rx | units;
        temp_scale = 2;
        }
    writechar(BNO055_UNIT_SEL_ADDR,rx);
}    

void BNO055::set_orientation(char units){
    readchar(BNO055_UNIT_SEL_ADDR);
    if (units == WINDOWS) rx = rx &0x7F;
    else rx = rx | units;
    writechar(BNO055_UNIT_SEL_ADDR,rx);
}        

void BNO055::setmode(char omode){
    writechar(BNO055_OPR_MODE_ADDR,omode);
    op_mode = omode;
}

void BNO055::setpowermode(char pmode){
    writechar(BNO055_PWR_MODE_ADDR,pmode);
    pwr_mode = pmode;
}

void BNO055::get_accel(void){
    tx[0] = BNO055_ACCEL_DATA_X_LSB_ADDR;
    _i2c.write(address,tx,1,true);  
    _i2c.read(address+1,rawdata,6,0); 
    accel.rawx = (rawdata[1] << 8 | rawdata[0]);
    accel.rawy = (rawdata[3] << 8 | rawdata[2]);
    accel.rawz = (rawdata[5] << 8 | rawdata[4]);
    accel.x = float(accel.rawx)*accel_scale;
    accel.y = float(accel.rawy)*accel_scale;
    accel.z = float(accel.rawz)*accel_scale;
}
    
void BNO055::get_gyro(void){
    tx[0] = BNO055_GYRO_DATA_X_LSB_ADDR;
    _i2c.write(address,tx,1,true);  
    _i2c.read(address+1,rawdata,6,0); 
    gyro.rawx = (rawdata[1] << 8 | rawdata[0]);
    gyro.rawy = (rawdata[3] << 8 | rawdata[2]);
    gyro.rawz = (rawdata[5] << 8 | rawdata[4]);
    gyro.x = float(gyro.rawx)*rate_scale;
    gyro.y = float(gyro.rawy)*rate_scale;
    gyro.z = float(gyro.rawz)*rate_scale;
}

void BNO055::get_mag(void){
    tx[0] = BNO055_MAG_DATA_X_LSB_ADDR;
    _i2c.write(address,tx,1,true);  
    _i2c.read(address+1,rawdata,6,0); 
    mag.rawx = (rawdata[1] << 8 | rawdata[0]);
    mag.rawy = (rawdata[3] << 8 | rawdata[2]);
    mag.rawz = (rawdata[5] << 8 | rawdata[4]);
    mag.x = float(mag.rawx);
    mag.y = float(mag.rawy);
    mag.z = float(mag.rawz);
}

void BNO055::get_lia(void){
    tx[0] = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR;
    _i2c.write(address,tx,1,true);  
    _i2c.read(address+1,rawdata,6,0); 
    lia.rawx = (rawdata[1] << 8 | rawdata[0]);
    lia.rawy = (rawdata[3] << 8 | rawdata[2]);
    lia.rawz = (rawdata[5] << 8 | rawdata[4]);
    lia.x = float(lia.rawx)*accel_scale;
    lia.y = float(lia.rawy)*accel_scale;
    lia.z = float(lia.rawz)*accel_scale;
}

void BNO055::get_grv(void){
    tx[0] = BNO055_GRAVITY_DATA_X_LSB_ADDR;
    _i2c.write(address,tx,1,true);  
    _i2c.read(address+1,rawdata,6,0); 
    gravity.rawx = (rawdata[1] << 8 | rawdata[0]);
    gravity.rawy = (rawdata[3] << 8 | rawdata[2]);
    gravity.rawz = (rawdata[5] << 8 | rawdata[4]);
    gravity.x = float(gravity.rawx)*accel_scale;
    gravity.y = float(gravity.rawy)*accel_scale;
    gravity.z = float(gravity.rawz)*accel_scale;
}

void BNO055::get_quat(void){
    tx[0] = BNO055_QUATERNION_DATA_W_LSB_ADDR;
    _i2c.write(address,tx,1,true);  
    _i2c.read(address+1,rawdata,8,0); 
    quat.raww = (rawdata[1] << 8 | rawdata[0]);
    quat.rawx = (rawdata[3] << 8 | rawdata[2]);
    quat.rawy = (rawdata[5] << 8 | rawdata[4]);
    quat.rawz = (rawdata[7] << 8 | rawdata[6]);
    quat.w = float(quat.raww)/16384.0f;
    quat.x = float(quat.rawx)/16384.0f;
    quat.y = float(quat.rawy)/16384.0f;
    quat.z = float(quat.rawz)/16384.0f;
}

void BNO055::get_angles(void){
    tx[0] = BNO055_EULER_H_LSB_ADDR;
    _i2c.write(address,tx,1,true);  
    _i2c.read(address+1,rawdata,6,0); 
    euler.rawyaw = (rawdata[1] << 8 | rawdata[0]);
    euler.rawroll = (rawdata[3] << 8 | rawdata[2]);
    euler.rawpitch = (rawdata[5] << 8 | rawdata[4]);
    euler.yaw = float(euler.rawyaw)*angle_scale;
    euler.roll = float(euler.rawroll)*angle_scale;
    euler.pitch = float(euler.rawpitch)*angle_scale;
}


void BNO055::get_temp(void){
    readchar(BNO055_TEMP_ADDR);
    temperature = rx / temp_scale;
}

void BNO055::get_calib(void){
     readchar(BNO055_CALIB_STAT_ADDR);
     calib = rx;
}

void BNO055::read_calibration_data(void){
    char tempmode = op_mode;
    setmode(OPERATION_MODE_CONFIG);
    wait_ms(20);
    tx[0] = ACCEL_OFFSET_X_LSB_ADDR;
    _i2c.write(address,tx,1,true);  
    _i2c.read(address,calibration,22,false); 
    setmode(tempmode);
    wait_ms(10);
}

void BNO055::write_calibration_data(void){
    char tempmode = op_mode;
    setmode(OPERATION_MODE_CONFIG);
    wait_ms(20);
    tx[0] = ACCEL_OFFSET_X_LSB_ADDR;
    _i2c.write(address,tx,1,true);  
    _i2c.write(address,calibration,22,false); 
    setmode(tempmode);
    wait_ms(10);
}

void BNO055::set_mapping(char orient){
    switch (orient){
        case 0: 
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x21);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x04);
            break;
        case 1:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x00);
            break;
        case 2:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x00);
            break;
        case 3:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x21);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x02);
            break;
        case 4:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x03);
            break;
        case 5:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x21);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x01);
            break;
        case 6:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x21);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x07);
            break;
        case 7:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x05);
            break;
        default:
            writechar(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
            writechar(BNO055_AXIS_MAP_SIGN_ADDR,0x00);
        }
}