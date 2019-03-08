/*
    __mpu9dof_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__mpu9dof_driver.h"
#include "__mpu9dof_hal.c"

/* ------------------------------------------------------------------- MACROS */



/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __MPU9DOF_DRV_I2C__
static uint8_t _slaveAddress;
#endif

// MPU9150A I2C address
const uint8_t _MPU9DOF_XLG_I2C_ADDR_1                   = 0x69;  //Device address jumper pos 1 (0xD2 >> 1)
const uint8_t _MPU9DOF_XLG_I2C_ADDR_0                   = 0x68;  //Device address jumper pos 0
const uint8_t _MPU9DOF_M_I2C_ADDR_0                     = 0x0C;  // Address of the magnetometer in bypass mode
const uint8_t _MPU9DOF_M_I2C_ADDR_1                     = 0x0D;

//Magnetometer Registers
const uint8_t _MPU9DOF_WHO_AM_I_MAG                     = 0x00;  // should return = 0x48
const uint8_t _MPU9DOF_INFO                             = 0x01;
const uint8_t _MPU9DOF_MAG_ST1                          = 0x02;  // data ready status bit 0
const uint8_t _MPU9DOF_MAG_ADDRESS                      = 0x0C;
const uint8_t _MPU9DOF_MAG_XOUT_L                       = 0x03;  // data
const uint8_t _MPU9DOF_MAG_XOUT_H                       = 0x04;
const uint8_t _MPU9DOF_MAG_YOUT_L                       = 0x05;
const uint8_t _MPU9DOF_MAG_YOUT_H                       = 0x06;
const uint8_t _MPU9DOF_MAG_ZOUT_L                       = 0x07;
const uint8_t _MPU9DOF_MAG_ZOUT_H                       = 0x08;
const uint8_t _MPU9DOF_MAG_ST2                          = 0x09;  // Data overflow bit 3 and data read error status bit 2
const uint8_t _MPU9DOF_MAG_CNTL                         = 0x0A;  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
const uint8_t _MPU9DOF_MAG_ASTC                         = 0x0C;  // Self test control
const uint8_t _MPU9DOF_MAG_ASAX                         = 0x10;  // Fuse ROM x-axis sensitivity adjustment value
const uint8_t _MPU9DOF_MAG_ASAY                         = 0x11;  // Fuse ROM y-axis sensitivity adjustment value
const uint8_t _MPU9DOF_MAG_ASAZ                         = 0x12;  // Fuse ROM z-axis sensitivity adjustment value

const uint8_t _MPU9DOF_XGOFFS_TC                        = 0x00;  // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD
const uint8_t _MPU9DOF_YGOFFS_TC                        = 0x01;
const uint8_t _MPU9DOF_ZGOFFS_TC                        = 0x02;
const uint8_t _MPU9DOF_X_FINE_GAIN                      = 0x03;  // [7:0] fine gain
const uint8_t _MPU9DOF_Y_FINE_GAIN                      = 0x04;
const uint8_t _MPU9DOF_Z_FINE_GAIN                      = 0x05;
const uint8_t _MPU9DOF_XA_OFFSET_H                      = 0x06;  // User-defined trim values for accelerometer
const uint8_t _MPU9DOF_XA_OFFSET_L_TC                   = 0x07;
const uint8_t _MPU9DOF_YA_OFFSET_H                      = 0x08;
const uint8_t _MPU9DOF_YA_OFFSET_L_TC                   = 0x09;
const uint8_t _MPU9DOF_ZA_OFFSET_H                      = 0x0A;
const uint8_t _MPU9DOF_ZA_OFFSET_L_TC                   = 0x0B;
const uint8_t _MPU9DOF_SELF_TEST_X                      = 0x0D;
const uint8_t _MPU9DOF_SELF_TEST_Y                      = 0x0E;
const uint8_t _MPU9DOF_SELF_TEST_Z                      = 0x0F;
const uint8_t _MPU9DOF_SELF_TEST_A                      = 0x10;
const uint8_t _MPU9DOF_XG_OFFS_USRH                     = 0x13;  // User-defined trim values for gyroscope, populate with calibration routine
const uint8_t _MPU9DOF_XG_OFFS_USRL                     = 0x14;
const uint8_t _MPU9DOF_YG_OFFS_USRH                     = 0x15;
const uint8_t _MPU9DOF_YG_OFFS_USRL                     = 0x16;
const uint8_t _MPU9DOF_ZG_OFFS_USRH                     = 0x17;
const uint8_t _MPU9DOF_ZG_OFFS_USRL                     = 0x18;
const uint8_t _MPU9DOF_SMPLRT_DIV                       = 0x19;
const uint8_t _MPU9DOF_CONFIG                           = 0x1A;
const uint8_t _MPU9DOF_GYRO_CONFIG                      = 0x1B;
const uint8_t _MPU9DOF_ACCEL_CONFIG                     = 0x1C;
const uint8_t _MPU9DOF_FF_THR                           = 0x1D;  // Free-fall
const uint8_t _MPU9DOF_FF_DUR                           = 0x1E;  // Free-fall
const uint8_t _MPU9DOF_MOT_THR                          = 0x1F;  // Motion detection threshold bits [7:0]
const uint8_t _MPU9DOF_MOT_DUR                          = 0x20;  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
const uint8_t _MPU9DOF_ZMOT_THR                         = 0x21;  // Zero-motion detection threshold bits [7:0]
const uint8_t _MPU9DOF_ZRMOT_DUR                        = 0x22;  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
const uint8_t _MPU9DOF_FIFO_EN                          = 0x23;
const uint8_t _MPU9DOF_I2C_MST_CTRL                     = 0x24;
const uint8_t _MPU9DOF_I2C_SLV0_ADDR                    = 0x25;
const uint8_t _MPU9DOF_I2C_SLV0_REG                     = 0x26;
const uint8_t _MPU9DOF_I2C_SLV0_CTRL                    = 0x27;
const uint8_t _MPU9DOF_I2C_SLV1_ADDR                    = 0x28;
const uint8_t _MPU9DOF_I2C_SLV1_REG                     = 0x29;
const uint8_t _MPU9DOF_I2C_SLV1_CTRL                    = 0x2A;
const uint8_t _MPU9DOF_I2C_SLV2_ADDR                    = 0x2B;
const uint8_t _MPU9DOF_I2C_SLV2_REG                     = 0x2C;
const uint8_t _MPU9DOF_I2C_SLV2_CTRL                    = 0x2D;
const uint8_t _MPU9DOF_I2C_SLV3_ADDR                    = 0x2E;
const uint8_t _MPU9DOF_I2C_SLV3_REG                     = 0x2F;
const uint8_t _MPU9DOF_I2C_SLV3_CTRL                    = 0x30;
const uint8_t _MPU9DOF_I2C_SLV4_ADDR                    = 0x31;
const uint8_t _MPU9DOF_I2C_SLV4_REG                     = 0x32;
const uint8_t _MPU9DOF_I2C_SLV4_DO                      = 0x33;
const uint8_t _MPU9DOF_I2C_SLV4_CTRL                    = 0x34;
const uint8_t _MPU9DOF_I2C_SLV4_DI                      = 0x35;
const uint8_t _MPU9DOF_I2C_MST_STATUS                   = 0x36;
const uint8_t _MPU9DOF_INT_PIN_CFG                      = 0x37;
const uint8_t _MPU9DOF_INT_ENABLE                       = 0x38;
const uint8_t _MPU9DOF_DMP_INT_STATUS                   = 0x39;  // Check DMP interrupt
const uint8_t _MPU9DOF_INT_STATUS                       = 0x3A;
const uint8_t _MPU9DOF_ACCEL_XOUT_H                     = 0x3B;
const uint8_t _MPU9DOF_ACCEL_XOUT_L                     = 0x3C;
const uint8_t _MPU9DOF_ACCEL_YOUT_H                     = 0x3D;
const uint8_t _MPU9DOF_ACCEL_YOUT_L                     = 0x3E;
const uint8_t _MPU9DOF_ACCEL_ZOUT_H                     = 0x3F;
const uint8_t _MPU9DOF_ACCEL_ZOUT_L                     = 0x40;
const uint8_t _MPU9DOF_TEMP_OUT_H                       = 0x41;
const uint8_t _MPU9DOF_TEMP_OUT_L                       = 0x42;
const uint8_t _MPU9DOF_GYRO_XOUT_H                      = 0x43;
const uint8_t _MPU9DOF_GYRO_XOUT_L                      = 0x44;
const uint8_t _MPU9DOF_GYRO_YOUT_H                      = 0x45;
const uint8_t _MPU9DOF_GYRO_YOUT_L                      = 0x46;
const uint8_t _MPU9DOF_GYRO_ZOUT_H                      = 0x47;
const uint8_t _MPU9DOF_GYRO_ZOUT_L                      = 0x48;
const uint8_t _MPU9DOF_EXT_SENS_DATA_00                 = 0x49;
const uint8_t _MPU9DOF_EXT_SENS_DATA_01                 = 0x4A;
const uint8_t _MPU9DOF_EXT_SENS_DATA_02                 = 0x4B;
const uint8_t _MPU9DOF_EXT_SENS_DATA_03                 = 0x4C;
const uint8_t _MPU9DOF_EXT_SENS_DATA_04                 = 0x4D;
const uint8_t _MPU9DOF_EXT_SENS_DATA_05                 = 0x4E;
const uint8_t _MPU9DOF_EXT_SENS_DATA_06                 = 0x4F;
const uint8_t _MPU9DOF_EXT_SENS_DATA_07                 = 0x50;
const uint8_t _MPU9DOF_EXT_SENS_DATA_08                 = 0x51;
const uint8_t _MPU9DOF_EXT_SENS_DATA_09                 = 0x52;
const uint8_t _MPU9DOF_EXT_SENS_DATA_10                 = 0x53;
const uint8_t _MPU9DOF_EXT_SENS_DATA_11                 = 0x54;
const uint8_t _MPU9DOF_EXT_SENS_DATA_12                 = 0x55;
const uint8_t _MPU9DOF_EXT_SENS_DATA_13                 = 0x56;
const uint8_t _MPU9DOF_EXT_SENS_DATA_14                 = 0x57;
const uint8_t _MPU9DOF_EXT_SENS_DATA_15                 = 0x58;
const uint8_t _MPU9DOF_EXT_SENS_DATA_16                 = 0x59;
const uint8_t _MPU9DOF_EXT_SENS_DATA_17                 = 0x5A;
const uint8_t _MPU9DOF_EXT_SENS_DATA_18                 = 0x5B;
const uint8_t _MPU9DOF_EXT_SENS_DATA_19                 = 0x5C;
const uint8_t _MPU9DOF_EXT_SENS_DATA_20                 = 0x5D;
const uint8_t _MPU9DOF_EXT_SENS_DATA_21                 = 0x5E;
const uint8_t _MPU9DOF_EXT_SENS_DATA_22                 = 0x5F;
const uint8_t _MPU9DOF_EXT_SENS_DATA_23                 = 0x60;
const uint8_t _MPU9DOF_MOT_DETECT_STATUS                = 0x61;
const uint8_t _MPU9DOF_I2C_SLV0_DO                      = 0x63;
const uint8_t _MPU9DOF_I2C_SLV1_DO                      = 0x64;
const uint8_t _MPU9DOF_I2C_SLV2_DO                      = 0x65;
const uint8_t _MPU9DOF_I2C_SLV3_DO                      = 0x66;
const uint8_t _MPU9DOF_I2C_MST_DELAY_CTRL               = 0x67;
const uint8_t _MPU9DOF_SIGNAL_PATH_RESET                = 0x68;
const uint8_t _MPU9DOF_MOT_DETECT_CTRL                  = 0x69;
const uint8_t _MPU9DOF_USER_CTRL                        = 0x6A;  // Bit 7 enable DMP, bit 3 reset DMP
const uint8_t _MPU9DOF_PWR_MGMT_1                       = 0x6B;  // Device defaults to the SLEEP mode
const uint8_t _MPU9DOF_PWR_MGMT_2                       = 0x6C;
const uint8_t _MPU9DOF_DMP_BANK                         = 0x6D;  // Activates a specific bank in the DMP
const uint8_t _MPU9DOF_DMP_RW_PNT                       = 0x6E;  // Set read/write pointer to a specific start address in specified DMP bank
const uint8_t _MPU9DOF_DMP_REG                          = 0x6F;  // Register in DMP from which to read or to which to write
const uint8_t _MPU9DOF_DMP_REG_1                        = 0x70;
const uint8_t _MPU9DOF_DMP_REG_2                        = 0x71;
const uint8_t _MPU9DOF_FIFO_COUNTH                      = 0x72;
const uint8_t _MPU9DOF_FIFO_COUNTL                      = 0x73;
const uint8_t _MPU9DOF_FIFO_R_W                         = 0x74;
const uint8_t _MPU9DOF_WHO_AM_I_XLG                     = 0x75;  // Should return = 0x68

// Configuration bits
const uint8_t _MPU9DOF_BIT_SLEEP                        = 0x40;
const uint8_t _MPU9DOF_BIT_H_RESET                      = 0x80;
const uint8_t _MPU9DOF_BITS_CLKSEL                      = 0x07;
const uint8_t _MPU9DOF_MPU_CLK_SEL_PLLGYROX             = 0x01;
const uint8_t _MPU9DOF_MPU_CLK_SEL_PLLGYROZ             = 0x03;
const uint8_t _MPU9DOF_MPU_EXT_SYNC_GYROX               = 0x02;
const uint8_t _MPU9DOF_BITS_AFSL_SEL_2G                 = 0x00;
const uint8_t _MPU9DOF_BITS_AFSL_SEL_4G                 = 0x08;
const uint8_t _MPU9DOF_BITS_AFSL_SEL_8G                 = 0x10;
const uint8_t _MPU9DOF_BITS_AFSL_SEL_16G                = 0x18;
const uint8_t _MPU9DOF_BITS_FS_250DPS                   = 0x00;
const uint8_t _MPU9DOF_BITS_FS_500DPS                   = 0x08;
const uint8_t _MPU9DOF_BITS_FS_1000DPS                  = 0x10;
const uint8_t _MPU9DOF_BITS_FS_2000DPS                  = 0x18;
const uint8_t _MPU9DOF_BITS_FS_MASK                     = 0x18;
const uint8_t _MPU9DOF_BITS_DLPF_CFG_256HZ_NOLPF2       = 0x00;
const uint8_t _MPU9DOF_BITS_DLPF_CFG_188HZ              = 0x01;
const uint8_t _MPU9DOF_BITS_DLPF_CFG_98HZ               = 0x02;
const uint8_t _MPU9DOF_BITS_DLPF_CFG_42HZ               = 0x03;
const uint8_t _MPU9DOF_BITS_DLPF_CFG_20HZ               = 0x04;
const uint8_t _MPU9DOF_BITS_DLPF_CFG_10HZ               = 0x05;
const uint8_t _MPU9DOF_BITS_DLPF_CFG_5HZ                = 0x06;
const uint8_t _MPU9DOF_BITS_DLPF_CFG_2100HZ_NOLPF       = 0x07;
const uint8_t _MPU9DOF_BITS_DLPF_CFG_MASK               = 0x07;
const uint8_t _MPU9DOF_BIT_INT_ANYRD_2CLEAR             = 0x10;
const uint8_t _MPU9DOF_BIT_RAW_RDY_EN                   = 0x01;
const uint8_t _MPU9DOF_BIT_I2C_IF_DIS                   = 0x10;
const uint8_t _MPU9DOF_BIT_INT_PIN_CFG                  = 0x02;
const uint8_t _MPU9DOF_BIT_FIFO_EN                      = 0x78;
const uint8_t _MPU9DOF_BIT_FIFO_DIS                     = 0x00;
const uint8_t _MPU9DOF_DEFAULT                          = 0x00;


/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */



/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */



/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __MPU9DOF_DRV_SPI__

void mpu9dof_spiDriverInit(T_MPU9DOF_P gpioObj, T_MPU9DOF_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __MPU9DOF_DRV_I2C__

void mpu9dof_i2cDriverInit(T_MPU9DOF_P gpioObj, T_MPU9DOF_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __MPU9DOF_DRV_UART__

void mpu9dof_uartDriverInit(T_MPU9DOF_P gpioObj, T_MPU9DOF_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif


/* ----------------------------------------------------------- IMPLEMENTATION */


/* Generic write data function MPU-9150 XL G */
void mpu9dof_writeData( uint8_t address, uint8_t writeCommand )
{
    uint8_t buffer[2];
    buffer[0]= address;
    buffer[1]= writeCommand;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, buffer, 2, END_MODE_STOP );
}

/* Generic write data function MPU-9150 MAG */
void mpu9dof_writeDataMag( uint8_t address, uint8_t writeCommand )
{
    uint8_t buffer[2];
    buffer[0]= address;
    buffer[1]= writeCommand;

    hal_i2cStart();
    hal_i2cWrite( _MPU9DOF_M_I2C_ADDR_0, buffer, 2, END_MODE_STOP );
}

/* Generic read data function MPU-9150 XL G */
uint8_t mpu9dof_readData( uint8_t address )
{
    uint8_t writeReg[1];
    uint8_t readReg[1];

    writeReg[0] = address;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, writeReg, 1, END_MODE_RESTART );
    Delay_10ms();
    hal_i2cRead( _slaveAddress, readReg, 1, END_MODE_STOP );
    Delay_10ms();

    return readReg[0];
}

/* Generic read data function MPU-9150 MAG */
uint8_t mpu9dof_readDataMag( uint8_t address )
{
    uint8_t writeReg[1];
    uint8_t readReg[1];

    writeReg[0] = address;

    hal_i2cStart();
    hal_i2cWrite( _MPU9DOF_M_I2C_ADDR_0, writeReg, 1, END_MODE_RESTART );
    Delay_10ms();
    hal_i2cRead( _MPU9DOF_M_I2C_ADDR_0, readReg, 1, END_MODE_STOP );
    Delay_10ms();

    return readReg[0];
}

/* Function get data from MPU-9150 XL G register */
int16_t mpu9dof_getAxis( uint8_t adrRegHigh )
{
    uint16_t result;
    uint8_t buffer[2];

    buffer[0] = mpu9dof_readData( adrRegHigh );
    buffer[1] = mpu9dof_readData( adrRegHigh + 1 );

    result = buffer[0];
    result <<= 8;
    result |= buffer[1];

    return result;
}

/* Function get data from MPU-9150 MAG register */
int16_t mpu9dof_getAxisMag( uint8_t adrRegLow )
{
    uint16_t result;
    uint8_t buffer[2];
    
    mpu9dof_writeDataMag( _MPU9DOF_MAG_CNTL, 0x01);
    Delay_10ms();

    buffer[0] = mpu9dof_readDataMag( adrRegLow + 1 );
    buffer[1] = mpu9dof_readDataMag( adrRegLow );

    result = buffer[0];
    result <<= 8;
    result |= buffer[1];

    return result;
}

/* Function read Gyro X-axis, Y-axis and Z-axis axis */
void mpu9dof_readGyro( int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ )
{
    *gyroX = mpu9dof_getAxis( _MPU9DOF_GYRO_XOUT_H );
    *gyroY = mpu9dof_getAxis( _MPU9DOF_GYRO_YOUT_H );
    *gyroZ = mpu9dof_getAxis( _MPU9DOF_GYRO_ZOUT_H );
}

/* Function read Accel X-axis, Y-axis and Z-axis */
void mpu9dof_readAccel( int16_t *accelX, int16_t *accelY, int16_t *accelZ )
{
    *accelX = mpu9dof_getAxis( _MPU9DOF_ACCEL_XOUT_H );
    *accelY = mpu9dof_getAxis( _MPU9DOF_ACCEL_YOUT_H );
    *accelZ = mpu9dof_getAxis( _MPU9DOF_ACCEL_ZOUT_H );
}

/* Function read Magnetometar X-axis, Y-axis and Z-axis */
void mpu9dof_readMag( int16_t *magX, int16_t *magY, int16_t *magZ )
{
    *magX = mpu9dof_getAxisMag( _MPU9DOF_MAG_XOUT_L );
    *magY = mpu9dof_getAxisMag( _MPU9DOF_MAG_YOUT_L );
    *magZ = mpu9dof_getAxisMag( _MPU9DOF_MAG_ZOUT_L );
}

/* Function read Temperature data from MPU-9150 XL G register */
float mpu9dof_readTemperature()
{
    int16_t result;
    float temperature;
    temperature = 0.00;

    result = mpu9dof_getAxis( _MPU9DOF_TEMP_OUT_H );
    Delay_10ms();

    temperature =  ( float ) result;
    temperature = ( temperature / 340.00 ) + 35.00;

    return temperature;
}




/* -------------------------------------------------------------------------- */
/*
  __mpu9dof_driver.c

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */