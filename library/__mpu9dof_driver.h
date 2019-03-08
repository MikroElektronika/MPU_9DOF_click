/*
    __mpu9dof_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __mpu9dof_driver.h
@brief    MPU_9DOF Driver
@mainpage MPU_9DOF Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   MPU9DOF
@brief      MPU_9DOF Click Driver
@{

| Global Library Prefix | **MPU9DOF** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Aug 2018.**      |
| Developer             | **Nenad Filipovic**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _MPU9DOF_H_
#define _MPU9DOF_H_

/** 
 * @macro T_MPU9DOF_P
 * @brief Driver Abstract type 
 */
#define T_MPU9DOF_P    const uint8_t*

/** @defgroup MPU9DOF_COMPILE Compilation Config */              /** @{ */

//  #define   __MPU9DOF_DRV_SPI__                            /**<     @macro __MPU9DOF_DRV_SPI__  @brief SPI driver selector */
   #define   __MPU9DOF_DRV_I2C__                            /**<     @macro __MPU9DOF_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __MPU9DOF_DRV_UART__                           /**<     @macro __MPU9DOF_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup MPU9DOF_VAR Variables */                           /** @{ */

// MPU9150A I2C address
extern const uint8_t    _MPU9DOF_XLG_I2C_ADDR_1;
extern const uint8_t    _MPU9DOF_XLG_I2C_ADDR_0;
extern const uint8_t    _MPU9DOF_M_I2C_ADDR_0;
extern const uint8_t    _MPU9DOF_M_I2C_ADDR_1;

//Magnetometer Registers
extern const uint8_t _MPU9DOF_WHO_AM_I_MAG;
extern const uint8_t _MPU9DOF_INFO;
extern const uint8_t _MPU9DOF_MAG_ST1;
extern const uint8_t _MPU9DOF_MAG_ADDRESS;
extern const uint8_t _MPU9DOF_MAG_XOUT_L;
extern const uint8_t _MPU9DOF_MAG_XOUT_H;
extern const uint8_t _MPU9DOF_MAG_YOUT_L;
extern const uint8_t _MPU9DOF_MAG_YOUT_H;
extern const uint8_t _MPU9DOF_MAG_ZOUT_L;
extern const uint8_t _MPU9DOF_MAG_ZOUT_H;
extern const uint8_t _MPU9DOF_MAG_ST2;
extern const uint8_t _MPU9DOF_MAG_CNTL;
extern const uint8_t _MPU9DOF_MAG_ASTC;
extern const uint8_t _MPU9DOF_MAG_ASAX;
extern const uint8_t _MPU9DOF_MAG_ASAY;
extern const uint8_t _MPU9DOF_MAG_ASAZ;

extern const uint8_t _MPU9DOF_XGOFFS_TC;
extern const uint8_t _MPU9DOF_YGOFFS_TC;
extern const uint8_t _MPU9DOF_ZGOFFS_TC;
extern const uint8_t _MPU9DOF_X_FINE_GAIN;
extern const uint8_t _MPU9DOF_Y_FINE_GAIN;
extern const uint8_t _MPU9DOF_Z_FINE_GAIN;
extern const uint8_t _MPU9DOF_XA_OFFSET_H;
extern const uint8_t _MPU9DOF_XA_OFFSET_L_TC;
extern const uint8_t _MPU9DOF_YA_OFFSET_H;
extern const uint8_t _MPU9DOF_YA_OFFSET_L_TC;
extern const uint8_t _MPU9DOF_ZA_OFFSET_H;
extern const uint8_t _MPU9DOF_ZA_OFFSET_L_TC;
extern const uint8_t _MPU9DOF_SELF_TEST_X;
extern const uint8_t _MPU9DOF_SELF_TEST_Y;
extern const uint8_t _MPU9DOF_SELF_TEST_Z;
extern const uint8_t _MPU9DOF_SELF_TEST_A;
extern const uint8_t _MPU9DOF_XG_OFFS_USRH;
extern const uint8_t _MPU9DOF_XG_OFFS_USRL;
extern const uint8_t _MPU9DOF_YG_OFFS_USRH;
extern const uint8_t _MPU9DOF_YG_OFFS_USRL;
extern const uint8_t _MPU9DOF_ZG_OFFS_USRH;
extern const uint8_t _MPU9DOF_ZG_OFFS_USRL;
extern const uint8_t _MPU9DOF_SMPLRT_DIV;
extern const uint8_t _MPU9DOF_CONFIG;
extern const uint8_t _MPU9DOF_GYRO_CONFIG;
extern const uint8_t _MPU9DOF_ACCEL_CONFIG;
extern const uint8_t _MPU9DOF_FF_THR;
extern const uint8_t _MPU9DOF_FF_DUR;
extern const uint8_t _MPU9DOF_MOT_THR;
extern const uint8_t _MPU9DOF_MOT_DUR;
extern const uint8_t _MPU9DOF_ZMOT_THR;
extern const uint8_t _MPU9DOF_ZRMOT_DUR;
extern const uint8_t _MPU9DOF_FIFO_EN;
extern const uint8_t _MPU9DOF_I2C_MST_CTRL;
extern const uint8_t _MPU9DOF_I2C_SLV0_ADDR;
extern const uint8_t _MPU9DOF_I2C_SLV0_REG;
extern const uint8_t _MPU9DOF_I2C_SLV0_CTRL;
extern const uint8_t _MPU9DOF_I2C_SLV1_ADDR;
extern const uint8_t _MPU9DOF_I2C_SLV1_REG;
extern const uint8_t _MPU9DOF_I2C_SLV1_CTRL;
extern const uint8_t _MPU9DOF_I2C_SLV2_ADDR;
extern const uint8_t _MPU9DOF_I2C_SLV2_REG;
extern const uint8_t _MPU9DOF_I2C_SLV2_CTRL;
extern const uint8_t _MPU9DOF_I2C_SLV3_ADDR;
extern const uint8_t _MPU9DOF_I2C_SLV3_REG;
extern const uint8_t _MPU9DOF_I2C_SLV3_CTRL;
extern const uint8_t _MPU9DOF_I2C_SLV4_ADDR;
extern const uint8_t _MPU9DOF_I2C_SLV4_REG;
extern const uint8_t _MPU9DOF_I2C_SLV4_DO;
extern const uint8_t _MPU9DOF_I2C_SLV4_CTRL;
extern const uint8_t _MPU9DOF_I2C_SLV4_DI;
extern const uint8_t _MPU9DOF_I2C_MST_STATUS;
extern const uint8_t _MPU9DOF_INT_PIN_CFG;
extern const uint8_t _MPU9DOF_INT_ENABLE;
extern const uint8_t _MPU9DOF_DMP_INT_STATUS;
extern const uint8_t _MPU9DOF_INT_STATUS;
extern const uint8_t _MPU9DOF_ACCEL_XOUT_H;
extern const uint8_t _MPU9DOF_ACCEL_XOUT_L;
extern const uint8_t _MPU9DOF_ACCEL_YOUT_H;
extern const uint8_t _MPU9DOF_ACCEL_YOUT_L;
extern const uint8_t _MPU9DOF_ACCEL_ZOUT_H;
extern const uint8_t _MPU9DOF_ACCEL_ZOUT_L;
extern const uint8_t _MPU9DOF_TEMP_OUT_H;
extern const uint8_t _MPU9DOF_TEMP_OUT_L;
extern const uint8_t _MPU9DOF_GYRO_XOUT_H;
extern const uint8_t _MPU9DOF_GYRO_XOUT_L;
extern const uint8_t _MPU9DOF_GYRO_YOUT_H;
extern const uint8_t _MPU9DOF_GYRO_YOUT_L;
extern const uint8_t _MPU9DOF_GYRO_ZOUT_H;
extern const uint8_t _MPU9DOF_GYRO_ZOUT_L;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_00;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_01;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_02;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_03;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_04;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_05;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_06;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_07;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_08;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_09;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_10;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_11;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_12;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_13;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_14;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_15;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_16;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_17;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_18;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_19;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_20;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_21;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_22;
extern const uint8_t _MPU9DOF_EXT_SENS_DATA_23;
extern const uint8_t _MPU9DOF_MOT_DETECT_STATUS;
extern const uint8_t _MPU9DOF_I2C_SLV0_DO;
extern const uint8_t _MPU9DOF_I2C_SLV1_DO;
extern const uint8_t _MPU9DOF_I2C_SLV2_DO;
extern const uint8_t _MPU9DOF_I2C_SLV3_DO;
extern const uint8_t _MPU9DOF_I2C_MST_DELAY_CTRL;
extern const uint8_t _MPU9DOF_SIGNAL_PATH_RESET;
extern const uint8_t _MPU9DOF_MOT_DETECT_CTRL;
extern const uint8_t _MPU9DOF_USER_CTRL;
extern const uint8_t _MPU9DOF_PWR_MGMT_1;
extern const uint8_t _MPU9DOF_PWR_MGMT_2;
extern const uint8_t _MPU9DOF_DMP_BANK;
extern const uint8_t _MPU9DOF_DMP_RW_PNT;
extern const uint8_t _MPU9DOF_DMP_REG;
extern const uint8_t _MPU9DOF_DMP_REG_1;
extern const uint8_t _MPU9DOF_DMP_REG_2;
extern const uint8_t _MPU9DOF_FIFO_COUNTH;
extern const uint8_t _MPU9DOF_FIFO_COUNTL;
extern const uint8_t _MPU9DOF_FIFO_R_W;
extern const uint8_t _MPU9DOF_WHO_AM_I_XLG;

// Configuration bits
extern const uint8_t _MPU9DOF_BIT_SLEEP;
extern const uint8_t _MPU9DOF_BIT_H_RESET;
extern const uint8_t _MPU9DOF_BITS_CLKSEL;
extern const uint8_t _MPU9DOF_MPU_CLK_SEL_PLLGYROX;
extern const uint8_t _MPU9DOF_MPU_CLK_SEL_PLLGYROZ;
extern const uint8_t _MPU9DOF_MPU_EXT_SYNC_GYROX;
extern const uint8_t _MPU9DOF_BITS_AFSL_SEL_2G;
extern const uint8_t _MPU9DOF_BITS_AFSL_SEL_4G;
extern const uint8_t _MPU9DOF_BITS_AFSL_SEL_8G;
extern const uint8_t _MPU9DOF_BITS_AFSL_SEL_16G;
extern const uint8_t _MPU9DOF_BITS_FS_250DPS;
extern const uint8_t _MPU9DOF_BITS_FS_500DPS;
extern const uint8_t _MPU9DOF_BITS_FS_1000DPS;
extern const uint8_t _MPU9DOF_BITS_FS_2000DPS;
extern const uint8_t _MPU9DOF_BITS_FS_MASK;
extern const uint8_t _MPU9DOF_BITS_DLPF_CFG_256HZ_NOLPF2;
extern const uint8_t _MPU9DOF_BITS_DLPF_CFG_188HZ;
extern const uint8_t _MPU9DOF_BITS_DLPF_CFG_98HZ;
extern const uint8_t _MPU9DOF_BITS_DLPF_CFG_42HZ;
extern const uint8_t _MPU9DOF_BITS_DLPF_CFG_20HZ;
extern const uint8_t _MPU9DOF_BITS_DLPF_CFG_10HZ;
extern const uint8_t _MPU9DOF_BITS_DLPF_CFG_5HZ;
extern const uint8_t _MPU9DOF_BITS_DLPF_CFG_2100HZ_NOLPF;
extern const uint8_t _MPU9DOF_BITS_DLPF_CFG_MASK;
extern const uint8_t _MPU9DOF_BIT_INT_ANYRD_2CLEAR;
extern const uint8_t _MPU9DOF_BIT_RAW_RDY_EN;
extern const uint8_t _MPU9DOF_BIT_I2C_IF_DIS;
extern const uint8_t _MPU9DOF_BIT_INT_PIN_CFG;
extern const uint8_t _MPU9DOF_BIT_FIFO_EN;
extern const uint8_t _MPU9DOF_BIT_FIFO_DIS;
extern const uint8_t _MPU9DOF_DEFAULT;



                                                                       /** @} */
/** @defgroup MPU9DOF_TYPES Types */                             /** @{ */



                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup MPU9DOF_INIT Driver Initialization */              /** @{ */

#ifdef   __MPU9DOF_DRV_SPI__
void mpu9dof_spiDriverInit(T_MPU9DOF_P gpioObj, T_MPU9DOF_P spiObj);
#endif
#ifdef   __MPU9DOF_DRV_I2C__
void mpu9dof_i2cDriverInit(T_MPU9DOF_P gpioObj, T_MPU9DOF_P i2cObj, uint8_t slave);
#endif
#ifdef   __MPU9DOF_DRV_UART__
void mpu9dof_uartDriverInit(T_MPU9DOF_P gpioObj, T_MPU9DOF_P uartObj);
#endif


/** @defgroup MPU9DOF_FUNC Driver Functions */                   /** @{ */

/**
 * @brief Generic write data function
 *
 * @param[in] address         Register address
 *
 * @param[in] writeCommand    Command to write
 *
 * Function write byte of data to MPU-9150 XL G
 */
void mpu9dof_writeData( uint8_t address, uint8_t writeCommand );

/**
 * @brief Generic write data function
 *
 * @param[in] address         Register address
 *
 * @param[in] writeCommand    Command to write
 *
 * Function write byte of data to MPU-9150 MAG
 */
void mpu9dof_writeDataMag( uint8_t address, uint8_t writeCommand );

/**
 * @brief Generic read data function
 *
 * @param[in] address         Register address
 *
 * @return    Data from addressed register in MPU-9150 XL G
 *
 * Function read byte of data from register address of MPU-9150 XL G
 */
uint8_t mpu9dof_readData( uint8_t address );

/**
 * @brief Generic read data function
 *
 * @param[in] address         Register address
 *
 * @return    Data from addressed register in MPU-9150 MAG
 *
 * Function read byte of data from register address of MPU-9150 MAG
 */
uint8_t mpu9dof_readDataMag( uint8_t address );

/**
 * @brief Function get low and high register data
 *
 * @param[in] adrRegLow         low data register address
 *
 * @param[in] adrRegHigh         high data register address
 *
 * @return         16-bit value ( low and high data )
 *
 * Function get data from two MPU-9150 XL G register
 */
int16_t mpu9dof_getAxis( uint8_t adrRegHigh );

/**
 * @brief Function get low and high register data
 *
 * @param[in] adrRegLow         low data register address
 *
 * @param[in] adrRegHigh         high data register address
 *
 * @return         16-bit value ( low and high data )
 *
 * Function get data from two MPU-9150 MAG register
 */
int16_t mpu9dof_getAxisMag( uint8_t adrRegLow );

/**
 * @brief Function read axis
 *
 * @param[out] gyroX             pointer to read Gyro X-axis data
 * @param[out] gyroY             pointer to read Gyro Y-axis data
 * @param[out] gyroZ             pointer to read Gyro Z-axis data
 *
 * Function read Gyro X-axis, Y-axis and Z-axis axis.
 *
 */
void mpu9dof_readGyro( int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ );

/**
 * @brief Function read axis
 *
 * @param[out] accelX             pointer to read Accel X-axis data
 * @param[out] accelY             pointer to read Accel Y-axis data
 * @param[out] accelZ             pointer to read Accel Z-axis data
 *
 * Function read Accel X-axis, Y-axis and Z-axis axis.
 *
 */
void mpu9dof_readAccel( int16_t *accelX, int16_t *accelY, int16_t *accelZ );

/**
 * @brief Function read axis
 *
 * @param[out] magX             pointer to read Accel X-axis data
 * @param[out] magY             pointer to read Accel Y-axis data
 * @param[out] magZ             pointer to read Accel Z-axis data
 *
 * Function read Magnetometar X-axis, Y-axis and Z-axis axis.
 *
 */
void mpu9dof_readMag( int16_t *magX, int16_t *magY, int16_t *magZ );

/**
 * @brief Function read temperature data in degrees [ °C ]
 *
 * @return         temperature in degrees [ °C ]
 *
 * Function read temperature data
 */
float mpu9dof_readTemperature();





                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_MPU_9DOF_STM.c
    @example Click_MPU_9DOF_TIVA.c
    @example Click_MPU_9DOF_CEC.c
    @example Click_MPU_9DOF_KINETIS.c
    @example Click_MPU_9DOF_MSP.c
    @example Click_MPU_9DOF_PIC.c
    @example Click_MPU_9DOF_PIC32.c
    @example Click_MPU_9DOF_DSPIC.c
    @example Click_MPU_9DOF_AVR.c
    @example Click_MPU_9DOF_FT90x.c
    @example Click_MPU_9DOF_STM.mbas
    @example Click_MPU_9DOF_TIVA.mbas
    @example Click_MPU_9DOF_CEC.mbas
    @example Click_MPU_9DOF_KINETIS.mbas
    @example Click_MPU_9DOF_MSP.mbas
    @example Click_MPU_9DOF_PIC.mbas
    @example Click_MPU_9DOF_PIC32.mbas
    @example Click_MPU_9DOF_DSPIC.mbas
    @example Click_MPU_9DOF_AVR.mbas
    @example Click_MPU_9DOF_FT90x.mbas
    @example Click_MPU_9DOF_STM.mpas
    @example Click_MPU_9DOF_TIVA.mpas
    @example Click_MPU_9DOF_CEC.mpas
    @example Click_MPU_9DOF_KINETIS.mpas
    @example Click_MPU_9DOF_MSP.mpas
    @example Click_MPU_9DOF_PIC.mpas
    @example Click_MPU_9DOF_PIC32.mpas
    @example Click_MPU_9DOF_DSPIC.mpas
    @example Click_MPU_9DOF_AVR.mpas
    @example Click_MPU_9DOF_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __mpu9dof_driver.h

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