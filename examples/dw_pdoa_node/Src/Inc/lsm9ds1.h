/*! ----------------------------------------------------------------------------
 * @file    lsm9ds1.h
 * @brief   header for lsm9ds1.c driver
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#ifndef __LSM9DS1_H__
#define __LSM9DS1_H__

#include "port_platform.h"
#include "lsm9ds1_spi.h"

/* LSM9DS1 Accel/Gyro (XL/G) Registers */
#define ACT_THS             0x04
#define ACT_DUR             0x05
#define INT_GEN_CFG_XL      0x06
#define INT_GEN_THS_X_XL    0x07
#define INT_GEN_THS_Y_XL    0x08
#define INT_GEN_THS_Z_XL    0x09
#define INT_GEN_DUR_XL      0x0A
#define REFERENCE_G         0x0B
#define INT1_CTRL           0x0C
#define INT2_CTRL           0x0D
#define WHO_AM_I            0x0F
#define CTRL_REG1_G         0x10
#define CTRL_REG2_G         0x11
#define CTRL_REG3_G         0x12
#define ORIENT_CFG_G        0x13
#define INT_GEN_SRC_G       0x14
#define OUT_TEMP_L          0x15
#define OUT_TEMP_H          0x16
#define STATUS_REG_0        0x17
#define OUT_X_L_G           0x18
#define OUT_X_H_G           0x19
#define OUT_Y_L_G           0x1A
#define OUT_Y_H_G           0x1B
#define OUT_Z_L_G           0x1C
#define OUT_Z_H_G           0x1D
#define CTRL_REG4           0x1E
#define CTRL_REG5_XL        0x1F
#define CTRL_REG6_XL        0x20
#define CTRL_REG7_XL        0x21
#define CTRL_REG8           0x22
#define CTRL_REG9           0x23
#define CTRL_REG10          0x24
#define INT_GEN_SRC_XL      0x26
#define STATUS_REG_1        0x27
#define OUT_X_L_XL          0x28
#define OUT_X_H_XL          0x29
#define OUT_Y_L_XL          0x2A
#define OUT_Y_H_XL          0x2B
#define OUT_Z_L_XL          0x2C
#define OUT_Z_H_XL          0x2D
#define FIFO_CTRL           0x2E
#define FIFO_SRC            0x2F
#define INT_GEN_CFG_G       0x30
#define INT_GEN_THS_XH_G    0x31
#define INT_GEN_THS_XL_G    0x32
#define INT_GEN_THS_YH_G    0x33
#define INT_GEN_THS_YL_G    0x34
#define INT_GEN_THS_ZH_G    0x35
#define INT_GEN_THS_ZL_G    0x36
#define INT_GEN_DUR_G       0x37

/* LSM9DS1 Magneto Registers */
#define OFFSET_X_REG_L_M    0x05
#define OFFSET_X_REG_H_M    0x06
#define OFFSET_Y_REG_L_M    0x07
#define OFFSET_Y_REG_H_M    0x08
#define OFFSET_Z_REG_L_M    0x09
#define OFFSET_Z_REG_H_M    0x0A
#define WHO_AM_I            0x0F
#define CTRL_REG1_M         0x20
#define CTRL_REG2_M         0x21
#define CTRL_REG3_M         0x22
#define CTRL_REG4_M         0x23
#define CTRL_REG5_M         0x24
#define STATUS_REG_M        0x27
#define OUT_X_L_M           0x28
#define OUT_X_H_M           0x29
#define OUT_Y_L_M           0x2A
#define OUT_Y_H_M           0x2B
#define OUT_Z_L_M           0x2C
#define OUT_Z_H_M           0x2D
#define INT_CFG_M           0x30
#define INT_SRC_M           0x30
#define INT_THS_L_M         0x32
#define INT_THS_H_M         0x33

/* LSM9DS1 WHO_AM_I Responses */
#define WHO_AM_I_XG         0x68
#define WHO_AM_I_M          0x3D

#define LSM_NO_DATA         0xDEAD


/// gyro_scale defines the possible full-scale ranges of the gyroscope:
typedef enum {
    G_SCALE_245DPS = 0x0,     // 00: +/- 245 degrees per second
    G_SCALE_500DPS = 0x1,     // 01: +/- 500 dps
    G_SCALE_2000DPS= 0x3,     // 11: +/- 2000 dps
    G_SCALE_MASK   = 0x3
} gyro_scale_e;

/// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
typedef enum {                  // ODR (Hz) --- Cutoff
    G_POWER_DOWN     = 0x00,    //  0           0
    G_ODR_15_BW_0    = 0x20,    //  14.9        0
    G_ODR_60_BW_16   = 0x40,    //  59.5        16
    G_ODR_119_BW_14  = 0x60,    //  119         14
    G_ODR_119_BW_31  = 0x61,    //  119         31
    G_ODR_238_BW_14  = 0x80,    //  238         14
    G_ODR_238_BW_29  = 0x81,    //  238         29
    G_ODR_238_BW_63  = 0x82,    //  238         63
    G_ODR_238_BW_78  = 0x83,    //  238         78
    G_ODR_476_BW_21  = 0xA0,    //  476         21
    G_ODR_476_BW_28  = 0xA1,    //  476         28
    G_ODR_476_BW_57  = 0xA2,    //  476         57
    G_ODR_476_BW_100 = 0xA3,    //  476         100
    G_ODR_952_BW_33  = 0xC0,    //  952         33
    G_ODR_952_BW_40  = 0xC1,    //  952         40
    G_ODR_952_BW_58  = 0xC2,    //  952         58
    G_ODR_952_BW_100 = 0xC3     //  952         100
} gyro_odr_e;

// accel_scale defines all possible FSR's of the accelerometer:
typedef enum {
    A_SCALE_2G  = 0x00, // 00: +/- 2g
    A_SCALE_16G = 0x01, // 01: +/- 16g
    A_SCALE_4G  = 0x02, // 10: +/- 4g
    A_SCALE_8G  = 0x03, // 11: +/- 8g
    A_SCALE_MASK= 0x03
} accel_scale_e;

/// accel_oder defines all possible output data rates of the accelerometer:
typedef enum {
    A_POWER_DOWN,   // Power-down mode (0x0)
    A_ODR_10,       // 10 Hz (0x1)
    A_ODR_50,       // 50 Hz (0x2)
    A_ODR_119,      // 119 Hz (0x3)
    A_ODR_238,      // 238 Hz (0x4)
    A_ODR_476,      // 476 Hz (0x5)
    A_ODR_952       // 952 Hz (0x6)
} accel_odr_e;

// accel_bw defines all possible bandwiths for low-pass filter of the accelerometer:
typedef enum {
    A_BW_AUTO_SCALE = 0x0,  // Automatic BW scaling (0x0)
    A_BW_408 = 0x4,         // 408 Hz (0x4)
    A_BW_211 = 0x5,         // 211 Hz (0x5)
    A_BW_105 = 0x6,         // 105 Hz (0x6)
    A_BW_50  = 0x7          // 50 Hz (0x7)
} accel_bw_e;

// mag_scale defines all possible FSR's of the magnetometer:
typedef enum {
    M_SCALE_4GS = 0x00,    // 00: +/- 4Gs
    M_SCALE_8GS = 0x01,    // 01: +/- 8Gs
    M_SCALE_12GS= 0x02,   // 10: +/- 12Gs
    M_SCALE_16GS= 0x03,   // 11: +/- 16Gs
    M_SCALE_MASK = 0x03
} mag_scale_e;

// mag_odr defines all possible output data rates of the magnetometer:
typedef enum {
    M_ODR_0625, // 0.625 Hz (0x00)
    M_ODR_125,  // 1.25 Hz  (0x01)
    M_ODR_25,   // 2.5 Hz   (0x02)
    M_ODR_5,    // 5 Hz     (0x03)
    M_ODR_10,   // 10       (0x04)
    M_ODR_20,   // 20 Hz    (0x05)
    M_ODR_40,   // 40 Hz    (0x06)
    M_ODR_80    // 80 Hz    (0x07)
} mag_odr_e;


// specifies the driver working mode
typedef enum {
    LSM_GYRO    = 0x01,
    LSM_ACCEL   = 0x02,
    LSM_MAG     = 0x04,
    LSM_TEMP    = 0x08
}lsm_func_e;


// driver structure, returned to the application
// on calling of lsm_drv_open(lsm_func_e);
// lsm_drv_open() is also calling an init "constructor", which read the status
//
typedef struct {

        const void *self;
        lsm_func_e  enable;
        uint16_t    devID;

        uint16_t (* init)(gyro_scale_e, accel_scale_e, mag_scale_e, gyro_odr_e, accel_odr_e, mag_odr_e);
        void     (* deinit)(void);

        void (* readGyro)(void);
        void (* readAccel)(void);
        void (* readMag)(void);
        void (* readTemp)(void);

        void (* setGyroScale)(gyro_scale_e);
        void (* setAccelScale)(accel_scale_e);
        void (* setMagScale)(mag_scale_e);

        void (* setGyroODR)(gyro_odr_e);
        void (* setAccelODR)(accel_odr_e);
        void (* setMagODR)(mag_odr_e);

        int (* accelReady)(void);

        struct {
            void (*write)(lsm9ds1_cs_e dest, uint8_t *buf, uint16_t len);
            void (*read) (lsm9ds1_cs_e dest, uint8_t *buf, uint16_t len);
        } lsmSpi;

        // We'll store the gyro, accel, and magnetometer readings in following
        // variables. Each sensor gets three variables -- one for each
        // axis.
        // Call readGyro(), readAccel(), and readMag() first, before using variables.
        //
        // These values are the RAW signed 16-bit readings from the sensors.
        int16_t gx_raw, gy_raw, gz_raw; // x, y, and z axis readings of the gyroscope
        int16_t ax_raw, ay_raw, az_raw; // x, y, and z axis readings of the accelerometer
        int16_t mx_raw, my_raw, mz_raw; // x, y, and z axis readings of the magnetometer
        int16_t temperature_raw;

        // floating-point values of scaled data in real-world units
        float gx, gy, gz; // in "
        float ax, ay, az; // in "g"
        float mx, my, mz; // in "G"
        float temperature_c; // temperature in Celcius

}lsm9ds_driver_t;

/* exported driver instance */
extern lsm9ds_driver_t *drv;


/* exported function prototypes */
lsm9ds_driver_t    *lsm9ds1_driver_open(int lsm_functions_enable);
void                lsm9ds1_driver_close(void);



/* function prototypes */

/* @fn lsm9ds1_init()
 * @brief
 * Initialize the gyro, accelerometer, and magnetometer.
 *  This will set up the scale and output rate of each sensor. It'll also
 *  "turn on" every sensor and every axis of every sensor, if its configured in the
 *  "enable" variable of the driver.
 *
 *  If sensor is not enabled, software will turn it off.
 *
 *  Input:
 *   - gScl = The scale of the gyroscope. This should be a gyro_scale value.
 *   - aScl = The scale of the accelerometer. Should be a accel_scale value.
 *   - mScl = The scale of the magnetometer. Should be a mag_scale value.
 *   - gODR = Output data rate of the gyroscope. gyro_odr value.
 *   - aODR = Output data rate of the accelerometer. accel_odr value.
 *   - mODR = Output data rate of the magnetometer. mag_odr value.
 *  Output: The function will return an unsigned 16-bit value. The most-sig
 *       bytes of the output are the WHO_AM_I reading of the accel/gyro. The
 *       least significant two bytes are the WHO_AM_I reading of the mag.
 *  All parameters have a defaulted value, so you can call just "begin()".
 *  Default values are FSR's of: +/- 245DPS, 4g, 2Gs; ODRs of 119 Hz for
 *  gyro, 119 Hz for accelerometer, 80 Hz for magnetometer.
 *  Use the return value of this function to verify communication.
 */
uint16_t lsm9ds1_init(gyro_scale_e, accel_scale_e, mag_scale_e, gyro_odr_e, accel_odr_e, mag_odr_e);

/* @fn lsm9ds1_deinit()
 * @brief
 *      instruct accel, gyro and mag to go to power-down mode
 */
void lsm9ds1_deinit();

/* @fn  readGyro()
 * @brief
 * Read the gyroscope output.
 * This function will read all six gyroscope output registers.
 * The readings are stored in the gx_raw, gy_raw, and gz_raw variables.
 */
void readGyro();

/* @fn readAccel()
 * @brief Read the accelerometer output registers.
 *
 * This function will read all six accelerometer output registers.
 * The readings are stored in the ax_raw, ay_raw, and az_raw variables.
 */
void readAccel();

/* @fn readMag()
 * @brief   Read the magnetometer output registers.
 *  This function will read all six magnetometer output registers.
 *  The readings are stored in the mx_raw, my_raw, and mz_raw variables.
 */
void readMag();

/* @fn readTemp()
 * @brief Read the temperature output register.
 *  This function will read two temperature output registers.
 *  The combined readings are stored in the temperature variables.
 */
void readTemp();

/* @fn setGyroScale()
 * @brief Sets the full-scale range of the gyroscope.
 *  This function can be called to set the scale of the gyroscope to
 *  245, 500, or 2000 degrees per second.
 * @param
 *   gScl = The desired gyroscope scale. Must be one of three possible
 *       values from the gyro_scale_e.
 */
void setGyroScale(gyro_scale_e);

/* @fn setAccelScale()
 * @brief Set the full-scale range of the accelerometer.
 *  This function can be called to set the scale of the accelerometer to
 *  2, 4, 8, or 16 g's.
 * @param
 *   aScl = The desired accelerometer scale. Must be one of five possible
 *       values from the accel_scale_e.
 */
void setAccelScale(accel_scale_e);

/* @fn setMagScale()
 * @brief   Set the full-scale range of the magnetometer.
 *  This function can be called to set the scale of the magnetometer to
 *  4, 8, 12, or 16 Gs.
 * @parm
 *   mScl = The desired magnetometer scale. Must be one of four possible
 *       values from the mag_scale_e.
 */
void setMagScale(mag_scale_e);

/* @fn setGyroODR()
 * @brief   Set the output data rate and bandwidth of the gyroscope
 * @param
 *   gRate = The desired output rate and cutoff frequency of the gyro.
 *       Must be a value from the gyro_odr_e.
 */
void setGyroODR(gyro_odr_e);

/* @fn setAccelODR()
 * @brief   Set the output data rate of the accelerometer
 * @param
 *   aRate = The desired output rate of the accel.
 *       Must be a value from the accel_odr_e.
 */
void setAccelODR(accel_odr_e);

/* @fn setMagODR()
 * @brief    Set the output data rate of the magnetometer
 * @param
 *   mRate = The desired output rate of the mag.
 *       Must be a value from the mag_odr_e.
 */
void setMagODR(mag_odr_e);

/* @fn accelReady()
 * @brief Accelerometer data ready indicator.
 * @output 0 when accelerometer data are not ready yet
 */
int accelReady();

/* @fn gyroReady()
 * @brief Gyro data ready indicator.
 * @output 0 when gyro data are not ready yet
 */
int gyroReady();

/* @fn magReady()
 * @brief Magnetometer data ready indicator.
 * @output 0 when magnetometer data are not ready yet
 */
int magReady();

#endif // __LSM9DS1_H__
