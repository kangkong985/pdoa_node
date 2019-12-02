/*! ----------------------------------------------------------------------------
 * @file    lsm9ds1.c
 * @brief   Accel, Gyro and Mag sensors driver for lsm9ds1
 *
 *          Currently only accelerometer is using, other sensors are in power-down mode.
 *          The accelerometer is initialized in a simple setup (no FIFO or interrupts) and using a fixed configuration (operation mode, update
 *          rate, etc.).
 *
 *  usage:
 *  1) drv = lsm9ds1_driver_open - this will return pointer to driver instance struct, if success. See lsm9ds_driver_t.
 *  2) drv->devID should be (WHO_AM_I_M_RSP<<8) | WHO_AM_I_AG_RSP)
 *  3) access to accelerometer driver functions:
 *    if(drv->accelReady()) {
 *     drv->readAccel();
 *    }
 *  4) results are in drv->gX,aX,mX variables, see lsm9ds_driver_t
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include "lsm9ds1.h"

lsm9ds_driver_t * drv = NULL;

//------------------------------------------------------------------------------
// Local variables

/* We have only one LSM9DS1 IMU sensor on the board,
 * will use static driver structure for it*/
static lsm9ds_driver_t lsm9ds_driver =
{
        .self           = &lsm9ds_driver,
        .enable         = (LSM_ACCEL),

        .init           = &lsm9ds1_init,
        .deinit         = &lsm9ds1_deinit,

        .readGyro       = &readGyro,
        .readAccel      = &readAccel,
        .readMag        = &readMag,
        .readTemp       = &readTemp,

        .setGyroScale   = &setGyroScale,
        .setAccelScale  = &setAccelScale,
        .setMagScale    = &setMagScale,

        .setGyroODR     = &setGyroODR,
        .setAccelODR    = &setAccelODR,
        .setMagODR      = &setMagODR,

        .accelReady     = &accelReady,

        .lsmSpi.read    = &lsm9ds1_read,
        .lsmSpi.write   = &lsm9ds1_write,

        .gx_raw = LSM_NO_DATA, .gy_raw = LSM_NO_DATA, .gz_raw = LSM_NO_DATA,
        .ax_raw = LSM_NO_DATA, .ay_raw = LSM_NO_DATA, .az_raw = LSM_NO_DATA,
        .mx_raw = LSM_NO_DATA, .my_raw = LSM_NO_DATA, .mz_raw = LSM_NO_DATA,

        .temperature_raw = LSM_NO_DATA,

        .gx = LSM_NO_DATA, .gy = LSM_NO_DATA, .gz = LSM_NO_DATA,
        .ax = LSM_NO_DATA, .ay = LSM_NO_DATA, .az = LSM_NO_DATA,
        .mx = LSM_NO_DATA, .my = LSM_NO_DATA, .mz = LSM_NO_DATA,

        .temperature_c = LSM_NO_DATA
};


/*  gRes, aRes, and mRes store the current resolution for each sensor.
 *  Units of these values would be DPS (or g's or Gs's) per ADC tick.
 *  This value is calculated as (scaleX) / (2^15).
 */
static float gRes, aRes, mRes;

//------------------------------------------------------------------------------
// Interface: "Constructor" and "destructor" for driver

/* @fn  lsm9ds1_driver_open()
 * @brief  Shall be called on beginning
 *         Opens driver and put all sensors to power down mode
 *  returns the pointer to driver instance lsm9ds_driver_t
 */
lsm9ds_driver_t *lsm9ds1_driver_open(int lsm_functions_enable)
{
    drv = &lsm9ds_driver;
    drv->enable = lsm_functions_enable;
    drv->devID = drv->init(    G_SCALE_245DPS,
                                A_SCALE_2G,
                                M_SCALE_4GS,
                                G_ODR_119_BW_14,
                                A_ODR_119,
                                M_ODR_80);

    if(drv->devID != ((WHO_AM_I_M<<8) | WHO_AM_I_XG))
    {
        //error_handler(0, _ERR_IMU_INIT);
        drv = NULL;
    }

    return drv;
};


/* @fn  lsm9ds1_driver_close()
 * @brief   close driver and put all sensors to power down mode
 */
void lsm9ds1_driver_close(void)
{
    if(drv)
    {
        drv->deinit();
    }
}

//------------------------------------------------------------------------------
// Internal driver functions

/* @fn  initGyro()
 * @brief   Sets up the gyroscope to begin reading.
 *  This function steps through all three gyroscope control registers.
 */
static void initGyro(gyro_scale_e tmp)
{
    uint8_t cmd[4] = {
        CTRL_REG1_G,
        (tmp<<3) | G_ODR_119_BW_14, // reg #10h
        0,          // reg #11h Default data out and int out
        0           // reg #12h Default power mode and high pass settings
    };

    // Write the data to the gyro control registers
    drv->lsmSpi.write(LSM9DS1_CS_XG, cmd, 4);
}


/* @fn  initAccel()
 * @brief   Sets up the accelerometer to begin reading.
 *  This function steps through all three accelerometer control registers.
 */
static void initAccel(accel_scale_e tmp)
{
    uint8_t cmd[4] = {
        CTRL_REG5_XL,
        0x38,       // reg #1Fh Enable all axis and don't decimate data in out Registers
        (A_ODR_119 << 5) | (tmp << 3) | (A_BW_AUTO_SCALE),   // reg #20h 119 Hz ODR, set scale, and auto BW
        0           // reg #21h Default resolution mode and filtering settings
    };

    // Write the data to the accel control registers
    drv->lsmSpi.write(LSM9DS1_CS_XG, cmd, 4);
}

/* @fn  initMag()
 * @brief   Sets up the magnetometer to begin reading.
 *  This function steps through all three magnetometer control registers.
 */
static void initMag(mag_scale_e tmp)
{
    uint8_t cmd[5] = {
        CTRL_REG1_M,
        0x10,       // reg #20 Default data rate, xy axes mode, and temp comp
        tmp << 5,   // reg #21 Set mag scale
        0,          // reg #22 Enable I2C, write only SPI, not LP mode, Continuous conversion mode
        0xA0        // reg #23 : disable I2C, low-power mode configuration : update rate is 0.625 Hz
    };

    // Write the data to the mag control registers
    drv->lsmSpi.write(LSM9DS1_CS_M, cmd, 5);
}

/* @fn deInitGyro()
 * @brief   Sets gyro to power down.
 */
static void deInitGyro()
{
    // reg #10h  Gyro power down
    uint8_t cmd[2] = {CTRL_REG1_G, 0};

    // Write the data to the gyro control registers
    drv->lsmSpi.write(LSM9DS1_CS_XG, cmd, 2);
}


/* @fn deInitAccel()
 * @brief   Sets accelerometer to power down.
 */
static void deInitAccel()
{
    // reg #20h Accel power down
    uint8_t cmd[2] = {CTRL_REG6_XL, 0 };

    // Write the data to the accel control registers
    drv->lsmSpi.write(LSM9DS1_CS_XG, cmd, 2);
}


/* @fn  deInitMag()
 * @brief   Sets magnetometer to power down.
 */
static void deInitMag()
{
    // reg #22h Mag power down
    uint8_t cmd[2] = {CTRL_REG3_M, 0x3};

    // Write the data to the mag control registers
    drv->lsmSpi.write(LSM9DS1_CS_M, cmd, 2);
}


/* @fn  calcgRes()
 * @brief    Calculate the resolution of the gyroscope.
 *  This function will set the value of the mRes variable according to scale value.
 */
static void calcgRes(gyro_scale_e tmp)
{
    // Possible gyro scales (and their register bit settings) are:
    // 245 DPS (00), 500 DPS (01), 2000 DPS (10).
    switch (tmp)
    {
        case G_SCALE_245DPS:
            gRes = 245.0 / 32768.0;
            break;
        case G_SCALE_500DPS:
            gRes = 500.0 / 32768.0;
            break;
        case G_SCALE_2000DPS:
            gRes = 2000.0 / 32768.0;
            break;
    }
}


/* @fn  calcaRes()
 * @brief    Calculate the resolution of the accelerometer.
 *  This function will set the value of the mRes variable according to scale value.
 */
static void calcaRes(accel_scale_e tmp)
{
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100).
    switch (tmp)
    {
        case A_SCALE_2G:
            aRes = 2.0 / 32768.0;
            break;
        case A_SCALE_4G:
            aRes = 4.0 / 32768.0;
            break;
        case A_SCALE_8G:
            aRes = 8.0 / 32768.0;
            break;
        case A_SCALE_16G:
            aRes = 16.0 / 32768.0;
            break;
    }
}

/* @fn  calcmRes()
 * @brief    Calculate the resolution of the magnetometer.
 *  This function will set the value of the mRes variable according to scale value.
 */
static void calcmRes(mag_scale_e tmp)
{
    // Possible magnetometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10) 12 Gs (11).
    switch (tmp)
    {
        case M_SCALE_4GS:
            mRes = 4.0 / 32768.0;
            break;
        case M_SCALE_8GS:
            mRes = 8.0 / 32768.0;
            break;
        case M_SCALE_12GS:
            mRes = 12.0 / 32768.0;
            break;
        case M_SCALE_16GS:
            mRes = 16.0 / 32768.0;
            break;
    }
}

//-----------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Exported driver functions

uint16_t lsm9ds1_init(  gyro_scale_e    gScl,
                        accel_scale_e   aScl,
                        mag_scale_e     mScl,
                        gyro_odr_e      gODR,
                        accel_odr_e     aODR,
                        mag_odr_e       mODR)
{
    uint16_t ret;
    uint8_t cmd[2];

    calcgRes(gScl); // Calculate DPS / ADC tick, stored in gRes variable
    calcaRes(aScl); // Calculate g / ADC tick, stored in aRes variable
    calcmRes(mScl); // Calculate Gs / ADC tick, stored in mRes variable

    // To verify communication, we need to read from the WHO_AM_I registers of each device.

    cmd[0] = WHO_AM_I;
    drv->lsmSpi.read(LSM9DS1_CS_XG, cmd, 2);
    ret = cmd[1];            // Read the accel/gyro WHO_AM_I

    cmd[0] = WHO_AM_I;
    drv->lsmSpi.read(LSM9DS1_CS_M, cmd, 2);
    ret |= cmd[1]<<8;        // Read the mag WHO_AM_I


    /* we are using default mode: do not configuring CTRL_REG8  */

    if(drv->enable & LSM_GYRO)
    {
        // Gyro
        initGyro(gScl);     // init the gyro. Setting up interrupts, etc.
        setGyroODR(gODR);   // Set the gyro output data rate and bandwidth.
        setGyroScale(gScl); // Set the gyro range
    }
    else
    {
        deInitGyro();
    }

    if(drv->enable & LSM_ACCEL)
    {
        // Accelerometer
        initAccel(aScl);    // init all axes of the accel. Set up interrupts, etc.
        setAccelODR(aODR);  // Set the accel data rate.
        setAccelScale(aScl);// Set the accel range.
    }
    else
    {
        deInitAccel();
    }

    if(drv->enable & LSM_MAG)
    {
        // Magnetometer
        initMag(mScl);      // init the mag. Set up interrupts, etc.
        setMagODR(mODR);    // Set the magnetometer output data rate.
        setMagScale(mScl);  // Set the magnetometer's range.
    }
    else
    {
        deInitMag();
    }

    return (ret);          //should be (WHO_AM_I_M<<8 + WHO_AM_I)
}

void lsm9ds1_deinit()
{
    if(drv)
    {
        deInitGyro();
        deInitAccel();
        deInitMag();
    }
}


void readGyro()
{
    if(drv->enable & LSM_GYRO)
    {
        uint8_t data[7];

        data[0] = OUT_X_L_G;

        // Read in 1+6 bytes registers containing the axes data
        drv->lsmSpi.read(LSM9DS1_CS_XG, data, 7);

        drv->gx_raw = data[1] | (data[2] << 8);
        drv->gy_raw = data[3] | (data[4] << 8);
        drv->gz_raw = data[5] | (data[6] << 8);

        drv->gx = drv->gx_raw * gRes;
        drv->gy = drv->gy_raw * gRes;
        drv->gz = drv->gz_raw * gRes;
    }
    else
    {
        drv->gx_raw = drv->gy_raw = drv->gz_raw = LSM_NO_DATA;
        drv->gx = drv->gy = drv->gz = LSM_NO_DATA;
    }
}

void readAccel()
{
    if(drv->enable & LSM_ACCEL)
    {
        uint8_t data[7];

        data[0] = OUT_X_L_XL;

        // Read in 1+6 bytes registers containing the axes data
        drv->lsmSpi.read(LSM9DS1_CS_XG, data, 7);

        drv->ax_raw = data[1] | (data[2] << 8);
        drv->ay_raw = data[3] | (data[4] << 8);
        drv->az_raw = data[5] | (data[6] << 8);
        // convert to g
        drv->ax = drv->ax_raw * aRes;
        drv->ay = drv->ay_raw * aRes;
        drv->az = drv->az_raw * aRes;
    }
    else
    {
        drv->ax_raw = drv->ay_raw = drv->az_raw = LSM_NO_DATA;
        drv->ax = drv->ay = drv->az = LSM_NO_DATA;
    }

}

void readMag()
{
    if(drv->enable & LSM_MAG)
    {
        uint8_t data[7];

        data[0] = OUT_X_L_M;

        // Read in 1+6 bytes registers containing the axes data
        drv->lsmSpi.read(LSM9DS1_CS_M, data, 7);

        drv->mx_raw = data[1] | (data[2] << 8);
        drv->my_raw = data[3] | (data[4] << 8);
        drv->mz_raw = data[5] | (data[6] << 8);
        // convert to Gs
        drv->mx = drv->mx_raw * mRes;
        drv->my = drv->my_raw * mRes;
        drv->mz = drv->mz_raw * mRes;
    }
    else
    {
        drv->mx_raw = drv->my_raw = drv->mz_raw = LSM_NO_DATA;
        drv->mx = drv->my = drv->mz = LSM_NO_DATA;
    }
}

int gyroReady()
{
    int ret=0;

    if(drv->enable & LSM_GYRO)
    {
        uint8_t cmd[2] = { STATUS_REG_0, 0 };
        drv->lsmSpi.read(LSM9DS1_CS_XG, cmd, 2);
        ret = (cmd[1] & 0x02)>>1;
    }
    return ret;
}

int accelReady()
{
    int ret=0;

    if(drv->enable & LSM_ACCEL)
    {
        uint8_t cmd[2] = { STATUS_REG_0, 0 };
        drv->lsmSpi.read(LSM9DS1_CS_XG, cmd, 2);
        ret = cmd[1] & 0x01;
    }
    return ret;
}

int magReady()
{
    int ret=0;

    if(drv->enable & LSM_MAG)
    {
        uint8_t cmd[2] = { STATUS_REG_M, 0 };
        drv->lsmSpi.read(LSM9DS1_CS_M, cmd, 2);
        ret = cmd[1] & 0x07;
    }
    return ret;
}

void readTemp()
{
    if(drv->enable & LSM_TEMP)
    {
        // The data we will read from the temp
        uint8_t data[3];

        data[0] = OUT_TEMP_L;

        drv->lsmSpi.read(LSM9DS1_CS_XG, data, 3);

        // Temperature is a 12-bit signed
        drv->temperature_raw = data[1] | (data[2] << 8);

        drv->temperature_c = (float)drv->temperature_raw / 8.0f + 25.0f;
    }
    else
    {
        drv->temperature_raw = LSM_NO_DATA;
        drv->temperature_c   = LSM_NO_DATA;
    }
}

void setGyroScale(gyro_scale_e gScl)
{
    uint8_t cmd[2] = { CTRL_REG1_G, 0 };

    drv->lsmSpi.read(LSM9DS1_CS_XG, cmd, 2);

    cmd[0] = CTRL_REG1_G;
    cmd[1] &= ~(G_SCALE_MASK<<3);
    cmd[1] |= (gScl<<3);

    drv->lsmSpi.write(LSM9DS1_CS_XG, cmd, 2);   // Set the gyro scale

    calcgRes(gScl); // calculate a new gRes
}

void setAccelScale(accel_scale_e aScl)
{
    uint8_t cmd[2] = {CTRL_REG6_XL, 0};

    drv->lsmSpi.read(LSM9DS1_CS_XG, cmd, 2);

    cmd[0] = CTRL_REG6_XL;
    cmd[1] &= ~(A_SCALE_MASK << 3);
    cmd[1] |= (aScl << 3);

    drv->lsmSpi.write(LSM9DS1_CS_XG, cmd, 2);

    calcaRes(aScl); // calculate a new aRes
}

void setMagScale(mag_scale_e mScl)
{
    uint8_t cmd[2] = {CTRL_REG2_M, 0};

    drv->lsmSpi.read(LSM9DS1_CS_M, cmd, 2);

    cmd[0] = CTRL_REG2_M;
    cmd[1] &= ~(M_SCALE_MASK << 5);  // clear scale
    cmd[1] |= mScl << 5;

    drv->lsmSpi.write(LSM9DS1_CS_M, cmd, 2);

    calcmRes(mScl); // calculate a new mRes
}

void setGyroODR(gyro_odr_e gRate)
{
    uint8_t cmd[2] = {CTRL_REG1_G, 0};

    drv->lsmSpi.read(LSM9DS1_CS_XG, cmd, 2);

    cmd[0] = CTRL_REG1_G;
    cmd[1] &= (M_SCALE_MASK << 3);  // KEEP scale
    cmd[1] |= gRate;                // new odr bits

    drv->lsmSpi.write(LSM9DS1_CS_XG, cmd, 2);
}

void setAccelODR(accel_odr_e aRate)
{
    uint8_t cmd[2] = {CTRL_REG6_XL, 0};

    drv->lsmSpi.read(LSM9DS1_CS_XG, cmd, 2);

    cmd[0] = CTRL_REG6_XL;
    cmd[1] &= ~(0x7 << 5);  // keep scale and bw
    cmd[1] |= aRate << 5;   // new odr bits

    drv->lsmSpi.write(LSM9DS1_CS_XG, cmd, 2);
}

void setMagODR(mag_odr_e mRate)
{
    uint8_t cmd[2] = { CTRL_REG1_M, 0};

    drv->lsmSpi.read(LSM9DS1_CS_M, cmd, 2);

    cmd[0] = CTRL_REG1_M;
    cmd[1] &= ~(0x7 << 2);  // mask out the mag odr bits
    cmd[1] |= mRate << 2;   // new odr bits

    drv->lsmSpi.write(LSM9DS1_CS_M, cmd, 2);
}
