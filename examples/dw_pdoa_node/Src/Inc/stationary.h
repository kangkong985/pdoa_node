/*! ----------------------------------------------------------------------------
 * @file    stationary.h
 *
 * @brief   header for stationary function
 *
 *          This function uses the reading of acceleration to determine if the device is stationary or moving, by monitoring the acceleration
 *          values evolution.
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#ifndef __STATIONARY_H_
#define __STATIONARY_H_ 1

#include <stdint.h>

#include "lsm9ds1.h"

/*! ----------------------------------------------------------------------------
 * @fn simple_stationary()
 *
 * @brief
 *      This function uses the acceleration values on all axes to determine
 *      if the device is moving or stationary.
 *
 * @param
 * *drv - pointer to the driver instance
 *          driver shall provide two functions:
 *          drv->accelReady() - IMU has accelerometer data ready
 *          drv->readAccel()  - IMU read accelerometer data
 *          and three output values
 *          drv-> ax ay az - normalized to "1g" values of X,Y,Z axis
 * delay_ms - the delay, at which this called from the upper application, ms
 *
 * output parameters none
 *
 * returns 1 if the device is stationary, 0 if the device is moving.
 */
int simple_stationary(lsm9ds_driver_t *drv, uint16_t delay_ms);


#endif /* __STATIONARY_H_ */
