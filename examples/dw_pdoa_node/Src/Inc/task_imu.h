/*
 *  @file    task_imu.h
 *  @brief
 *
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __IMU_TASK__H__
#define __IMU_TASK__H__ 1

#ifdef __cplusplus
 extern "C" {
#endif

#include "node.h"

#include "stationary.h"

void stationary_accel_data_cb(int16_t x, int16_t y, int16_t z);
void ImuTask(void const * argument);

typedef struct {
    uint16_t addr;
    bool     flag;
    int16_t  acc_x;
    int16_t  acc_y;
    int16_t  acc_z;
}stationary_res_t;

#ifdef __cplusplus
}
#endif

#endif /* __IMU_TASK__H__ */
