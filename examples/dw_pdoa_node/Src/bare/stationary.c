/*! ----------------------------------------------------------------------------
 * @file
 * @brief   stationary function
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
#include "lsm9ds1.h"
#include "stationary.h"

#include "usb_uart_tx.h"

/* Accelerometer floating point "1g" will be normalized to the number below */
#define ACCEL_NORMALIZED_RESOLUTION (1000)


/*
 * @brief
 *  externaly defined function to receive accelerometer's data
 *
 * */
__weak void stationary_accel_data_cb (int16_t ax, int16_t ay, int16_t az)
{

}

/* Exported functions */

int simple_stationary(lsm9ds_driver_t *drv, uint16_t delay_ms)
{
    static int      stationary = 0;
    static int16_t  prev_acc_x = 0, prev_acc_y = 0, prev_acc_z = 0;
    static uint16_t consequence_moving_detection = 0;
    static uint16_t consequence_stationary_detection = 0;

    if (drv)
    {
        /* Stationary state detection. Proceed if new accelerometer data is available. */
        if (drv->accelReady())
        {
            /* Read raw acceleration values. */
            drv->readAccel();

            /* Normalize accelerometer output */
            int16_t     ax, ay, az;

            /*
             * ACCEL_NORMALIZED_RESOLUTION is for normal circumstances, i.e. for 1g.
             * Every axis is in range
             * [-Max*ACCEL_NORMALIZED_RESOLUTION ... Max*ACCEL_NORMALIZED_RESOLUTION]
             * where Max is related to a acceleration circumstances.
             * For normal circumstances, where the acceleration is due to gravity, Max = 1g.
             * It can be higher if tag is in the car or runway.
             * */
            ax = (int16_t)(ACCEL_NORMALIZED_RESOLUTION* drv->ax);
            ay = (int16_t)(ACCEL_NORMALIZED_RESOLUTION* drv->ay);
            az = (int16_t)(ACCEL_NORMALIZED_RESOLUTION* drv->az);

            /* Send normalized to values to the callback*/
            stationary_accel_data_cb(ax, ay, az);

            /* Check if any axis of acceleration has changed "significantly" */
            if ((abs(ax - prev_acc_x) + (abs(ay - prev_acc_y) + (abs(az - prev_acc_z))))
               >
               app.pConfig->s.acc_threshold )
            {
                if (stationary)
                {
                    /* Check if acceleration values have been evolving for long enough to consider state is moving */
                    consequence_moving_detection++;
                    if (consequence_moving_detection > (app.pConfig->s.acc_moving_ms/delay_ms))
                    {
                        /* Acceleration values have changed significantly for several readings now, we are moving. */
                        stationary = 0;
                        consequence_stationary_detection = 0;
                    }
                }
                else
                {
                    /* We are moving: reduce the "stationary" counter, if stationary state was detected during movement */
                    consequence_stationary_detection = (consequence_stationary_detection > 1) ? (consequence_stationary_detection-2) : (0);
                }
            }
            else
            {
                if (stationary)
                {
                    /* We are stationary: reduce the "moving" counter, if movement state was detected during stationary */
                    consequence_moving_detection = (consequence_moving_detection > 1) ? (consequence_moving_detection-2) : (0);
                }
                else
                {
                    /* Check if acceleration values have been steady for long enough to consider state is not moving */
                    consequence_stationary_detection++;
                    if (consequence_stationary_detection > (app.pConfig->s.acc_stationary_ms/delay_ms))
                    {
                        /* Acceleration values have been steady for several readings now, we are stationary. */
                        stationary = 1;
                        consequence_moving_detection = 0;
                    }
                }
            }


            /* Update previous values. */
            prev_acc_x = ax;
            prev_acc_y = ay;
            prev_acc_z = az;
        }
    }

    return stationary;
}
