/*
 * @file      task_imu.c
 * @brief
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#include <task_imu.h>

#define IMU_READ_DELAY_MS    (100)

static lsm9ds_driver_t * lsmdrv = NULL;


//-----------------------------------------------------------------------------
// extern functions to report output data
extern void send_to_pc_stationary(stationary_res_t *p);

//-----------------------------------------------------------------------------
void stationary_accel_data_cb(int16_t acc_x, int16_t acc_y, int16_t acc_z);

/*
 * @brief callback for normalized values from accelerometer sensor
 * updates values for reporting to the Node (over the air)
 *
 * Reports to the PC values on moving every time it is called.
 * If stationary was detected, sends last stationary data only once.
 *
 * */
void stationary_accel_data_cb(int16_t acc_x, int16_t acc_y, int16_t acc_z)
{
    static bool prev = false;
    twr_info_t  *pTwrInfo;
    pTwrInfo = getTwrInfoPtr();

    taskENTER_CRITICAL();
    pTwrInfo->acc.acc_x[0] = acc_x    &0xff;
    pTwrInfo->acc.acc_x[1] = (acc_x>>8) &0xff;
    pTwrInfo->acc.acc_y[0] = acc_y    &0xff;
    pTwrInfo->acc.acc_y[1] = (acc_y>>8) &0xff;
    pTwrInfo->acc.acc_z[0] = acc_z    &0xff;
    pTwrInfo->acc.acc_z[1] = (acc_z>>8) &0xff;
    taskEXIT_CRITICAL();

    if (!prev || (prev && !pTwrInfo->stationary_imu))
    {
        stationary_res_t    p;
        p.addr = AR2U16(pTwrInfo->env.tagAddr);
        p.flag = pTwrInfo->stationary_imu;
        p.acc_x = acc_x;
        p.acc_y = acc_y;
        p.acc_z = acc_z;

        send_to_pc_stationary(&p);
    }

    prev = pTwrInfo->stationary_imu;
}

/* @brief sets the stationary indicator flag in shared parameters
 *           (we can use other methods to inform other threads that
 *            IMU is stationary)
 * */
static bool run_imu(void)
{
    bool ret = FALSE;

    if (simple_stationary(lsmdrv, IMU_READ_DELAY_MS))
    {
        ret = TRUE;
        HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, 0);
    }
    else
    {
        HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, 1);
    }

    return (ret);
}


/*
 * */
static bool start_imu(void)
{
    bool    ret = TRUE;

    //power ON IMU
    HAL_GPIO_WritePin(MEMS_PWRON_GPIO_Port, MEMS_PWRON_Pin, 0);

    //init IMU SPI and read IMU ID : this may fail sometimes

    lsmdrv = lsm9ds1_driver_open(LSM_ACCEL);

    if(!lsmdrv)
    {
        error_handler(0, _ERR_IMU_INIT);
        ret = FALSE;
    }

    return (ret);
}

/*
 * */
static void stop_imu(void)
{
    //deinit IMU SPI
    lsm9ds1_driver_close();
    lsmdrv = NULL;
    //power OFF IMU
    HAL_GPIO_WritePin(MEMS_PWRON_GPIO_Port, MEMS_PWRON_Pin, 1);

}


//-----------------------------------------------------------------------------

/* Bckgrnd Imu Service Task */
void ImuTask(void const * argument)
{
    bool        imu_enabled;
    twr_info_t  *pTwrInfo;

    do{
        osDelay(1000);
    }while(!(pTwrInfo = getTwrInfoPtr()));    //wait for initialization of pTwrInfo

    stop_imu();

    imu_enabled = FALSE;
    pTwrInfo->stationary_imu = FALSE;

    while(1)
    {
        osMutexRelease(app.imuTask.MutexId);

        osDelay(IMU_READ_DELAY_MS / portTICK_PERIOD_MS);

        osMutexWait(app.imuTask.MutexId, 0);

        if(pTwrInfo->env.imuOn)
        {
            if(imu_enabled)
            {
                pTwrInfo->stationary_imu = run_imu();
            }
            else
            {
                imu_enabled = start_imu();
            }
        }
        else
        {
            if(imu_enabled)
            {
                stop_imu();
                imu_enabled = FALSE;
                pTwrInfo->stationary_imu = FALSE;
            }
        }

        osThreadYield();
    }
}
