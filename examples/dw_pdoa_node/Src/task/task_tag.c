/*!----------------------------------------------------------------------------
 * @file    task_tag.c
 * @brief   DecaWave Application Layer
 *          RTOS tag implementation
 *
 * @attention
 *
 * Copyright 2016-2017 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author
 *
 * DecaWave
 */

//TBD  remove unwanted Header files
#include <error.h>
#include "cmsis_os.h"
#include <util.h>
#include <circ_buf.h>
//#include <task_imu.h>
#include <task_tag.h>
#include <limits.h>		
#include <port_platform.h>
#include <tag.h>
#include <util.h>
#include <uwb_frames.h>
#include "croutine.h"
//-----------------------------------------------------------------------------

#define BLINK_PERIOD_MS            (500)    /* range init phase - blink sends period, ms */

#define UNUSED(X)  ((void)(X))		// TBD

extern void send_to_pc_tag_location(twr_res_ext_t*);
//-----------------------------------------------------------------------------

/* @brief    The software timer.
 *  It is signalling to the TwrBlinkTask to transmit a blink.
 *  Will be stopped on reception of Ranging Config response from a Node
 * */
static void
BlinkTimer_cb(void const *arg)
{
    if(app.blinkTask.Handle) //RTOS : blinkTask can be not started yet
    {
        if(osSignalSet(app.blinkTask.Handle, app.blinkTask.Signal) != osOK)
        {
            error_handler(1, _Err_Signal_Bad);
        }
    }
    UNUSED(arg);
}


/*
 * @brief
 *     The thread is initiating the transmission of the blink
 *     on reception of app.blinkTask.Signal
 *
 * */
static void
BlinkTask(void const * arg)
{
    twr_info_t     *p;

    do{
        osDelay(100);
    }while(!(p = getTwrInfoPtr()));    //wait for initialisation of pTwrInfo


    do {
        osMutexRelease(app.blinkTask.MutexId);

        osSignalWait(app.blinkTask.Signal, osWaitForever);

        osMutexWait(app.blinkTask.MutexId, 0);    //we do not want the task can be deleted in the middle of operation

        uint32_t new_blink_period;
        new_blink_period = (uint32_t)((float)((BLINK_PERIOD_MS/3.0)*rand()/RAND_MAX));
        new_blink_period += BLINK_PERIOD_MS;
        
        new_blink_period /= portTICK_PERIOD_MS;

        /* start discovery : start blinking timer */
        if(osTimerStart(app.blinkTmr.Handle, new_blink_period) != osOK)
        {
            error_handler(1,_Err_Timer_Start_Bad);
        }

        taskENTER_CRITICAL();
        tag_wakeup_dw1000_blink_poll(p);
        taskEXIT_CRITICAL();

        initiator_send_blink(p);

    }while(1);

    UNUSED(arg);
}


/*
 * @brief
 *     The thread is initiating the TWR sequence
 *  on reception of .signal.twrTxPoll
 *
 * */
static void
TagPollTask(void const * arg)
{
    twr_info_t     *p = getTwrInfoPtr();

    do{
        osDelay(100);
    }while(!(p = getTwrInfoPtr()));    //wait for initialisation of pTwrInfo


    do {
        osMutexRelease(app.pollTask.MutexId);

        osSignalWait(app.pollTask.Signal, osWaitForever);

        osMutexWait(app.pollTask.MutexId, 0);

        initiator_wake_and_send_poll(p);

    }while(1);

    UNUSED(arg);
}


/* @brief DW1000 RX : RTOS implementation
 *
 * */
static void
TagRxTask(void const * arg)
{
    error_e     ret;
    uint16_t    head, tail;

    twr_info_t  *ptwrInfo;

    do{
        osDelay(10);
    }while(!(ptwrInfo = getTwrInfoPtr()));    //wait for initialisation of pTwrInfo

    int size = sizeof(ptwrInfo->rxPcktBuf.buf) / sizeof(ptwrInfo->rxPcktBuf.buf[0]);

    do {
        osMutexRelease(app.rxTask.MutexId);

        osSignalWait(app.rxTask.Signal, osWaitForever);

        osMutexWait(app.rxTask.MutexId, 0);

        taskENTER_CRITICAL();
        head = ptwrInfo->rxPcktBuf.head;
        tail = ptwrInfo->rxPcktBuf.tail;
        taskEXIT_CRITICAL();

        /* We are using circular buffer + Signal to safely deliver packets from ISR to APP */
        if(CIRC_CNT(head,tail,size) > 0)
        {
            rx_pckt_t       *prxPckt    = &ptwrInfo->rxPcktBuf.buf[tail];
            twr_res_ext_t   p;

            ret = twr_initiator_algorithm_rx(prxPckt, ptwrInfo); /**< Run bare twr_initiator_algorithm */

            if (ret == _No_Err_Response)
            {
                ptwrInfo->faultyRangesCnt = 0;

                p.addr          = AR2U16(ptwrInfo->env.tagAddr);
                p.node_addr     = AR2U16(ptwrInfo->env.nodeAddr);
                p.rNum          = prxPckt->msg.respExtMsg.resp.rNum;
                p.x_cm          = (int16_t)AR2U16(prxPckt->msg.respExtMsg.resp.x_cm);
                p.y_cm          = (int16_t)AR2U16(prxPckt->msg.respExtMsg.resp.y_cm);
                p.clkOffset_pphm= (int16_t)AR2U16(prxPckt->msg.respExtMsg.resp.clkOffset_pphm); //Crystal Clock offset value reported back from the Node

                {//XTAL trimming will be performed after sending of Final.
                    /* Instead of using a clkOffset_pphm from Response, which calculated by Node based on distances measurements,
                     * the more precise and direct method of adjusting clock offset using
                     * carrier integrator counter value will be used.*/
                    float co_ppm;
                    co_ppm= dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 );
                    ptwrInfo->clkOffset_pphm = (int)(co_ppm * 100); //save the offset value for future apply after Final message been sent
                }

            }

            taskENTER_CRITICAL();
            tail = (tail + 1) & (size-1);
            ptwrInfo->rxPcktBuf.tail = tail;
            taskEXIT_CRITICAL();

            /* ready to serve next raw reception */
#if 0		// TBD
            /* Report previous range/angle/offset back to UART/USB */
            if((app.pConfig->s.reportLevel) && (ret == _No_Err_Response))
            {
                send_to_pc_tag_location(&p);
            }
#endif
        }

        osThreadYield();
    }while(1);

    UNUSED(arg);
}


/* @brief Setup TWR tasks and timers for discovery phase.
 *         - blinking timer
 *         - blinking task
 *          - twr polling task
 *         - rx task
 * Only setup, do not start.
 * */
static void tag_setup_tasks(void)
{

#if	0	// TBD
    /* Setup IMU task.
     * It reports a stationary status of the tag.
     * */
    osThreadDef(imuTask, ImuTask, osPriorityBelowNormal, 0, 256);
    osMutexDef(imuMutex);
#endif
    /* Setup initial Software timer for discovery phase.
     * It works until Ranging Config message will be received
     * */
    osTimerDef(blTmr, BlinkTimer_cb);

    /* Blinking thread for discovery phase of the Anchor until reception of Range Init */
    osThreadDef(blinkTask, BlinkTask, osPriorityNormal, 0, 400);
    osMutexDef(blinkMutex);

    /* This will be the main poll thread for the Tag after discovery completed
     * Do not reduce the stack size for this thread.
     * */
    osThreadDef(pollTask, TagPollTask, osPriorityAboveNormal, 0, 400);
    osMutexDef(pollMutex);

    /* rxThread is passing signal from RX IRQ to an actual two-way ranging algorithm.
     * It awaiting of Rx Signal from RX IRQ ISR and decides what to do next in TWR exchange process.
     * Do not reduce the stack size for this thread.
     * */
    osThreadDef(rxTask, TagRxTask, osPriorityRealtime, 0, 400);
    osMutexDef(rxMutex);

    app.blinkTask.Handle    = osThreadCreate(osThread(blinkTask), NULL);
    app.blinkTask.MutexId   = osMutexCreate(osMutex(blinkMutex));

    app.blinkTmr.Handle     = osTimerCreate(osTimer(blTmr), osTimerPeriodic, NULL);
#if	0	// TBD
    app.imuTask.Handle      = osThreadCreate(osThread(imuTask), NULL);
    app.imuTask.MutexId     = osMutexCreate(osMutex(imuMutex));
#endif

    app.pollTask.Handle     = osThreadCreate(osThread(pollTask), NULL);
    app.pollTask.MutexId    = osMutexCreate(osMutex(pollMutex));

    app.rxTask.Handle       = osThreadCreate(osThread(rxTask), NULL);
    app.rxTask.MutexId      = osMutexCreate(osMutex(rxMutex));

	// TBD Check for - app.imuTask.  Handle == NULL
    if( (app.blinkTmr. Handle == NULL)   ||\
        (app.blinkTask.Handle == NULL)   ||\
        (app.pollTask. Handle == NULL)   ||\
        (app.rxTask.   Handle == NULL))
    {
        error_handler(1, _Err_Create_Task_Bad);
    }
}

//-----------------------------------------------------------------------------

/* @brief
 *      Stop and delete blink timer and blink timer Task
 * */
void blink_timer_delete(void)
{
    taskENTER_CRITICAL();

    if(app.blinkTmr.Handle)
    {
        osTimerStop(app.blinkTmr.Handle);

        if (osTimerDelete(app.blinkTmr.Handle) == osOK)
        {
            app.blinkTmr.Handle = NULL;
        }
        else
        {
            error_handler(1, _ERR_Cannot_Delete_Timer);
        }
    }

    taskEXIT_CRITICAL();
}

void blink_task_delete(void)
{
    if(app.blinkTask.Handle)
    {
        osMutexWait(app.blinkTask.MutexId, osWaitForever);
        taskENTER_CRITICAL();
        osMutexRelease(app.blinkTask.MutexId);
        if(osThreadTerminate(app.blinkTask.Handle) == osOK)
        {
            osMutexDelete(app.blinkTask.MutexId);
            app.blinkTask.Handle = NULL;
        }
        else
        {
            error_handler(1, _ERR_Cannot_Delete_Task);
        }
        taskEXIT_CRITICAL();
    }
}

//-----------------------------------------------------------------------------

/* @brief
 *      Kill all task and timers related to TWR if any
 *      DW1000's RX and IRQ shall be switched off before task termination,
 *      that IRQ will not produce unexpected Signal
 * */
void tag_terminate_tasks(void)
{
//    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);		// TBD

    blink_timer_delete();

    blink_task_delete();
#if	0 // TBD
    if(app.imuTask.Handle)
    {
        osMutexWait(app.imuTask.MutexId, osWaitForever);
        taskENTER_CRITICAL();
        osMutexRelease(app.imuTask.MutexId);
        if(osThreadTerminate(app.imuTask.Handle) == osOK)
        {
            osMutexDelete(app.imuTask.MutexId);
            app.imuTask.Handle = NULL;
        }
        else
        {
            error_handler(1, _ERR_Cannot_Delete_Task);
        }
        taskEXIT_CRITICAL();
    }
#endif
    if(app.rxTask.Handle)
    {
        osMutexWait(app.rxTask.MutexId, osWaitForever);
        taskENTER_CRITICAL();
        osMutexRelease(app.rxTask.MutexId);
        if(osThreadTerminate(app.rxTask.Handle) == osOK)
        {
            osMutexDelete(app.rxTask.MutexId);
            app.rxTask.Handle = NULL;
        }
        else
        {
            error_handler(1, _ERR_Cannot_Delete_Task);
        }
        taskEXIT_CRITICAL();
    }

    if(app.pollTask.Handle)
    {
        osMutexWait(app.pollTask.MutexId, osWaitForever);
        taskENTER_CRITICAL();
        osMutexRelease(app.pollTask.MutexId);
        if(osThreadTerminate(app.pollTask.Handle) == osOK)
        {
            osMutexDelete(app.pollTask.MutexId);
            app.pollTask.Handle = NULL;
        }
        else
        {
            error_handler(1, _ERR_Cannot_Delete_Task);
        }
        taskEXIT_CRITICAL();
    }

    tag_process_terminate();    //de-allocate Tag RAM Resources
}


/* @fn         tag_helper
 * @brief     this is a service function which starts the Tag
 * top-level application
 *
 * */
void tag_helper(void const *argument)
{
    error_e     tmp;

    port_disable_wake_init_dw();

    taskENTER_CRITICAL();    /**< When the app will setup RTOS tasks, then if task has a higher priority,
                                 the kernel will start it immediately, thus we need to stop the scheduler.*/
    set_dw_spi_fast_rate(DW_MASTER);

    tag_setup_tasks();        /**< "RTOS-based" : setup all RTOS tasks. */

    /* "RTOS-independent" part : initialisation of two-way ranging process */
    tmp = tag_process_init();

    if( tmp != _NO_ERR)
    {
        error_handler(1, tmp);
    }

    /* start discovery : start blinking timer */
    if(osTimerStart(app.blinkTmr.Handle, BLINK_PERIOD_MS/portTICK_PERIOD_MS) != osOK)
    {
        error_handler(1,_Err_Timer_Start_Bad);
    }

    tag_process_start();

    taskEXIT_CRITICAL();    /**< all RTOS tasks can be scheduled */
}


