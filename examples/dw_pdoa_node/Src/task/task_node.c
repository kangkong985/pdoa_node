/*
 * @file      node_task.c
 * @brief
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 ==============================================================================
 =                                                                            =
 =                                                                            =
 =                D E C A W A V E    C O N F I D E N T I A L                  =
 =                                                                            =
 =                                                                            =
 ==============================================================================

 This software contains Decawave confidential information and techniques,
 subject to licence and non-disclosure agreements.  No part of this software
 package may be revealed to any third-party without the express permission of
 Decawave Ltd.

 ==============================================================================
 */
#include "node.h"
#include "util.h"

#include "circ_buf.h"
#include "dw_pdoa_node_common.h"

//#include "data_filter.h"

#include "nrf_timer.h"

//#include <task_imu.h>
#include <task_node.h>

//-----------------------------------------------------------------------------
// extern functions to report output data
extern void send_to_pc_diag_acc(rx_mail_t *pRxMailPckt);
extern void send_to_pc_twr(result_t *pRes);
extern void soft_crc16_init(void);

//-----------------------------------------------------------------------------

/*
 * @brief Node RTOS implementation
 *          This is normal priority task, which is awaiting for
 *          a mail with a new reception of raw data from a tag.
 *
 *          On reception of a raw data mail it's performing the calculation
 *          of range and pDoa and reports the results.
 * */
static void CalcTask(void const * arg)
{
    twr_info_t    *pTwrInfo;

    pTwrInfo = getTwrInfoPtr();
	soft_crc16_init();
    osMutexDef(calcMutex);
    app.calcTask.MutexId = osMutexCreate(osMutex(calcMutex));

    do {

        osMutexRelease(app.calcTask.MutexId);

        osEvent event = osMailGet(app.rxPcktPool_q_id, osWaitForever);

        osMutexWait(app.calcTask.MutexId, 0);
        //Received the mail in RxPckt structure format : it includes all necessary raw data to
        //calculate range, phase difference, (diagnostics and accumulators if any), etc.
        nrf_gpio_pin_toggle(LED_STATIONARY);
        if(event.status != osEventMail)
        {
            continue;
        }

        rx_mail_t   *pRxMailPckt;
        result_t    *pRes;
        uint16_t    slot;

        pRxMailPckt = (rx_mail_t *)(event.value.p);    // ".p" indicates that the message is a pointer

        slot = pRxMailPckt->tag->slot;

        pRes = &pTwrInfo->result[slot];

        memcpy(pRes, &pRxMailPckt->res, sizeof(result_t));

        int32_t tofi = INVALID_TOF;
        //volatile int32_t tofi = INVALID_TOF;  // TBD - Debug Purpose

        {    //calculate the time of fly and phase difference

            int64_t     Rb, Da, Ra, Db ;
            pdoa_info_t *pA = &pRxMailPckt->pdoa_info;

            uint64_t    tagPollTxTime;
            uint64_t    tagRespRxTime;
            uint64_t    tagFinalTxTime;

            uint64_t    nodeRespTxTime;
            uint64_t    nodePollRxTime;
            uint64_t    nodeFinalRxTime;

            float    RaRbxDaDb = 0;
            float    RbyDb = 0;
            float    RayDa = 0;

            TS2U64_MEMCPY(tagRespRxTime, pRxMailPckt->tagRespRx_ts);
            TS2U64_MEMCPY(tagPollTxTime, pRxMailPckt->tagPollTx_ts);
            TS2U64_MEMCPY(tagFinalTxTime, pRxMailPckt->tagFinalTx_ts);

            TS2U64_MEMCPY(nodeRespTxTime, pRxMailPckt->nodeRespTx_ts);
            TS2U64_MEMCPY(nodePollRxTime, pRxMailPckt->nodePollRx_ts);
            TS2U64_MEMCPY(nodeFinalRxTime, pRxMailPckt->nodeFinalRx_ts);

            pRes->pdoa_raw_deg  = calcPD(pA);

            Ra = (int64_t)((tagRespRxTime - tagPollTxTime) & MASK_40BIT);
            Db = (int64_t)((nodeRespTxTime - nodePollRxTime) & MASK_40BIT);

            Rb = (int64_t)((nodeFinalRxTime - nodeRespTxTime) & MASK_40BIT);
            Da = (int64_t)((tagFinalTxTime - tagRespRxTime) & MASK_40BIT);

            RaRbxDaDb = (((float)Ra))*(((float)Rb)) - (((float)Da))*(((float)Db));

            RbyDb = ((float)Rb + (float)Db);

            RayDa = ((float)Ra + (float)Da);

            tofi = (int32) ( RaRbxDaDb/(RbyDb + RayDa) );

//			printf("tof = %d\r\n", tofi);

            // Compute clock offset, in parts per million
            pRes->clockOffset_pphm = ((((float)Ra / 2) - (float)tofi) / (float)Db)
                                  - ((((float)Rb / 2) - (float)tofi) / (float)Da);

            pRes->clockOffset_pphm *= 1e8 ; //hundreds of ppm

        }

        /* reportTOF : correct Range bias if needed */
        float r_m;

        if(tof2range(app.pConfig, &r_m, tofi) == _NO_ERR)
        {
            pRes->dist_cm = r_m*100.0;

            pdoa2XY(pRes, (float)app.pConfig->s.pdoaOffset_deg);

            /* Transmit the report to the PC */
           send_to_pc_twr(pRes);
           //send_to_pc_diag_acc(pRxMailPckt);       //This maybe a slow blocking send of large amount of data
                                                    //FlushTask should have higher priority than CalckTask
        }
        else
        {
            pRes->dist_cm = (float)(0xDEADBEEF);
            pRes->x_cm    = (float)(0xDEADBEEF);
            pRes->y_cm    = (float)(0xDEADBEEF);

//            if(app.pConfig->s.debugEn)
//            {
//                const char text[] = "Bad Range \r\n";
//                port_tx_msg((uint8_t*)text, strlen(text));
//            }
        }

        /* remove the message from the mail queue */
        osMailFree(app.rxPcktPool_q_id, pRxMailPckt);

    }while(1);

    UNUSED(arg);
}

//extern bool start_not;
//extern bool running;
//extern nrf_drv_timer_t TIMER_MONITOR;

/* @brief DW1000 RX : Node RTOS implementation
 *          this is a high-priority task, which will be executed immediately
 *          on reception of waiting Signal. Any task with lower priority will be interrupted.
 *          No other tasks in the  system should have higher priority.
 * */
static void RxTask(void const * arg)
{
    int         head, tail, size, tmp;
    error_e     ret;

    twr_info_t  *pTwrInfo;
    pTwrInfo = getTwrInfoPtr();

    size = sizeof(pTwrInfo->rxPcktBuf.buf) / sizeof(pTwrInfo->rxPcktBuf.buf[0]);

    osMutexDef(rxMutex);
    app.rxTask.MutexId = osMutexCreate(osMutex(rxMutex));

    taskENTER_CRITICAL();
    set_SPI_master();
    dwt_rxenable(0);        /**< enable the receiver on the MASTER chip : start the Node RX operation */
    taskEXIT_CRITICAL();

//	add_tag_to_knownTagList(0x082261440D73120F, 0x120F);

    do{
        osMutexRelease(app.rxTask.MutexId);
		
        /* ISR is delivering RxPckt via circ_buf & Signal.
         * This is the fastest method.
         * */

//		printf("Line: %d waiting\r\n", __LINE__);
	
        osSignalWait(app.rxTask.Signal, osWaitForever);

        osMutexWait(app.rxTask.MutexId, 0);

        taskENTER_CRITICAL();
        head = pTwrInfo->rxPcktBuf.head;
        tail = pTwrInfo->rxPcktBuf.tail;
        taskEXIT_CRITICAL();
		
//		start_not = true;
		
        if(CIRC_CNT(head,tail,size) > 0)
        {		
            rx_pckt_t *pRxPckt  = &pTwrInfo->rxPcktBuf.buf[tail];

            ret = twr_responder_algorithm_rx(pRxPckt, pTwrInfo); /**< Run the bare-metal algorithm */
			
//            printf("ret = %d from 0x%02x%02x\r\n", ret, pRxPckt->msg.stdMsg.mac.destAddr[1], pRxPckt->msg.stdMsg.mac.destAddr[0]);

//			if (ret == _Err_Not_Twr_Frame)
//				printf("Func:%s, Line:%d, ret = _Err_Not_Twr_Frame from 0x%x%x\r\n", __FUNCTION__, __LINE__, pRxPckt->msg.stdMsg.mac.destAddr[1], pRxPckt->msg.stdMsg.mac.destAddr[0]);

//            osDelay(1);
			
            if (ret == _No_Err_Final)
            {
                /* Mail the RxPckt to someone : another RTOS thread is awaiting for it */
                rx_mail_t        *pMail;
                final_accel_t    *pFinalMsg;

//				if (!nrfx_timer_is_enabled(&TIMER_MONITOR))
//					nrf_drv_timer_enable(&TIMER_MONITOR);
//
//				running = true;

                pFinalMsg = &pRxPckt->msg.finalMsg.final;
                pMail = osMailAlloc(app.rxPcktPool_q_id, 10); //timeout 10ms
				//printf("f=%s,l=%d pMail=%x\r\n",__FUNCTION__,__LINE__,pMail);
                if(pMail)
                {
                    if(app.pConfig->s.diagEn)
                    {
                        memcpy(&pMail->diagnostics,  &pRxPckt->diagnostics[0], sizeof(pMail->diagnostics));
                    }

                    if(app.pConfig->s.accEn)
                    {
                        memcpy(&pMail->acc,  &pRxPckt->acc[0], sizeof(pMail->acc));
                    }

                    /* Tag's information */
                    pMail->tag = pRxPckt->tag;
                    pMail->res.addr16   = pRxPckt->tag->addr16;
                    pMail->res.rangeNum = pFinalMsg->rNum;

                    /* Angle information */
                    memcpy(&pMail->pdoa_info,  &pRxPckt->pdoa_info, sizeof(pMail->pdoa_info));

                    /* Accelerometer state of the tag: from pFinalMsg */
                    pMail->res.flag =  (uint16_t)(pFinalMsg->flag);
                    pMail->res.acc_x = (int16_t)(AR2U16(pFinalMsg->acc_x));
                    pMail->res.acc_y = (int16_t)(AR2U16(pFinalMsg->acc_y));
                    pMail->res.acc_z = (int16_t)(AR2U16(pFinalMsg->acc_z));

                    /* Convert raw reading of temperature using calibrated values for chips.
                     * Note, the calibrated values available only after dwt_initialize() */
                    float  temp;
                    temp = (uint8_t)pRxPckt->temperature_Master;
                    temp = 23.0 + (float)(temp - pTwrInfo->TmeasMaster)*1.14;   /**< UserManual 2.10: formula for conversion */
                    pMail->res.tMaster_C = (int8_t)(temp);

                    temp = (uint8_t)pRxPckt->temperature_Slave;
                    temp = 23.0 + (float)(temp - pTwrInfo->TmeasSlave)*1.14;    /**< UserManual 2.10: formula for conversion */
                    pMail->res.tSlave_C  = (int8_t)(temp);

                    /* Add flags for PC application */
                    if (app.pConfig->s.pdoaOffset_deg == 0)
                    {
                        pMail->res.flag |= RES_FLAG_PDOA_OFFSET_ZERO_BIT;
                    }
                    if (app.pConfig->s.rngOffset_mm == 0)
                    {
                        pMail->res.flag |= RES_FLAG_RANGE_OFFSET_ZERO_BIT;
                    }

                    /* Tag's ranging times: from pFinalMsg */
                    TS2TS_MEMCPY(pMail->tagPollTx_ts,   pFinalMsg->pollTx_ts);
                    TS2TS_MEMCPY(pMail->tagRespRx_ts,   pFinalMsg->responseRx_ts);
                    TS2TS_MEMCPY(pMail->tagFinalTx_ts,  pFinalMsg->finalTx_ts);

                    /* Node's exchange timings for current Ranging */
                    TS2TS_MEMCPY(pMail->nodePollRx_ts , pRxPckt->nodePollRx_ts);
                    TS2TS_MEMCPY(pMail->nodeRespTx_ts , pRxPckt->nodeRespTx_ts);
                    TS2TS_MEMCPY(pMail->nodeFinalRx_ts, pRxPckt->timeStamp_Master);

                    tmp = pTwrInfo->pollRtcTimeStamp - pTwrInfo->gRtcSFrameZeroCnt;

                    if(tmp < 0)
                    {
                        tmp += RTC_WKUP_CNT_OVFLW;   // RTC Timer overflow - 24 bit counter
                    }

                    tmp = (int)(tmp * WKUP_RESOLUTION_NS)/1000; //us -  RTC Timer - Each Tick is configured as 30.517 us

                    pMail->res.resTime_us = (tmp);  // resTime_us in us

                    if(osMailPut(app.rxPcktPool_q_id, pMail) != osOK)
                    {
                        error_handler(1, _ERR_Cannot_Send_Mail);
                    }
					
                }
                else
                {
                    error_handler(0, _ERR_Cannot_Alloc_Mail); //non-blocking error : no memory : resources, etc. This can happen when
                                                              //tags exchange received, but previous exchange has not been reported to the host yet
                                                              //For example on Accumulators readings
                }
            }

            if (ret == _No_Err_New_Tag)
            {
                if(addTagToDList(pTwrInfo->newTag_addr64))
                {
                    signal_to_pc_new_tag_discovered(pTwrInfo->newTag_addr64);
                }

                pTwrInfo->newTag_addr64 = 0;
            }

            taskENTER_CRITICAL();
            tail = (tail + 1) & (size-1);
            pTwrInfo->rxPcktBuf.tail = tail;
            taskEXIT_CRITICAL();

            if( (ret != _NO_ERR) )
            {
                /* If the Node is performing a Tx, then the receiver will be enabled
                 * after the Tx automatically and twr_responder_algorithm_rx reports "_NO_ERR".
                 *
                 * Otherwise always re-enable the receiver : unknown frame, not expected tag,
                 * final message, _Err_DelayedTX_Late, etc.
                 * */
                taskENTER_CRITICAL();
                set_SPI_master();
                dwt_setrxtimeout(0);
                dwt_rxenable(0);
                taskEXIT_CRITICAL();
            }

//			if (ret == _No_Err_Final)
//			{
//				taskENTER_CRITICAL();
//				xEventGroupSetBits(app.xStartTaskEvent, Ev_Stop_All);
//				if (master_slave_control % 2)
//					xEventGroupSetBits(app.xStartTaskEvent, Ev_Node_B_Task);
//				else
//					xEventGroupSetBits(app.xStartTaskEvent, Ev_Node_A_Task);
//				master_slave_control++;
//				taskEXIT_CRITICAL();
//			}

            /* ready to serve next raw reception */
        }

        osThreadYield();
    }while(1);

    UNUSED(arg);
}

//-----------------------------------------------------------------------------

/* @brief Setup TWR tasks and timers for discovery phase.
 *         - blinking timer
 *         - blinking task
 *          - twr polling task
 *         - rx task
 * Only setup, do not start.
 * */
static void node_setup_tasks(void)
{
#if 0 // TBD
    osThreadDef(imuTask, ImuTask, osPriorityBelowNormal, 0, 256);
    app.imuTask.Handle = osThreadCreate(osThread(imuTask), NULL);
#endif

    osThreadDef(calcTask, CalcTask, osPriorityNormal, 0, 1024);
    app.calcTask.Handle = osThreadCreate(osThread(calcTask), NULL);

    /* rxTask is receive the signal from
     * passing signal from RX IRQ to an actual two-way ranging algorithm.
     * It awaiting of an Rx Signal from RX IRQ ISR and decides what to do next in TWR exchange process
     * */
    osThreadDef(rxTask, RxTask, osPriorityRealtime, 0, 1024);
    app.rxTask.Handle = osThreadCreate(osThread(rxTask), NULL);

    if  ((app.rxTask.Handle == NULL) ||
        (app.calcTask.Handle == NULL) )
    {
        error_handler(1, _Err_Create_Task_Bad);
    }	
}

/* @brief Terminate all tasks and timers related to Node functionality, if any
 *        DW1000's RX and IRQ shall be switched off before task termination,
 *        that IRQ will not produce unexpected Signal
 * */
void node_terminate_tasks(void)
{
    //HAL_RTCEx_DeactivateWakeUpTimer(&hrtc); // TBD

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
            error_handler(1, _ERR_Cannot_Delete_rxTask);
        }
        taskEXIT_CRITICAL();
    }

    if(app.calcTask.Handle)
    {
        osDelay(1000 / portTICK_PERIOD_MS); //wait 1s thus the calcTask should receive all mail sent

        osMutexWait(app.calcTask.MutexId, osWaitForever);
        taskENTER_CRITICAL();
        osMutexRelease(app.calcTask.MutexId);
        if(osThreadTerminate(app.calcTask.Handle) == osOK)
        {
            osMutexDelete(app.calcTask.MutexId);
            app.calcTask.Handle = NULL;
        }
        else
        {
            error_handler(1, _ERR_Cannot_Delete_calcTask);
        }
        taskEXIT_CRITICAL();
    }

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
            error_handler(1, _ERR_Cannot_Delete_imuTask);
        }
        taskEXIT_CRITICAL();
    }

    node_process_terminate();
}


/* @fn         node_helper
 * @brief      this is a service function which starts the
 *             TWR Node functionality
 *             Note: the previous instance of TWR shall be killed
 *             with node_terminate_tasks();
 *
 * */
void node_helper(void const *argument)
{
    error_e   tmp;

    dw_name_e master_chip = (dw_name_e)(*(int*)argument);

    port_init_dw_chips();

    port_disable_wake_init_dw();

    initDList();    /**< The List of Discovered Tags during listening of the air */

    /* "RTOS-independent" part : initialization of two-way ranging process */
    tmp = node_process_init(master_chip);    /* will initialize TwrInfo */

    if(tmp != _NO_ERR)
    {
        error_handler(1, tmp);
    }

    node_setup_tasks();     /**< "RTOS-based" : setup (not start) all necessary tasks for the Node operation. */

    node_process_start();   /**< IRQ is enabled from MASTER chip and it may receive UWB immediately after this point */
    nrf_gpio_pin_write(LED_NODE, 0);   /* Indicate the Node top-level application is started */
//    printf("f=%s,l=%d  id=0x%x app.mode=%d\r\n",__FUNCTION__,__LINE__,dwt_readdevid(),app.mode);
//	printf("%s-%d 0x%x\r\n", __FUNCTION__, __LINE__, dwt_readdevid());

//	add_tag_to_knownTagList(0x082261440D73120F, 0x120F);
}