/*
 * @file    node.c
 * @brief    DecaWave Application level
 *           collection of TWR bare-metal functions for a Node
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
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


/* Includes */
#include <node.h>

#include <util.h>

#include <math.h>
#include "errno.h"
#include <circ_buf.h>
#include <deca_regs.h>
#include <assert.h>
#include <math.h>

#include <data_filter.h>

// ----------------------------------------------------------------------------
// implementation-specific: critical section protection
#define TWR_ENTER_CRITICAL()    taskENTER_CRITICAL()
#define TWR_EXIT_CRITICAL()     taskEXIT_CRITICAL()

// ----------------------------------------------------------------------------
#ifndef M_PI
#define M_PI    (3.141592654f)
#endif

#ifndef M_PI_2
#define M_PI_2  (1.570796327f)
#endif

#ifndef TWO_PI
#define TWO_PI  (2*M_PI)
#endif

#define L_M    (SPEED_OF_LIGHT/6.5e9f)  /* Lambda, m */
#define D_M    (0.0208f)                /* Distance between centers of antennas, ~(L_M/2), m */

extern float dwt_getrangebias(uint8 chan, float range, uint8 prf);
//-----------------------------------------------------------------------------
/* TX power and PG delay configuration structure */
static const struct
{
    uint8_t        PGdelay;

    //TX POWER
    //31:24     BOOST_0.125ms_PWR
    //23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
    //15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
    //7:0       DEFAULT_PWR-TX_DATA_PWR
    uint32_t    txPwr[2]; //
}txSpectrumConfig
=
{//Channel 5
    0xc0,   //PG_DELAY
    {
        0x0E082848, //16M prf power
        0x25456585 //64M prf power
    }
};

//-----------------------------------------------------------------------------
// This static TWR structure holds all Node's process parameters
// for safety reasons it allocated statically
static twr_info_t    TwrInfo;

static int rtcInitState = 0;
static uint32_t gRTC_SF_PERIOD = 3276;

//-----------------------------------------------------------------------------
/* Note : for slot period less than 10 ms, more sofisticated TagHW shall be used.
 * STM32L1x & STM32F3x provide RTC with resolution of 61.035us, thus
 * current project can support precise timings.
 * See "default_config.h"
 **/

#define RX_RELAX_TIMEOUT_SY        (50)    /**< relaxed RX Timeout in sequential TWR process exchange */
#define SAFE_TXDATA                (1)     /**< see start_tx() */

void rtcWakeUpTimerEventCallback(nrf_drv_rtc_int_type_t int_type);

//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Implementation

/*
 * @brief     get pointer to the twrInfo structure
 * */
twr_info_t * getTwrInfoPtr(void)
{
    return (&TwrInfo);
}



/**
 * @brief     ISR level (need to be protected if called from APP level)
 *     read full diagnostic data form the received frame from the two DW1000s
 *
 * */
static int
read_full_diagnostics(rx_pckt_t *prxPckt,
                      uint8_t   chip,
                      uint32    status )
{
    assert( (chip==DW_MASTER || chip==DW_SLAVE) );

    uint16_t     fpIndex;
    diag_v5_t    *p = &prxPckt->diagnostics[(chip==DW_MASTER)?(0):(1)];

    p->header = DWT_DIAGNOSTIC_LOG_REV_5;

    memcpy(p->r0F, (uint8_t*) &status, 4);                        //copy 4bytes of status (saved on entry to ISR)
    dwt_readfromdevice(RX_FINFO_ID, 4, 5, (uint8_t*)(p+5) );    //read MSB from status and 4byte frame info
    dwt_readfromdevice(RX_FQUAL_ID, 0, 17,(uint8_t *)(p->r12)); //read 17 bytes of diagnostic data from 0x12,13,14

    if(chip == DW_MASTER)
    {
        TS2TS_MEMCPY(p->r15, prxPckt->timeStamp_Master);    //copy TS MASTER
    }
    else
    {
        TS2TS_MEMCPY(p->r15, prxPckt->timeStamp_Slave);    //copy TS SLAVE
    }

    dwt_readfromdevice(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET, 9, (uint8_t *)(p->r15 + 5)); //2FP, 2Diag, 5TSraw

    // Calculate the First Path Index ((LDE0 + LDE1 << 8) / 64)
    fpIndex = (*((uint8_t*)(p+32)) >> 6) + (*((uint8_t*)(p+33)) << 2);

    fpIndex = fpIndex*4 + 1;                 //get location in the accumulator

    //printf("%d FP index %02x %02x %i %i\n", offset, *((uint8_t*)(p+32)), *((uint8_t*)(p+33)), fpIndex, (fpIndex-1)>>2);
    //Read CIR for the First Path + 3 More samples (4*4 = 16)
    dwt_readaccdata(p->r25, 17, fpIndex-1); //read 1 extra as first will be dummy byte
    dwt_readfromdevice(LDE_IF_ID,     LDE_PPINDX_OFFSET,    2, p->r2E);
    dwt_readfromdevice(DRX_CONF_ID, 0x28,                   4, p->r27);
    dwt_readfromdevice(LDE_IF_ID,     LDE_PPAMPL_OFFSET,    2, p->r2E2);

    return (int) fpIndex ;
}


/**
 * @brief   ISR level (need to be protected if called from APP level)
 *          Transmit packet
 * */
static error_e
tx_start(tx_pckt_t * pTxPckt)
{
    error_e ret = _NO_ERR;
    uint8_t  txFlag = 0;

    set_SPI_master();
    dwt_forcetrxoff();    //Stop the Receiver and Write Control and Data

    dwt_writetxdata(pTxPckt->psduLen, (uint8_t *) &pTxPckt->msg.stdMsg, 0);

    dwt_writetxfctrl(pTxPckt->psduLen, 0, 1);

    //Setup for delayed Transmit
    if(pTxPckt->delayedTxTimeH_sy != 0UL)
    {
        dwt_setdelayedtrxtime(pTxPckt->delayedTxTimeH_sy) ;
    }

    if((pTxPckt->delayedRxTime_sy !=0) && (pTxPckt->txFlag & DWT_RESPONSE_EXPECTED))
    {
        dwt_setrxaftertxdelay(pTxPckt->delayedRxTime_sy);
        dwt_setrxtimeout(pTxPckt->delayedRxTimeout_sy);
    }

    // Begin delayed TX of frame
    txFlag = (pTxPckt->delayedTxTimeH_sy != 0UL) | (pTxPckt->txFlag);

    if(dwt_starttx(txFlag) != DWT_SUCCESS)
    {
        ret = _Err_DelayedTX_Late;
    }

    return (ret);
}

//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------

/**
 *  @brief function to convert the ToF to range/distance to be output over UART/USB.
 *  if p->s.rbcEn(RANGE_BIAS_CORRECTION) is used to correct the range (then the output value will contain corrected range)
 *  otherwise the it will contain raw range
 *
 *  @param [in]
 *  *p  : pointer to run-time param_block_t;
 *  tofi: time of flight used to calculate the output distance;
 *  @param [out]
 *  *pdist_cm : pointer to output distance result. This also corrected with p->s.rngOffset_mm offset value.
 *              (float)(0XDEADBEEF) if error
 *
 */
error_e
tof2range(param_block_t *p,
          float         *r_m,
          int32_t       tofi)
{
    error_e      ret    = _NO_ERR;
    dwt_config_t *pCfg  = &p->dwt_config;
    float        dist_to_correct;

    *r_m  = (float)tofi* (SPEED_OF_LIGHT/499.2e6f/128.0f);

    /* Apply Range bias correction */
    if(p->s.rbcEn == 1)
    {
        dist_to_correct = *r_m;

        //for the 6.81Mb data rate we assume gating gain of 6dB is used,
        //thus a different range bias needs to be applied
        //if(inst->configData.dataRate == DWT_BR_6M8)
        if(app.pConfig->s.smartTxEn == 1)
        {
            //1.51 for channel 5
            if(pCfg->chan == 5)
            {
                dist_to_correct /= 1.51f;
            }
            else if(pCfg->chan == 2)
            {
                dist_to_correct /= 1.31f;
            }
        }

        *r_m = *r_m - dwt_getrangebias(pCfg->chan,
                                     dist_to_correct,
                                     pCfg->prf);
    }

    if(*r_m >2000.0f)
    {
        ret =  _Err_Range_Calculation;
    }
    else
    {
        *r_m = (*r_m - ((float)p->s.rngOffset_mm/1000.0f) );
    }

    return ret;
}

void
pdoa2XY(result_t *pRes, float zeroOffset)
{
    float   phase_m;/* phase in [-L/2..L/2], m */
    float   alfa;   /* phase_m/D_M [-1..1]*/
    float   pM, r_m;

    r_m = pRes->dist_cm / 100;

    /* pdoa_raw_deg & zeroOffset are both guaranteed E [-180..180] */
    /* [-180..180] [-180..180] => pM E [-360..360] */
    pM  = pRes->pdoa_raw_deg;
    pM -= zeroOffset;

    if(pM < -180.0f)
    {
        pM += 360.0f;
    }

    if(pM > 180.0f)
    {
        pM -= 360.0f;
    }

    /* pM E [-180..180] */

    /* Calculate new x_cm & y_cm from PDOA and Range information */

    phase_m = pM * (L_M/360.0f); /* Path difference in [-L/2..L/2], m */

    /* Below is a polynomial which can be used to correct Node's antenna characteristics */
    if(app.pConfig->s.phaseCorrEn)
    {
        float coef[] = {-14205, 419, 4.59, 0.8361, 0};

        /* Calculation of polynomial using Horner's method. This is O(O) */
        float x = coef[0];
		
        for(int i = 1; i < 5 ; i++)
        {
            x = (x*phase_m) + coef[i];
        }

        phase_m = x;
    }

    alfa   = (phase_m / D_M);

    (alfa > 1.0f)?(alfa = 1.0f):((alfa < -1.0f)?(alfa = -1.0f):(alfa)); /* Ensure alfa in [-1..1] */

//    double temp_Xcm, temp_Ycm;
//	temp_Xcm = (D_M/2 + alfa *(r_m - phase_m/2)) * 100.0f;
//	temp_Ycm = (r_m - phase_m/2) * sqrt(1- (alfa*alfa)) * 100.0f;
//	motionFilter(&temp_Xcm, &temp_Ycm);
//
//	pRes->x_cm = temp_Xcm;
//	pRes->y_cm = temp_Ycm;
    pRes->x_cm = (D_M/2 + alfa *(r_m - phase_m/2)) * 100.0f;
    pRes->y_cm = (r_m - phase_m/2) * sqrt(1- (alfa*alfa)) * 100.0f;
}

/*
 * @brief   Calculates phase differences using giving data
 *          from MASTER and SLAVE chips
 *
 * @attention
 *          (C) Decawave 2017
 *
 * @return  pdoa_raw: phase difference in RAD
 * */
float calcPD(pdoa_info_t *pA)
{
    float pd = 0;
    int16 cmplex_m[2];
    int16 cmplex_s[2];

    uint8_t sfdAngleM   = pA->sfdangle_master;
    uint8_t *pAccM      =  &pA->acc_master[1];
    uint8_t sfdAngleS   = pA->sfdangle_slave;
    uint8_t *pAccS      = &pA->acc_slave[1];

    float sfdanglem, sfdangles ;
    float acc_angle_master, acc_angle_slave ;

    // Master
    sfdanglem = (((float)(sfdAngleM & 0x7F))/64.0f)*M_PI;
    // Get the complex number from accumulator samples
    // memcpy(&cmplex_m[0], &acc_master[1], 4);
    cmplex_m[0] = (pAccM[1]<<8) | pAccM[0];
    cmplex_m[1] = (pAccM[3]<<8) | pAccM[2];

    // Slave
    sfdangles = (((float)(sfdAngleS & 0x7F))/64.0f)*M_PI;
    // Get the complex number from accumulator samples
    // memcpy(&cmplex_s[0], &acc_slave[1], 4);
    cmplex_s[0] = (pAccS[1]<<8) | pAccS[0];
    cmplex_s[1] = (pAccS[3]<<8) | pAccS[2];

    // Calculate phase difference
    // atan2 - -pi to pi
    acc_angle_master = atan2f((float)cmplex_m[1], (float)cmplex_m[0]) ;
    acc_angle_slave = atan2f((float)cmplex_s[1], (float)cmplex_s[0]) ;

    // phase = sfdangle - accangle
    // acc angle = real + imag @ floor (fp index)

    pd = ((acc_angle_master - sfdanglem)
         - (acc_angle_slave - sfdangles));

    pd += M_PI;

    /* pdoa_true = mod((pdoa - pdoa_offset)+pi, 2pi) - pi; */
    while (pd >= (TWO_PI))
    {
        pd -= TWO_PI;
    }

    while (pd < 0)
    {
        pd += TWO_PI;
    }

    pd -= M_PI;

    pd *= (180 / M_PI);

    return pd;
}



/*
 * @brief     ISR level (need to be protected if called from APP level)
 *             low-level configuration for DW1000
 *
 *             if called from app, shall be performed with DW IRQ off &
 *             TWR_ENTER_CRITICAL(); / TWR_EXIT_CRITICAL();
 *
 *             The SPI for selected chip shall already be chosen
 *
 * @note
 * */
static void
rxtx_node_configure
(
    dw_name_e     chip,
    dwt_config_t *pdwCfg,
    uint16_t      frameFilter,
    uint16_t      txAntDelay,
    uint16_t      rxAntDelay,
    uint16_t      panId,
    uint16_t     shortaddr
)
{
    assert(pdwCfg->chan == 5);          /**< This project supports only CH5 */
    assert(chip == DW_MASTER || chip == DW_SLAVE);

    dwt_txconfig_t  dtmp;
    uint32_t        power;

    set_dw_spi_fast_rate(chip);         /**/

    dwt_configure(pdwCfg);    /**< Configure the Physical Channel parameters (PLEN, PRF, etc) */

    /* configure power */
    power = txSpectrumConfig.txPwr[pdwCfg->prf - DWT_PRF_16M];

    if(app.pConfig->s.smartTxEn == 1)
    {
        dwt_setsmarttxpower(1);
    }
    else
    {
        power = (power & 0xff) ;
        power |= (power << 8) + (power << 16) + (power << 24);
		power = 0x851f1f85;
        dwt_setsmarttxpower(0);
    }

    dtmp.power = power;
	
    dtmp.PGdly = txSpectrumConfig.PGdelay ;

    dwt_configuretxrf(&dtmp);

    /* set antenna delays */
    dwt_settxantennadelay(txAntDelay);
    dwt_setrxantennadelay(rxAntDelay);

    dwt_setdblrxbuffmode (0);       /**< dblBuf is not used in TWR */
    dwt_setrxaftertxdelay(0);       /**< no any delays set by default : part of config of receiver on Tx sending */
    dwt_setrxtimeout     (0);       /**< no any delays set by default : part of config of receiver on Tx sending */
    dwt_enableframefilter(frameFilter);
    dwt_setpanid(panId);
    dwt_setaddress16(shortaddr);

    /*patch for preamble length 64 */
    if(pdwCfg->txPreambLength == DWT_PLEN_64)
    {
        set_dw_spi_slow_rate(DW_MASTER);
        dwt_loadopsettabfromotp(DWT_OPSET_64LEN);
        set_dw_spi_fast_rate(DW_MASTER);
    }

    dwt_setleds(3) ;                                    /**< DEBUG I/O 2&3 : configure the GPIOs which control the LEDs on HW */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);   /**< DEBUG I/O 5&6 : configure TX/RX states to output on GPIOs */
}


/*
 * @brief   This is a special function, which starts the SAR sampling.
 *          This shall be started in order to update temperature of the chip.
 *      Note: the reading of temperature will be available ~1mS after start of this function.
 * */
static void
start_tempvbat_sar(void)
{
    uint8_t wr_buf[1];
    // These writes should be single writes and in sequence
    wr_buf[0] = 0x80; // Enable TLD Bias
    dwt_writetodevice(RF_CONF_ID,0x11,1,wr_buf);

    wr_buf[0] = 0x0A; // Enable TLD Bias and ADC Bias
    dwt_writetodevice(RF_CONF_ID,0x12,1,wr_buf);

    wr_buf[0] = 0x0f; // Enable Outputs (only after Biases are up and running)
    dwt_writetodevice(RF_CONF_ID,0x12,1,wr_buf);    //

    // Reading All SAR inputs
    wr_buf[0] = 0x00;
    dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C,1,wr_buf);
    wr_buf[0] = 0x01; // Set SAR enable
    dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C,1,wr_buf);
}

/**
 * @brief  ISR level (need to be protected if called from APP level)
 *         Setup MASTER's and SLAVE's chip receivers to receive at expected Final Tx Time + rx_timeout
 *
 *         This function is called from the (TX) interrupt handler.
 *         The Node can range to one tag at a time only.
 */
static void
resp_tx_cb_setup_receivers(twr_info_t *pTwrInfo)
{
//------------------------------------------------------------------------------
//
// Description of the ranging protocol.
//
//           |< ------------------------------- tag_pollTxFinalTx_us --------------------------------->|
//TAGTX      |                                                                                         |
//   PollTxSFD   PollTxEnd                                                                    FinalTxSFD   FinalTxEnd
//_i---------|---i___________________________________________________________________________i---------|---i___
//           |
//           |   |<--- tag_replyDly_us --->|                                                 |
//TAGRX         |                   RespRxStart                                                 |
//_______________________________________..i---------------i___________________________________________________
//
//           |                             |                                                 |
//NODERX     |                             |                                                 |
//CHIP MASTER|                             |           |<-(2)- dwt_setdelayedtrxtime for M ->|
//           |                             |           |                                     |
//    PollRxTs                             |           |                                      FinalRxSFD
//-----------|---i___________________________________________________________________________i---------|---i___
//           |
//           |<--[1]-- delayedTxTimeH_sy (tmp64) ----->|
//           |                                         |
//NODETX                                        respTxTs
//_________________________________________i-----------|---i___________________________________________________
//
//           |                                         |
//NODERX     |                                         |
//CHIP SLAVE's DEVTIME TIME is synchronized and close to MASTER's dev_time. use Master's dev_time to activate SLAVE's receiver
//the differences between M&S dev_time is << delayed_RX time, so we should never experience fail in starting of M/S receivers.
//           |                                         |<-[2]- dwt_setdelayedtrxtime for B ->|
//    PollRxTs                                  RespTxTs                                     |
//           |                                         |                                      FinalRxSFD
//___________________________________________________________________________________________i---------|---i___
//
//[1] is calculating in send_response()
//[2] is calculating in TX_IRQ callback
//------------------------------------------------------------------------------

    //[2] startup receivers.
    uint32_t    tmp;
    uint16_t    timeout;
    uint64_t    anchorRespTxTime;

    TS2U64_MEMCPY(anchorRespTxTime, pTwrInfo->nodeRespTx_ts);

    tmp  = pTwrInfo->pSfConfig->tag_pollTxFinalTx_us;   //configured time when Tag will transmit Final

    tmp -= pTwrInfo->pSfConfig->tag_replyDly_us;        //configured time when Node was transmitting Response (now)
    tmp -= pTwrInfo->msg_time .poll     .phrAndData_us; //length of data part of the poll
    tmp -= pTwrInfo->msg_time .response .preamble_us;   //length of response's preamble
    tmp -= pTwrInfo->msg_time .final    .preamble_us;   //length of final's preamble

    tmp = (uint32_t) (util_us_to_dev_time(tmp) >>8);

    tmp += (uint32_t)(anchorRespTxTime >>8);
    tmp &= 0xFFFFFFFE;                                              //This is the time when to enable the receiver

    timeout = pTwrInfo->msg_time .final .sy + RX_RELAX_TIMEOUT_SY;  //timeout for reception of Final msg

    set_SPI_slave();
    dwt_setdelayedtrxtime(tmp);         //set delayed RX for Slave : waiting the Final
    dwt_setrxtimeout(timeout);
    dwt_rxenable(DWT_START_RX_DELAYED); //start delayed Rx : if late, then the RX will be enabled immediately
    start_tempvbat_sar();               //start sampling of a temperature on the Slave chip: on reception of Final read the value.

    set_SPI_master();
    dwt_setdelayedtrxtime(tmp);         //set delayed RX for Master : waiting the Final
    dwt_setrxtimeout(timeout);
    dwt_rxenable(DWT_START_RX_DELAYED); //start delayed Rx : if late, then the RX will be enabled immediately.
    start_tempvbat_sar();               //start sampling of a temperature on the Master chip: on reception of Final read the value.
}


//-----------------------------------------------------------------------------
//    DW1000 callbacks section :
//    if RTOS, the preemption priority of the dwt_isr() shall be such, that
//    allows signal to the thread.

/* @brief   ISR level
 *          Real-time TWR application Tx callback
 *          to be called from dwt_isr()
 * */
void twr_tx_node_cb(const dwt_cb_data_t *txd)
{
    twr_info_t     *pTwrInfo = getTwrInfoPtr();

    if(!pTwrInfo)
    {
        return;
    }

    uint32_t tmp = nrf_drv_rtc_counter_get(&rtc);

    // Store the Tx Time Stamp of the transmitted packet
    switch(pTwrInfo->txState)
    {
    case Twr_Tx_Range_Config_Sent :                //responder (node)
        pTwrInfo->rangeInitRtcTimeStamp = tmp;
        dwt_readtxtimestamp(pTwrInfo->rangeInitTx_ts);
        break;

    case Twr_Tx_Resp_Sent :                        //responder (node)
        pTwrInfo->respRtcTimeStamp = tmp;
        dwt_readtxtimestamp(pTwrInfo->nodeRespTx_ts);

        resp_tx_cb_setup_receivers(pTwrInfo);    //node additional algorithm for receivers
        break;

    default :
        break;
    }
}

/* @brief   ISR level
 *          TWR application Rx callback
 *          to be called from dwt_isr() as an Rx call-back
 * */
void twr_rx_node_cb(const dwt_cb_data_t *rxd)
{
    twr_info_t  *pTwrInfo = getTwrInfoPtr();

    const int   size = sizeof(pTwrInfo->rxPcktBuf.buf) / sizeof(pTwrInfo->rxPcktBuf.buf[0]);

    int head = pTwrInfo->rxPcktBuf.head;
    int tail = pTwrInfo->rxPcktBuf.tail;

    if(CIRC_SPACE(head,tail,size) <= 0)
    {
        return;    //no space in the fast intermediate circular buffer
    }

    rx_pckt_t *p = &pTwrInfo->rxPcktBuf.buf[head];

    p->rtcTimeStamp = nrf_drv_rtc_counter_get(&rtc);    // MCU RTC timestamp 

    //SLAVE
    set_SPI_slave();
    dwt_readrxtimestamp(p->timeStamp_Slave);            //Raw Rx TimeStamp of the SLAVE chip
    p->status_Slave = dwt_read32bitreg(SYS_STATUS_ID);  //save status of the SLAVE chip
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD |
                                     SYS_STATUS_ALL_RX_TO   |
                                     SYS_STATUS_ALL_RX_ERR );


    //MASTER
    set_SPI_master();
    dwt_readrxtimestamp(p->timeStamp_Master);      //Raw Rx TimeStamp of the MASTER chip
    p->status_Master = rxd->status;                //save status of the MASTER chip

    p->rxDataLen = MIN(rxd->datalength, sizeof(p->msg));

    dwt_readrxdata((uint8_t *)&p->msg, p->rxDataLen, 0); //Raw message

    if(app.rxTask.Handle)         // RTOS : rxTask can be not started yet
    {
        head = (head + 1) & (size-1);
        pTwrInfo->rxPcktBuf.head = head;    // ISR level : do not need to protect

        //Sends the Signal to the application level via OS kernel.
        //This will add a small delay of few us, but
        //this method make sense from a program structure point of view.
        if(osSignalSet(app.rxTask.Handle, app.rxTask.Signal) != osOK)
        {
            error_handler(1, _Err_Signal_Bad);
        }
    }
}

void twr_rx_timeout_cb(const dwt_cb_data_t *rxd)
{
  set_SPI_master();
  
  dwt_setrxtimeout(0);
  dwt_rxenable(0);

//  printf("Func: %s, Line: %d\r\n", __FUNCTION__, __LINE__);
}

void twr_rx_error_cb(const dwt_cb_data_t *rxd)
{
    twr_rx_timeout_cb(rxd);
}

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

/**
 * @brief Bare-metal level
 * function used to prime the DW1000 for the external clock sync
 * configure the mode as OSTR, OSTS or OFF
 * configure the wait time for OSTR mode
 *
 */
static void
device_set_sync(sync_mode_e mode, uint8 wait)
{
    decaIrqStatus_t  stat ;
    stat = decamutexon() ;
    uint16 reg;
//    spim_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
//    p_cb->state = NRFX_DRV_STATE_INITIALIZED;

    switch(mode)
    {
        case Sync_Mode_Ostr:
        {
            reg = dwt_read16bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET);

            reg &= EC_CTRL_WAIT_MASK; //clear timer value, clear OSTRM

            reg |= EC_CTRL_OSTRM;

            reg |= ((((uint16) wait) & 0xff) << 3); //set new timer value

            dwt_write16bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET, reg);
            break;
        }

        case Sync_Mode_Off:
        {
            reg = dwt_read16bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET);

            reg &= EC_CTRL_WAIT_MASK ; //clear timer value, clear OSTRM

            dwt_write16bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET, reg);
            break;
        }

        default:
        case Sync_Mode_Osts:
            break;
    }

    decamutexoff(stat);
}


/**
 * @brief Bare-metal
 *           Used to synchronise the system clocks of two DW1000 ICs
 *
 */
void synchronize_DW1000clocks(void)
{
    set_SPI_slave();
    //set up OSTS wait and OSTR mode for SLAVE chip
    device_set_sync(Sync_Mode_Ostr, 33);

    set_SPI_master();
    //set up OSTS wait and OSTR mode for MASTER chip
    device_set_sync(Sync_Mode_Ostr, 33);

    //then sync the clocks
    port_set_syncenable(1);
    port_set_syncclear(1);

    //this gives about 2.7 MHz pulse
    port_set_sync(1);
    port_set_sync(0);

    //then turn off mode and sync
    port_set_syncenable(0);
    //port_set_syncclear(0);                //TODO: should this stays disabled?

    set_SPI_slave();
    device_set_sync(Sync_Mode_Off, 0); //clear for SLAVE

    set_SPI_master();
    device_set_sync(Sync_Mode_Off, 0); //clear for MASTER
}

//-----------------------------------------------------------------------------
// The most real-time section of TWR algorithm

/* return us - correction of time for a given Tag wrt uTimeStamp
 * */
static int32
calc_slot_correction(twr_info_t         *pTwr,
                     uint16_t       slot,
                     uint32_t           uTimeStamp)

{
    int    tmp;

    tmp = uTimeStamp - pTwr->gRtcSFrameZeroCnt;

    if (tmp < 0)
    {
        tmp += RTC_WKUP_CNT_OVFLW;    // RTC Timer overflow - 24 bit counter
    }

    // (tmp*(30.517) -  RTC Timer - Each Tick is configured as 30.517 us
    // (1e3f * slot * pTwr->pSfConfig->slotPeriod) - Slot period in terms of us
    tmp = (int)((tmp * WKUP_RESOLUTION_NS) - (1e6f * slot * pTwr->pSfConfig->slotPeriod));
    tmp /= 1e3f;

    return (tmp); //tagSleepCorrection_us
}

/**
 * @brief this function constructs the Ranging Config message,
 *        not including mac header
 *
 */
static void
prepare_ranging_config_msg(rng_cfg_t        *pRcfg,
                           tag_addr_slot_t  *tag,
                           twr_info_t       *p,
                           uint32           uTimeStamp)
{

    pRcfg->fCode = Twr_Fcode_Rng_Config;          //function code (specifies if message is a rangeInit, poll, response, etc)

    pRcfg->tagAddr[0] = tag->addrShort[0];        //tag short address to be used in TWR
    pRcfg->tagAddr[1] = tag->addrShort[1];

    pRcfg->version    = RC_VERSION_PDOA;

    {   //slot period correction for the Tag wrt to
        int32_t tagSleepCorrection_us = calc_slot_correction(p, tag->slot, uTimeStamp);
        U32TOAR_MEMCPY(pRcfg->slotCorr_us , tagSleepCorrection_us);
    }

    pRcfg->sframePeriod_ms[0] = p->pSfConfig->sfPeriod_ms       &0xff;
    pRcfg->sframePeriod_ms[1] = p->pSfConfig->sfPeriod_ms>>8    &0xff;

    pRcfg->pollTxToFinalTx_us[0] = p->pSfConfig->tag_pollTxFinalTx_us       &0xff;
    pRcfg->pollTxToFinalTx_us[1] = p->pSfConfig->tag_pollTxFinalTx_us>>8    &0xff;

    pRcfg->delayRx_us[0] = p->pSfConfig->tag_replyDly_us    &0xff;
    pRcfg->delayRx_us[1] = p->pSfConfig->tag_replyDly_us>>8 &0xff;

    pRcfg->pollMultFast[0] = tag->multFast      &0xff;  //tag config : multiplier: i.e poll every 1 periods
    pRcfg->pollMultFast[1] = tag->multFast>>8   &0xff;  //if moving

    pRcfg->pollMultSlow[0] = tag->multSlow      &0xff;  //tag config : multiplier: i.e. poll every 10 period
    pRcfg->pollMultSlow[1] = tag->multSlow>>8   &0xff;  //if stationary

    pRcfg->mode[0] = tag->mode     &0xff;      //tag config : i.e. use imu to identify stationary : bit 0;
    pRcfg->mode[1] = tag->mode>>8  &0xff;      //
	
}

/**
 * @brief   This function constructs the data part of Response Msg.
 *          It is called after node receives a Poll from a tag.
 *          Mac header for the message should be created separately.
 */
extern uint8_t BATTERY_SPEED;
static void
prepare_response_msg(resp_tag_t         *pResp,
                     tag_addr_slot_t    *tag,
                     twr_info_t         *p,
                     uint32             uTimeStamp)
{
    int32_t    tmp;
    pResp->fCode = Twr_Fcode_Resp_Ext;

    {//slot period correction for the Tag
        int32_t tagSleepCorrection_us = calc_slot_correction(p, tag->slot, uTimeStamp);

        U32TOAR_MEMCPY(pResp->slotCorr_us , tagSleepCorrection_us);
    }

    //pResp->rNum = p->result[tag->slot].rangeNum;
    pResp->rNum = BATTERY_SPEED;

    /* Send back to the tag the previous X, Y & clockOffset */
    tmp = (int32_t)(p->result[tag->slot].x_cm);
    pResp->x_cm[0] = (uint8_t)(tmp &0xFF);
    pResp->x_cm[1] = (uint8_t)(tmp>>8 &0xFF);

    tmp = (int32_t)(p->result[tag->slot].y_cm);
    pResp->y_cm[0] = (uint8_t)(tmp &0xFF);
    pResp->y_cm[1] = (uint8_t)(tmp>>8 &0xFF);

    tmp = (int32_t)(p->result[tag->slot].clockOffset_pphm);
    pResp->clkOffset_pphm[0] = (uint8_t)(tmp &0xFF);
    pResp->clkOffset_pphm[1] = (uint8_t)(tmp>>8 &0xFF);

}


/* @brief    APP level
 *             part of Real-time TWR algorithm implementation (Responder)
 *
 *             if called from ISR level, then revise/remove
 *             TWR_ENTER_CRITICAL()
 *             TWR_EXIT_CRITICAL()
 *
 *            Note:
 *            Shall be called with guarantee that the DWT_IRQ will not happen
 *
 * @return     _NO_ERR for no errors / error_e otherwise
 * */
static error_e
node_send_ranging_config(rx_pckt_t  *pRxPckt,
                         twr_info_t *p)

{
    error_e         ret;
    tx_pckt_t       TxPckt;     /**< allocate TxPckt */
    tag_addr_slot_t *tag = pRxPckt->tag;

    /* tmp, tmp64 used to calculate the time when to send the delayed Ranging Config (i.e. R-Marker of RangingConfig),
     * using Tag's configuration of when Tag is activating its receiver
     * */
    uint32_t    tmp;
    uint64_t    tmp64;

    /* setup the rest of Ranging Config data */
    rng_cfg_t *pRcfg;

    if(tag->reqUpdatePending == 0)
    {
        /* When the Node revceives a blink from a Tag,
         * setup the Ranging Config MAC frame header with Long-Short addressing mode
         * */
        rng_cfg_msg_t   *pTxMsg = &TxPckt.msg.rngCfgMsg;

        /* Construct TX Ranging Config UWB message (LS) */
        TxPckt.psduLen      = sizeof(rng_cfg_msg_t);

        /* See IEEE frame header description */
        pTxMsg->mac.frameCtrl[0]    = Head_Msg_STD;
        pTxMsg->mac.frameCtrl[1]    = Frame_Ctrl_LS;       /**<long address */
        pTxMsg->mac.panID[0]        = p->panID     &0xff;
        pTxMsg->mac.panID[1]        = p->panID>>8  &0xff;
        memcpy(pTxMsg->mac.destAddr, &tag->addr64, sizeof(pTxMsg->mac.destAddr)); /**< tag's address */
        pTxMsg->mac.sourceAddr[0]   = p->euiShort[0];     /**< node short address to be used by the tag in TWR */
        pTxMsg->mac.sourceAddr[1]   = p->euiShort[1];
        pTxMsg->mac.seqNum          = p->seqNum;

        /* rest of Ranging Config */
        pRcfg               = &pTxMsg->rngCfg;

        //rcDelay_us is a SYSTEM configuration value of the delay
        //after completion of a Tag's Blink Tx, when the Tag will turn on its receiver
        //and will wait for the Ranging Config Response from the Node.
        //From Node's view this is a delay between end of reception of Blink's data
        //and start of transmission of preamble of Ranging Config.
        tmp  = app.pConfig->s.rcDelay_us;

        /* Adjust the transmission time with respect to the last System TimeStamp, i.e. Timestamp of Blink */
        tmp += p->msg_time.blink.phrAndData_us;          //pre-calculated length of Blink packet (data)
        tmp += p->msg_time.ranging_config.preamble_us;   //pre-calculated length of preamble length of Ranging Config
        TS2U64_MEMCPY(tmp64, pRxPckt->timeStamp_Master);        //Blink's timestamp

        tmp64 += util_us_to_dev_time(tmp);
        tmp64 &= MASK_TXDTS;
    }
    else
    {
        tag->reqUpdatePending = 0;

        /* When the Node receives a poll and need to update the Tag, it sends Ranging Config instead of response
         * setup the Ranging Config MAC frame header with Short-Short addressing mode
         * */
        rng_cfg_upd_msg_t   *pTxMsg = &TxPckt.msg.rngCfgUpdMsg;

        /* Construct TX Ranging Config Update UWB message */
        TxPckt.psduLen  = sizeof(rng_cfg_upd_msg_t);

        /* See IEEE frame header description */
        pTxMsg->mac.frameCtrl[0]    = Head_Msg_STD;
        pTxMsg->mac.frameCtrl[1]    = Frame_Ctrl_SS;
        pTxMsg->mac.panID[0]        = p->panID & 0xff;
        pTxMsg->mac.panID[1]        = (p->panID >> 8) & 0xff;
        pTxMsg->mac.destAddr[0]     = tag->addrShort[0];
        pTxMsg->mac.destAddr[1]     = tag->addrShort[1];
        pTxMsg->mac.sourceAddr[0]   = p->euiShort[0];
        pTxMsg->mac.sourceAddr[1]   = p->euiShort[1];
        pTxMsg->mac.seqNum          = p->seqNum;

        /* rest of Ranging Config */
        pRcfg           = &pTxMsg->rngCfg;

        //tag_replyDly_us is a SYSTEM configuration value of the delay
        //after completion of a Tag's Poll Tx, when the Tag will turn on its receiver
        //and will wait for the Response from the Node.
        //From Node's view this is a delay between end of reception of Poll's data
        //and start of transmission of preamble of Reply (Ranging Config).
        tmp  = app.pConfig->s.sfConfig.tag_replyDly_us;

        /* Adjust the transmission time with respect to the last System TimeStamp, i.e. Timestamp of Poll */
        tmp += p->msg_time.poll.phrAndData_us;          //pre-calculated length of Poll packet (data)
        tmp += p->msg_time.ranging_config.preamble_us;  //pre-calculated length of preamble length of Ranging Config
        TS2U64_MEMCPY(tmp64, pRxPckt->timeStamp_Master);       //Poll's timestamp
        tmp64 += util_us_to_dev_time(tmp);
        tmp64 &= MASK_TXDTS;
    }

    /* write the rest of Ranging Config */
    prepare_ranging_config_msg(pRcfg, tag, p,  pRxPckt->rtcTimeStamp);

    TxPckt.txFlag               = ( DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    TxPckt.delayedTxTimeH_sy    = tmp64 >>8;    //at this time the Node will transmit the Ranging Config TimeStamp (R-Marker)
    TxPckt.delayedRxTime_sy     = 0;            //switch on Rx after RangingConfig transmission immediately
    TxPckt.delayedRxTimeout_sy  = 0;

    p->txState                  = Twr_Tx_Range_Config_Sent; //indicate to TX ISR that the RangeInit has been sent
    p->seqNum++;

    TWR_ENTER_CRITICAL();

    ret = tx_start(&TxPckt);

    TWR_EXIT_CRITICAL();

    if( ret != _NO_ERR)
    {
        p->lateTxCount++;
    }

//	printf("Poll sent\r\n");

    return (ret);
}


/* @brief   APP level
 *          part of Real-time TWR algorithm implementation (Responder)
 *
 *          if called from ISR level, then remove
 *          TWR_ENTER_CRITICAL() and TWR_EXIT_CRITICAL() around tx_start()
 *
 *          Note:
 *          Shall be called with guarantee that the DWT_IRQ will not happen
 *
 * @return  _NO_ERR for no errors / error_e otherwise
 * */
static error_e
node_send_response(rx_pckt_t *pRxPckt, twr_info_t *p)
{
    error_e     ret;
    uint32_t    tmp;
    uint64_t    tmp64;

    /* Construct the response tx packet to the tag */
    tx_pckt_t       TxPckt;
    resp_pdoa_msg_t  *pTxMsg = &TxPckt.msg.respMsg;

    pTxMsg->mac.frameCtrl[0]    = Head_Msg_STD;
    pTxMsg->mac.frameCtrl[1]    = Frame_Ctrl_SS;
    pTxMsg->mac.panID[0]        = p->panID & 0xff;
    pTxMsg->mac.panID[1]        = (p->panID >> 8) & 0xff;
    pTxMsg->mac.destAddr[0]     = pRxPckt->tag->addrShort[0];
    pTxMsg->mac.destAddr[1]     = pRxPckt->tag->addrShort[1];
    pTxMsg->mac.sourceAddr[0]   = p->euiShort[0];
    pTxMsg->mac.sourceAddr[1]   = p->euiShort[1];
    pTxMsg->mac.seqNum          = p->seqNum;

    prepare_response_msg(&pTxMsg->resp, pRxPckt->tag, p, pRxPckt->rtcTimeStamp);

//------------------------------------------------------------------------------
//
// Description of the ranging protocol.
//
//           |< ------------------------------- tag_pollTxFinalTx_us --------------------------------->|
//TAGTX      |                                                                                         |
//   PollTxSFD   PollTxEnd                                                                   FinalTxSFD    FinalTxEnd
//_i---------|---i___________________________________________________________________________i---------|---i___
//           |
//           |   |<--- tag_replyDly_us --->|                                                 |
//TAGRX         |                   RespRxStart                                                 |
//_______________________________________..i---------------i___________________________________________________
//
//           |                             |                                                 |
//NODERX     |                             |                                                 |
//CHIP MASTER|                             |           |<-(2)- dwt_setdelayedtrxtime for M ->|
//           |                             |           |                                     |
//    PollRxTs                             |           |                                      FinalRxSFD
//-----------|---i___________________________________________________________________________i---------|---i___
//           |
//           |<--[1]-- delayedTxTimeH_sy (tmp64) ----->|
//           |                                         |
//NODETX                                        respTxTs
//_________________________________________i-----------|---i___________________________________________________
//
//           |                                         |
//           |                                         |
//CHIP SLAVE : DEVTIME TIME is synchronized and close to MASTER's dev_time. use Master's dev_time to activate SLAVE's receiver
//the differences between M&S dev_time is << delayed_RX time, so we should never experience fail in starting of M/S receivers.
//           |                                         |<-[2]- dwt_setdelayedtrxtime for S ->|
//    PollRxTs                                  RespTxTs                                     |
//           |                                         |
//___________________________________________________________________________________________i---------|---i___
//
//[1] is calculating in node_send_response()
//[2] is calculating in TX_IRQ callback: twr_tx_node_cb()
//------------------------------------------------------------------------------

    /* [1] configure TX devtime when R-marker of Response will be transmitted */

    // DW1000 will adjust the transmission of SFD of the response packet exactly
    // to PollRx + tag_replyDly + length of PLEN of msg

    tmp  = p->pSfConfig->tag_replyDly_us;    //tag_replyDly_us is a SYSTEM configuration value of the delay
                                                    //after completion of a Tag PollTx, when the Tag will turn on its receiver
                                                    //and will wait for the Response from the Node.

    tmp += p->msg_time.poll.phrAndData_us;   //pre-calculated length of Poll packet
    tmp += p->msg_time.response.preamble_us; //pre-calculated length of Response packet


    /* Adjust the Response transmission time with respect to the last system TimeStamp */
    TS2U64_MEMCPY(tmp64, p->nodePollRx_ts);
    tmp64 += util_us_to_dev_time(tmp);
    tmp64 &= MASK_TXDTS;

    TxPckt.delayedTxTimeH_sy    = tmp64 >>8;    //at this time the Node will transmit the Response's TimeStamp
                                                //in the DWT_START_TX_DELAYED mode.

    TxPckt.psduLen              = sizeof(resp_pdoa_msg_t);
    TxPckt.txFlag               = ( DWT_START_TX_IMMEDIATE ); //RX will be set in the dwt_isr() : twr_tx_node_cb()
    TxPckt.delayedRxTime_sy     = 0;
    TxPckt.delayedRxTimeout_sy  = 0;

    p->seqNum++;
    p->txState                  = Twr_Tx_Resp_Sent;         //indicate to TX_IRQ that the Response was sent

    TWR_ENTER_CRITICAL();

    ret = tx_start(&TxPckt);

    TWR_EXIT_CRITICAL();

    //after a good tx_start(), DW_A and DW_B will be configured for reception in the TX_IRQ callback [2].
    //if the tx_start() failed transmit delayed, the DW_A receiver shall be re-enabled in the control application.

    return (ret);
}


/* @brief   ISR level
 *          read the angle data from selected chip
 * */
static void
read_phase_data(uint8_t *pSfdAngle, uint8_t *pAcc)
{
    uint16_t    fp;

    dwt_readfromdevice(0x14/*RX_TTCKO_ID*/, 4, 1, pSfdAngle); //read SFD angle (RCPHASE)
    fp = dwt_read16bitoffsetreg(0x15, 5) ;                    //read the HW FP index (10.6 integer format)

    //floor of (FP + 0.5)
    //get location (index) in the accumulator at which to calculate phase difference (x 4 as complex 16-bit numbers)
    fp = ((int) (((float) fp) * (1.0f/64.0f) + 0.5f)) * 4.0f ;
    // Read ACC data
    // Read real/imaginary at fp + 2 sample (Re (2 bytes) + Imag (2 bytes))
    dwt_readaccdata(pAcc, 5, fp); //need to read 1 extra as first will be dummy byte
}


/* @brief   APP level
 *          part of Real-time TWR algorithm implementation (Responder)
 *
 *          if called from ISR level, then remove
 *          TWR_ENTER_CRITICAL() and TWR_EXIT_CRITICAL()
 *
 *          Note:
 *          Shall be called with guarantee that the DWT_IRQ will not happen
 *
 * @return
 * */
void node_received_final(rx_pckt_t *pRxPckt, twr_info_t *pTwrInfo)
{
    /* read Final info from chips */
    TWR_ENTER_CRITICAL();

    TS2TS_MEMCPY(pRxPckt->nodePollRx_ts, pTwrInfo->nodePollRx_ts);
    TS2TS_MEMCPY(pRxPckt->nodeRespTx_ts, pTwrInfo->nodeRespTx_ts);

    { //AOA data

        //---------------------
        // read phase data for MASTER
        set_SPI_master();
        read_phase_data(&pRxPckt->pdoa_info.sfdangle_master, &pRxPckt->pdoa_info.acc_master[0]);
        // read temperature data, this should be started ~1mS before with start_tempvbat_sar() function
        dwt_readfromdevice(TX_CAL_ID, TC_SARL_SAR_LTEMP_OFFSET, 1, &pRxPckt->temperature_Master);

        //---------------------
        // read phase data for SLAVE
        set_SPI_slave();
        read_phase_data(&pRxPckt->pdoa_info.sfdangle_slave, &pRxPckt->pdoa_info.acc_slave[0]);
        // read temperature data, this should be started ~1mS before with start_tempvbat_sar() function
        dwt_readfromdevice(TX_CAL_ID, TC_SARL_SAR_LTEMP_OFFSET, 1, &pRxPckt->temperature_Slave);

        if(app.pConfig->s.diagEn == 1)
        {
            set_SPI_master();
            read_full_diagnostics(pRxPckt, DW_MASTER, pRxPckt->status_Master);
            set_SPI_slave();
            read_full_diagnostics(pRxPckt, DW_SLAVE, pRxPckt->status_Slave);
        }

        if(app.pConfig->s.accEn == 1)
        {
            memset(pRxPckt->acc, 0, sizeof(pRxPckt->acc));
            set_SPI_master();
            dwt_readaccdata(&pRxPckt->acc[0][0], sizeof(pRxPckt->acc[0]), ACC_OFFSET);
            set_SPI_slave();
            dwt_readaccdata(&pRxPckt->acc[1][0],  sizeof(pRxPckt->acc[1]), ACC_OFFSET);
        }
		
		set_SPI_master();
    }

    TWR_EXIT_CRITICAL();
}

void dwt_signalPowerDetect(double* powerFirstPath, double* receiveSignalPower)
{
	const double A = 113.77;
	uint16 F1 = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_AMPL1_OFFSET);
	uint16 F2 = ((dwt_read32bitreg(RX_FQUAL_ID)& RX_EQUAL_FP_AMPL2_MASK) >> RX_EQUAL_FP_AMPL2_SHIFT);
	uint16 F3 = dwt_read16bitoffsetreg(RX_FQUAL_ID, 4);
	uint16 C = dwt_read16bitoffsetreg(RX_FQUAL_ID, 6);
	uint16 N = (dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT;

	long double temp_firstPower = 10 * log10((pow(F1, 2) + pow(F2, 2) + pow(F3, 2)) / N*N) - A;
	long double	temp_receivePower = 10 * log10((pow(2, 17) * C) / N*N) - A;
	*powerFirstPath = temp_firstPower;
	*receiveSignalPower = temp_receivePower;
}


/* @brief    APP level
 *             Real-time TWR algorithm implementation (Responder)
 *
 *             prefer to be called from application UWB Rx thread, but
 *            can be called bare-metal from ISR, i.e. twr_rx_node_cb() directly.
 *             if called from ISR level, then revise/remove
 *             TWR_ENTER_CRITICAL()
 *             TWR_EXIT_CRITICAL()
 *
 *            Note:
 *            Shall be called with the guarantee that the DWT_IRQ will not happen
 *
 * @return     returning the result of low-level parse as error_e code:
 *             _NO_ERR             : TX sent
 *             _Err_Not_Twr_Frame    :
 *             _Err_DelayedTX_Late
 *             _Err_Not_Twr_Frame
 *             _Err_Unknown_Tag
 *             _No_Err_New_Tag
 *             _No_Err_Final
 *
 * */

double	signalPower_master[2], signalPower_slave[2];

error_e twr_responder_algorithm_rx(rx_pckt_t *pRxPckt, twr_info_t *pTwrInfo)
{
    fcode_e     fcode = Twr_Fcode_Not_Defined;
    error_e     ret   = _Err_Not_Twr_Frame;
    std_msg_t   *pMsg = &pRxPckt->msg.stdMsg;

	//printf("%s - 0x%x\r\n", __FUNCTION__, pMsg->mac.frameCtrl[0]);

    if( (pMsg->mac.frameCtrl[0] == Head_Msg_BLINK) && (pRxPckt->rxDataLen == sizeof(blink_msg_t)) )
    {
        fcode = Twr_Fcode_Blink;
    }
    else if(pMsg->mac.frameCtrl[0] == Head_Msg_STD || pMsg->mac.frameCtrl[0] == Head_Msg_STD1)
    {
        /* Apart of Blinks only SS MAC headers supported in current Node application */
        switch (pMsg->mac.frameCtrl[1] & Frame_Ctrl_MASK)
        {
        case Frame_Ctrl_SS:
            if((pRxPckt->rxDataLen == sizeof(poll_msg_t)) ||        /* Poll */
               (pRxPckt->rxDataLen == sizeof(final_msg_accel_t)))   /* Final extended */
            {
                fcode = ((std_msg_ss_t*)pMsg)->messageData[0] ;
            }
            break;
        default:
            fcode = Twr_Fcode_Not_Defined ;
            break;
        }
    }
    else
    {
        fcode = Twr_Fcode_Not_Defined ;
    }

//	printf("fcode = 0x%x\r\n", fcode);

    /* received packet with "fcode" functional code */
    switch (fcode)
    {
        case Twr_Fcode_Blink :
        {

            /* Responder (Node) received Blink message from Tag in discovery process.
             * 1. if Tag addr64 is unknown : report to upper application
             * 2. if Tag addr64 is in known Tag list, setup Timing parameters and send to him the
             *       Ranging Config message;
             * */
            uint64_t        addr64;

//            printf("Blink recvd, Line:%d\r\n", __LINE__);

            memcpy(&addr64, ((blink_msg_t*)pMsg)->tagID, sizeof(addr64));           //valid only for low endian

            pRxPckt->tag = get_tag64_from_knownTagList(addr64);

            pTwrInfo->pDestTag            = NULL;   /* New Blink: interrupt any range exchange if it was in progress */
            memset(pTwrInfo->nodePollRx_ts, 0, sizeof(pTwrInfo->nodePollRx_ts));    /* received time of poll message */
            memset(pTwrInfo->nodeRespTx_ts, 0, sizeof(pTwrInfo->nodePollRx_ts));    /* anchor's response tx time */

            if (pRxPckt->tag)
            {
                pRxPckt->tag->reqUpdatePending = 0;
                ret = node_send_ranging_config(pRxPckt, pTwrInfo);
            }
            else
            {
                pTwrInfo->newTag_addr64 = addr64;
                ret = _No_Err_New_Tag;              /* report to the upper application there a new Tag appears in the air */
            }
            break;
        }

        case  Twr_Fcode_Tag_Poll :
        {
            /* Responder (Node) received the Poll from a Tag.
             * 1. if Tag addr16 is in known Tag list, perform ranging sequence with this tag.
             * 2. otherwise ignore
             * */
            uint16_t    addr16, nodeAddr;

            addr16      = AR2U16(((poll_msg_t*)pMsg)->mac.sourceAddr);
			nodeAddr	= AR2U16(((poll_msg_t*)pMsg)->mac.destAddr);

//            printf("Poll recvd, addr=0x%04x, eui16=0x%04x, Line:%d\r\n", nodeAddr, pTwrInfo->eui16, __LINE__);

            pRxPckt->tag = get_tag16_from_knownTagList( addr16 );

            if (pRxPckt->tag && nodeAddr == pTwrInfo->eui16)
            {
                pTwrInfo->pDestTag = pRxPckt->tag;                                  /* current tag we are ranging to*/
                TS2TS_MEMCPY(pTwrInfo->nodePollRx_ts, pRxPckt->timeStamp_Master);   /* node's received time of poll message */
                pTwrInfo->pollRtcTimeStamp = pRxPckt->rtcTimeStamp;

                if(pRxPckt->tag->reqUpdatePending)
                {
                    ret = node_send_ranging_config(pRxPckt, pTwrInfo);
                }
                else
                {
                    ret = node_send_response(pRxPckt, pTwrInfo);
                }

                if(ret == _NO_ERR)
                {
                    pTwrInfo->result[pRxPckt->tag->slot].rangeNum           = pRxPckt->msg.pollMsg.poll.rNum;

                    /* Below results will be calculated in the Node on reception of Final from the Tag.
                     * Node will not report wrong values to usb/uart, but will sent them back to the Tag
                     * if the calculations was not successful */
                    pTwrInfo->result[pRxPckt->tag->slot].pdoa_raw_deg       = (float)0xDEAD;
                    pTwrInfo->result[pRxPckt->tag->slot].clockOffset_pphm   = (float)0xDEAD;
                    pTwrInfo->result[pRxPckt->tag->slot].dist_cm            = (float)0xDEAD;
                    pTwrInfo->result[pRxPckt->tag->slot].x_cm               = (float)0xDEAD;
                    pTwrInfo->result[pRxPckt->tag->slot].y_cm               = (float)0xDEAD;
                }
                else
                {
                    pTwrInfo->pDestTag            = NULL;    /* no range exchange is in progress */
                    memset(pTwrInfo->nodePollRx_ts, 0, sizeof(pTwrInfo->nodePollRx_ts));    /* received time of poll message */
                    memset(pTwrInfo->nodeRespTx_ts, 0, sizeof(pTwrInfo->nodeRespTx_ts));    /* node's response tx time */

                    pTwrInfo->lateTxCount++;
                }
            }
            else
            {
                ret = _Err_Unknown_Tag;
            }
            break;
        }

        case Twr_Fcode_Tag_Accel_Final :
        {
            /* Responder (Node) received the FINAL from Tag,
             * 1. if Tag is that one, currently the node is ranging to :
             *       read registers, necessary for ranging & phase difference and report this to upper application
             *       to calculate the result
             * 2. otherwise ignore
             * */
            uint16_t        addr16, nodeAddr;

            ret = _Err_Unknown_Tag;

            addr16 = AR2U16( ((final_msg_accel_t*)pMsg)->mac.sourceAddr );
			nodeAddr = AR2U16(((final_msg_accel_t*)pMsg)->mac.destAddr);

            pRxPckt->tag = get_tag16_from_knownTagList( addr16 );
			
//			printf("Final recvd, addr=0x%04x, eui16=0x%04x, Line:%d\r\n", nodeAddr, pTwrInfo->eui16, __LINE__);

            if (pRxPckt->tag && (pRxPckt->tag == pTwrInfo->pDestTag) && (nodeAddr == pTwrInfo->eui16))
            {
                node_received_final(pRxPckt, pTwrInfo);
                ret = _No_Err_Final;
            }
			//signal power detection
            //TWR_ENTER_CRITICAL();
            set_SPI_master();
            dwt_signalPowerDetect(&signalPower_master[1], &signalPower_master[0]);
			set_SPI_slave();
			dwt_signalPowerDetect(&signalPower_slave[1], &signalPower_slave[0]);
            //TWR_EXIT_CRITICAL();
			set_SPI_master();
            break;
        }

        default:
            /* Responder (Node) received unknown data : discard previous measurements and restart the reception
             * */
//            printf("%s-%d, 0x%x %x\r\n", __FUNCTION__, __LINE__, pMsg->mac.frameCtrl[0], pMsg->mac.frameCtrl[1]);
            pTwrInfo->pDestTag = NULL;

            ret = _Err_Not_Twr_Frame;
            break;
    }

//	if (ret == _NO_ERR)
//		printf("0x%x\r\n", fcode);

    return (ret);
}

/*
 * @brief    setup RTC wakeup timer
 * */
void twr_configure_rtc_wakeup(uint32_t     period)
{
    ret_code_t err_code;
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    uint32_t rtc_period;

    config.prescaler = RTC_WKUP_PRESCALER;   // WKUP_RESOLUTION_US counter period

    gRTC_SF_PERIOD = (period * 1e6) / WKUP_RESOLUTION_NS;

    if(rtcInitState == 0)
    {
      err_code = nrf_drv_rtc_init(&rtc, &config, rtcWakeUpTimerEventCallback);
      APP_ERROR_CHECK(err_code);
      rtcInitState = 1;
    }
    else
    {
      err_code = nrf_drv_rtc_reinit(&rtc, &config, rtcWakeUpTimerEventCallback);
      APP_ERROR_CHECK(err_code);
    }

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc, true);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
}

/* HAL RTC Wakeup timer callback.
 *         Use the LED_NODE_Pin to see synchronization of tags
 *         on the oscilloscope if needed
 * */
void rtcWakeUpTimerEventCallback(nrf_drv_rtc_int_type_t int_type)
{
    static uint32_t gRTCtickCnt = 1;

    twr_info_t     *pTwrInfo = getTwrInfoPtr();

    if(gRTCtickCnt >= gRTC_SF_PERIOD)    // 100 ms
    {
      nrf_gpio_pin_clear(LED_NODE);

      pTwrInfo->gRtcSFrameZeroCnt = nrf_drv_rtc_counter_get(&rtc);
   
      nrf_gpio_pin_set(LED_NODE);
      gRTCtickCnt = 0;
    }
    gRTCtickCnt++;
}

//-----------------------------------------------------------------------------

/* @brief     app level
 *     RTOS-independent application level function.
 *     initializing of a TWR Node functionality.
 *
 * */

error_e node_process_init(int chip)
{
    twr_info_t    *pTwrInfo = &TwrInfo;

    /* switch off receiver's rxTimeOut, RxAfterTxDelay, delayedRxTime,
     * autoRxEnable, dblBufferMode and autoACK,
     * clear all initial counters, etc.
     * */
    memset(pTwrInfo, 0 , sizeof(twr_info_t));

    if(chip == 1 )
    {
        pDwMaster = &dw_chip_B;
        pDwSlave  = &dw_chip_A;
    }
    else
    {
        pDwMaster = &dw_chip_A;
        pDwSlave  = &dw_chip_B;
    }

    port_reinit_dw_chips();

    /* Configure non-zero initial variables.1 : from app parameters */

    /* The Node has its configuration in the app->pConfig, see DEFAULT_CONFIG.
     * Tag will receive its configuration, such as
     * panID, tagAddr, node0Addr and TWR delays:
     * pollTx2FinalTxDelay_us and tag_replyDly_us from Range Config message.
     *
     * The reception timeouts calculated based on known length of
     * RangeInit and Response packets.
     *
     * */
    pTwrInfo->pSfConfig = &app.pConfig->s.sfConfig; /**< Super Frame configuration */
    pTwrInfo->panID     = app.pConfig->s.panID;     /**< panID    */
    pTwrInfo->eui16     = app.pConfig->s.addr;      /**< Node's address */

    { //calculate two-way ranging frame timings
        dwt_config_t *pCfg = &app.pConfig->dwt_config;  //dwt_config : holds node's UWB mode

        msg_t msg;

        msg.prf             = pCfg->prf;            //Deca define: e.g. DWT_PRF_64M
        msg.dataRate        = pCfg->dataRate;       //Deca define: e.g. DWT_BR_6M8
        msg.txPreambLength  = pCfg->txPreambLength; //Deca define: e.g. DWT_PLEN_128


        msg.msg_len = sizeof(blink_msg_t);
        calculate_msg_time(&msg, &pTwrInfo->msg_time.blink);

        msg.msg_len = sizeof(rng_cfg_msg_t);
        calculate_msg_time(&msg, &pTwrInfo->msg_time.ranging_config);

        msg.msg_len = sizeof(poll_msg_t);
        calculate_msg_time(&msg, &pTwrInfo->msg_time.poll);

        msg.msg_len = sizeof(resp_pdoa_msg_t);
        calculate_msg_time(&msg, &pTwrInfo->msg_time.response);

        msg.msg_len = sizeof(final_msg_accel_t);
        calculate_msg_time(&msg, &pTwrInfo->msg_time.final);
    }

    /* dwt_xx calls in app level Must be in protected mode (DW1000 IRQ disabled) */
    disable_dw1000_irq();

    TWR_ENTER_CRITICAL();

    port_stop_all_UWB();    /**< switch off all UWB and set all callbacks to NULL */

    /* configure the RTC Wakeup timer with a high priority:
     * this timer is saving global Super Frame Timestamp,
     * so we want this timestamp as stable as we can.
     * */
    twr_configure_rtc_wakeup(pTwrInfo->pSfConfig->sfPeriod_ms);         // TBD

    //HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

    /* Configure SLAVE */
    set_dw_spi_slow_rate(DW_SLAVE);
    if (dwt_initialise(DWT_LOADUCODE) != DWT_SUCCESS)
    {
        return (_ERR_INIT);
    }

    /* read SLAVE's OTP Temperature calibration parameter */
    pTwrInfo->TmeasSlave = (uint8_t)(0xff&dwt_gettmeas());

    /* Configure SLAVE receiver's UWB mode, sets power and antenna delays for TWR mode (not used)
     * Configure SPI to fast rate */
    rxtx_node_configure(DW_SLAVE,
                        &app.pConfig->dwt_config,
                        DWT_FF_DATA_EN,             /* enable frame filtering for SLAVE */
                        app.pConfig->s.antTx_b,
                        app.pConfig->s.antRx_b,
                        pTwrInfo->panID,
                        pTwrInfo->eui16 );

    /* NO Callbacks or IRQ from SLAVE */

    /* Configure MASTER */
    set_dw_spi_slow_rate(DW_MASTER);
    if (dwt_initialise(DWT_LOADUCODE) != DWT_SUCCESS)
    {
        return (_ERR_INIT);
    }

    set_dw_spi_fast_rate(DW_MASTER);

    /* read MASTER's OTP Temperature calibration parameter */
    pTwrInfo->TmeasMaster = (uint8_t)(0xff&dwt_gettmeas());

    /* Configure MASTER's receiver's UWB mode, sets power and antenna delays for TWR mode
     * Configure SPI to fast rate */
    rxtx_node_configure(DW_MASTER,
                        &app.pConfig->dwt_config,
                        DWT_FF_NOTYPE_EN,            /* No frame filtering for MASTER */
                        app.pConfig->s.antTx_a,
                        app.pConfig->s.antRx_a,
                        pTwrInfo->panID,
                        pTwrInfo->eui16 );

    /* IRQ from MASTER; and no IRQ from SLAVE */
    dwt_setcallbacks(twr_tx_node_cb, twr_rx_node_cb, twr_rx_timeout_cb, twr_rx_error_cb); // TWD

    dwt_setinterrupt( DWT_INT_TFRS | DWT_INT_RFCG |
                     (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT |
                      DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);


    synchronize_DW1000clocks();

    /* End configure of chips */

    /* Configure non-zero initial variables.2 : dwt_getlotid/dwt_getpartid are valid after dwt_initialise() */
    pTwrInfo->seqNum    = (uint8_t)(0xff*rand()/RAND_MAX);

    TWR_EXIT_CRITICAL();

    return (_NO_ERR);
}



/*
 * @brief
 *     Enable DW1000 IRQ to start
 * */
void node_process_start(void)
{
    enable_dw1000_irq();
}


/* @brief     app level
 *     RTOS-independent application level function.
 *     deinitialize the pTwrInfo structure.
 *    This must be executed in protected mode.
 *
 * */
void node_process_terminate(void)
{
    //do not need to terminate and freed the resources (pTwrInfo)
    //since Node's task is the main task and it used statically allocated resource twrInfo
}

//-----------------------------------------------------------------------------

