/*
 * @file    tag.c
 * @brief    DecaWave Application Layer
 *           TWR functions collection
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
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_drv_rtc.h"
#include <circ_buf.h>
#include <error.h>
#include "deca_regs.h"
#include "math.h"

#include "deca_device_api.h"

#include "cmsis_os.h"
#include "assert.h"
#include <limits.h>
#include <port_platform.h>
#include <tag.h>
#include <uwb_frames.h>
#include <util.h>

//-----------------------------------------------------------------------------
// Definitions
#define SAFE_TXDATA        /*see start_tx()*/

#define TWR_ENTER_CRITICAL()    taskENTER_CRITICAL()
#define TWR_EXIT_CRITICAL()     taskEXIT_CRITICAL()

/* Two parameters will be sent to the Tag in the Ranging Config message:
 * First, is the ~delay after Poll_Tx, when Tag needs to go to Rx, and the second is
 * the ~delay between Poll_Tx's and Final_Tx's R-marks.
 * Tag hardware shall be capable to meet the requested timings.
 * Below defined limits, used to specify Tag's maximum HW capability.
 *
 * That limits depend on the MCU speed, code optimization, latency, introduced by
 * the application architecture, especially number of reading/writing from/to DW1000 and RTOS latencies.
 * Timings can be improved more (decreased) by placing a ranging-dependent
 * functionality below RTOS, (so app will not have a delay, introduced by RTOS itself).
 * However, current realization is not optimized for that.
 * */
#define MIN_RESPONSE_CAPABILITY_US          (100)   /**< time when Tag can be ready to receive the Respose after its Poll */
#define MIN_POLL_TX_FINAL_TX_CAPABILITY_US  (1500)  /**< time for tag to transmit Poll_TX and then Final_TX) */


/* WKUP timer used at Ranging phase.
 * It is counts the Super Frame period, received in the Ranging Config message
 * in Tag's local time domain.
 * */
#define WKUP_RESOLUTION_NS                  (1e9f/32768.f)

/* RTC WKUP timer counts Super Frame period.
 * The RTC WKUP timer prescaler is configured as each Tick count is 30.517 us.
 */
#define RTC_WKUP_PRESCALER          (0)

/* RTC WKUP timer counts Super Frame period.
 * The RTC WKUP timer is 24 bit counter. Counter oveflows at 2^24 - 16777216
 */
#define RTC_WKUP_CNT_OVFLW          (16777216)

/* For best PDOA performance the clock offset between PDOA node and the tag
 * should be >2ppm.
 * In case Tag's crystal is very off, it also will be trimmed to stay in the [2 .. 4]ppm range,
 * as defined below:
 * */
#define TARGET_XTAL_OFFSET_VALUE_PPHM_MIN   (200)
#define TARGET_XTAL_OFFSET_VALUE_PPHM_MAX   (400)
#define AVG_TRIM_PER_PPHM                   (32.f/4800.f) /* Trimimg per 100 ppm*/

//-----------------------------------------------------------------------------

/* const txSpectrumConfig for channel 5 spectrum config */
static const struct
{
    uint8_t     PGdelay;

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
        0x25456585  //64M prf power
    }
};

//-----------------------------------------------------------------------------

static int rtcInitState = 0;
static uint32_t gRTC_SF_PERIOD = 3276;

// TWR structure holds all TWR data
#define TAG_STATIC_TWRINFO
#ifdef TAG_STATIC_TWRINFO
//static ("safe") implementation
static twr_info_t    TwrInfo;

#else
//dynamic allocation of TwrInfo
static twr_info_t    *pTwrInfo = NULL;

#define TAG_MALLOC     pvPortMalloc
#define TAG_FREE    vPortFree

#endif

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Local functions prototypes
static void rtcWakeUpTimerEventCallback(nrf_drv_rtc_int_type_t int_type);
static void twr_configure_rtc_wakeup(uint32_t     period);

//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Implementation


//-----------------------------------------------------------------------------
// Support section

/*
 * @brief     get pointer to the twrInfo structure
 * */
twr_info_t *
getTwrInfoPtr(void)
{
#ifdef TAG_STATIC_TWRINFO
    return (&TwrInfo);
#else
    return (pTwrInfo);
#endif
}


/*
 * @brief   ISR level (need to be protected if called from APP level)
 *          low-level configuration for DW1000
 *
 *          if called from app, shall be performed with DW IRQ off &
 *          TWR_ENTER_CRITICAL(); / TWR_EXIT_CRITICAL();
 *
 *
 * @note
 * */
static void
rxtx_tag_configure(rxtx_tag_configure_t *p)
{
    assert(p->pdwCfg->chan == 5);            /**< This project supports only CH5 + DWT_PRF_64M */

    dwt_txconfig_t  dtmp;
    uint32_t        power;

    dwt_configure(p->pdwCfg);    /**< Configure the Physical Channel parameters (PLEN, PRF, etc) */

    /* configure power */
    power = txSpectrumConfig.txPwr[p->pdwCfg->prf - DWT_PRF_16M];

    if(app.pConfig->s.smartTxEn == 0)
    {
        power = (power & 0xff) ;
        power |= (power << 8) + (power << 16) + (power << 24);
        dwt_setsmarttxpower(0);
    }
    else
    {
        dwt_setsmarttxpower(1);
    }

    dtmp.power = power;
    dtmp.PGdly = txSpectrumConfig.PGdelay ;

    dwt_configuretxrf(&dtmp);

    /* set antenna delays */
    dwt_setrxantennadelay(p->rxAntDelay);
    dwt_settxantennadelay(p->txAntDelay);

    dwt_setdblrxbuffmode (0);    /**< dblBuf is not used in TWR */
    dwt_setrxaftertxdelay(0);    /**< no any delays set by default : part of config of receiver on Tx sending */
    dwt_setrxtimeout     (0);    /**< no any delays set by default : part of config of receiver on Tx sending */
    dwt_enableframefilter(p->frameFilter);

    dwt_setpanid(p->panId);

    /*patch for preamble length 64 */
    if(p->pdwCfg->txPreambLength == DWT_PLEN_64)
    {
        set_dw_spi_slow_rate(DW_MASTER);
        dwt_loadopsettabfromotp(DWT_OPSET_64LEN);
        set_dw_spi_fast_rate(DW_MASTER);
    }

    dwt_setaddress16(p->shortadd);

}


/*
 * @brief   ISR level (need to be protected if called from APP level)
 * @param   twr_info_t *pTwrInfo has two members
 *          xtaltrim - current trimmed value
 *          clkOffset_pphm -
 *
 * @note    This changes the DW1000 system clock and shall be performed
 *          when DW1000 is not in active Send/Receive state.
 * */
void trim_tag_proc(twr_info_t *pTwrInfo)
{
    unsigned tmp = abs(pTwrInfo->clkOffset_pphm);

    if(tmp > TARGET_XTAL_OFFSET_VALUE_PPHM_MAX ||
       tmp < TARGET_XTAL_OFFSET_VALUE_PPHM_MIN)
    {
        int8_t tmp8 = pTwrInfo->xtaltrim;
        tmp8 -= (int8_t)(((TARGET_XTAL_OFFSET_VALUE_PPHM_MAX + TARGET_XTAL_OFFSET_VALUE_PPHM_MIN)/2 + pTwrInfo->clkOffset_pphm) * AVG_TRIM_PER_PPHM);
        pTwrInfo->xtaltrim = (uint8_t)(FS_XTALT_MASK & tmp8);

        /* Configure new Crystal Offset value */
        dwt_setxtaltrim(pTwrInfo->xtaltrim);
    }
}


#if (DIAG_READ_SUPPORT==1)
/*
 * @brief    ISR layer
 *     read full diagnostic data form the received frame from the two DW1000s
 *     offset 0 / 1
 * */
static int
read_full_diagnostics(rx_pckt_t *prxPckt,
                        uint32   status )
{
    uint16_t     fpIndex;
    diag_v5_t    *p = &prxPckt->diagnostics;

    p->header = DWT_DIAGNOSTIC_LOG_REV_5;

    memcpy(p->r0F, (uint8_t*) &status, 4);                      //copy 4bytes of status (saved on entry to ISR)
    dwt_readfromdevice(RX_FINFO_ID, 4, 5, (uint8_t*)(p+5) );    //read MSB from status and 4byte frame info
    dwt_readfromdevice(RX_FQUAL_ID, 0, 17,(uint8_t *)(p->r12)); //read 17 bytes of diagnostic data from 0x12,13,14

    memcpy((uint8_t*)p->r15, prxPckt->rxTimeStamp, TS_40B_SIZE);//copy TS
    dwt_readfromdevice(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET, 9, (uint8_t *)(p->r15 + 5)); //2FP, 2Diag, 5TSraw

    // Calculate the First Path Index ((LDE0 + LDE1 << 8) / 64)
    fpIndex = (*((uint8_t*)(p+32)) >> 6) + (*((uint8_t*)(p+33)) << 2);

    fpIndex = fpIndex*4 + 1;                 //get location in the accumulator

    //printf("%d FP index %02x %02x %i %i\n", offset, *((uint8_t*)(p+32)), *((uint8_t*)(p+33)), fpIndex, (fpIndex-1)>>2);
    //Read CIR for the First Path + 3 More samples (4*4 = 16)
    dwt_readaccdata(p->r25, 17, fpIndex-1); //read 1 extra as first will be dummy byte
    dwt_readfromdevice(LDE_IF_ID, LDE_PPINDX_OFFSET, 2, p->r2E);
    dwt_readfromdevice(DRX_CONF_ID, 0x28,            4, p->r27);
    dwt_readfromdevice(LDE_IF_ID, LDE_PPAMPL_OFFSET, 2, p->r2E2);

    return (int) fpIndex ;
}
#endif

/**
 * @brief   ISR layer
 *          Transmit packet
 * */
static error_e
tx_start(tx_pckt_t * pTxPckt)
{
    error_e ret = _NO_ERR;
    uint8_t  txFlag = 0;

    dwt_forcetrxoff();    //Stop the Receiver and Write Control and Data

#ifdef SAFE_TXDATA
    dwt_writetxdata(pTxPckt->psduLen, (uint8_t *) &pTxPckt->msg.stdMsg, 0);
#endif

    dwt_writetxfctrl(pTxPckt->psduLen, 0, 1);

    //Setup for delayed Transmit
    if(pTxPckt->delayedTxTimeH_sy != 0UL)
    {
        dwt_setdelayedtrxtime(pTxPckt->delayedTxTimeH_sy) ;
    }

    if(/*(pTxPckt->delayedRxTime_sy !=0) && */(pTxPckt->txFlag & DWT_RESPONSE_EXPECTED))
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

#ifndef SAFE_TXDATA
    /* while transmitting of the preamble we can fill the data path
     * to save time : this is not "safe" approach and
     * additional check to be done to use this feature.
     * TODO: left for future : do not use this feature
     * */
    if (ret == _NO_ERR)
    {
        dwt_writetxdata(pTxPckt->psduLen, (uint8_t *)  &pTxPckt->msg.stdMsg, 0);
    }
#endif

    return (ret);
}

//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
//    DW1000 callbacks section :
//    if RTOS, the preemption priority of the dwt_isr() shall be such, that
//    allows signal to the thread.

/* @brief    ISR layer
 *             Real-time TWR application Tx callback
 *            to be called from dwt_isr()
 * */
static void
twr_tx_tag_cb(const dwt_cb_data_t *txd)
{
    twr_info_t *    pTwrInfo = getTwrInfoPtr();

    if(!pTwrInfo)
    {
        return;
    }

    uint32_t tmp = nrf_drv_rtc_counter_get(&rtc);		// MCU RTC time : this will be HW timestamped

    // Store the Tx Time Stamp of the transmitted packet
    switch(pTwrInfo->txState)
    {
    case Twr_Tx_Blink_Sent :                    //tag
        pTwrInfo->blinkRtcTimeStamp = tmp;
        dwt_readtxtimestamp(pTwrInfo->blinkTx_ts);
        break;
    case Twr_Tx_Poll_Sent :                     //tag
        pTwrInfo->pollRtcTimeStamp = tmp;
        dwt_readtxtimestamp(pTwrInfo->pollTx_ts);
        break;
    case Twr_Tx_Final_Sent :                    //tag
        pTwrInfo->finalRtcTimeStamp = tmp;
        dwt_readtxtimestamp(pTwrInfo->finalTx_ts);
        trim_tag_proc(pTwrInfo);                //Trim Tag's offset automatically WRT to the PDOA-Node
        app.DwCanSleep = 1;
        break;
    default :
        break;
    }
}

/* @brief     ISR layer
 *             TWR application Rx callback
 *             to be called from dwt_isr() as an Rx call-back
 * */
static void
twr_rx_cb(const dwt_cb_data_t *rxd)
{
    twr_info_t     *pTwrInfo = getTwrInfoPtr();

    if(!pTwrInfo)
    {
        return;
    }

    uint16_t head = pTwrInfo->rxPcktBuf.head;
    uint16_t tail = pTwrInfo->rxPcktBuf.tail;
    uint16_t size = sizeof(pTwrInfo->rxPcktBuf.buf) / sizeof(pTwrInfo->rxPcktBuf.buf[0]);

    if(CIRC_SPACE(head,tail,size) <= 0)
    {
        return;
    }

    rx_pckt_t *p = &pTwrInfo->rxPcktBuf.buf[head];

    p->rxRtcTimeStamp = nrf_drv_rtc_counter_get(&rtc);    // MCU RTC timestamp 

    dwt_readrxtimestamp(p->rxTimeStamp);    //Raw Rx TimeStamp

    p->rxDataLen = MIN(rxd->datalength, sizeof(twr_msg_t));

    dwt_readrxdata((uint8_t *)&p->msg.stdMsg, p->rxDataLen, 0); //Raw message

#if (DIAG_READ_SUPPORT == 1)
    read_full_diagnostics(p, rxd->status);
#endif

    if(app.rxTask.Handle != NULL) // RTOS : rxTask can be not started
    {
        head = (head + 1) & (size-1);
        pTwrInfo->rxPcktBuf.head = head;    //IRQ : do not need to protect

        if(osSignalSet(app.rxTask.Handle, app.rxTask.Signal) != osOK)
        {
            error_handler(1, _Err_Signal_Bad);
        }
    }
}

/*
 * @brief    ISR layer
 *
 * */
static void
twr_rx_timeout_cb(const dwt_cb_data_t *rxd)
{
    twr_info_t     *pTwrInfo = getTwrInfoPtr();

    dwt_forcetrxoff() ;

    if(!pTwrInfo)
    {
        return;
    }

    if(pTwrInfo->txState == Twr_Tx_Poll_Sent)
    {
        pTwrInfo->faultyRangesCnt++;
    }

//range aborted
}


static void
twr_rx_error_cb(const dwt_cb_data_t *rxd)
{
    twr_rx_timeout_cb(rxd);
}
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------

/* @brief     app layer
 *     RTOS independent application layer function.
 *     initialising of TWR from scratch.
 *    This MUST be executed in protected mode.
 *
 *     This will setup the process of:
 *     1. broadcast blink / wait for Ranging Config response;
 *     2. receive setup parameters from Ranging Config;
 *     3. if version of Ranging Config is not compatible, keep blinking;
 *     4. otherwise setup slot, new panID, framefiltering, address, twr timings;
 *     6. switch off blinking timer and switch on precise WUP timer;
 *     5. range to the Node addr from MAC of Ranging Config
 * */
error_e tag_process_init(void)
{

#ifdef TAG_STATIC_TWRINFO
    twr_info_t *pTwrInfo = &TwrInfo;
#else

    pTwrInfo = TAG_MALLOC(sizeof(twr_info_t));

    if(!pTwrInfo)
    {
        return(_ERR_Cannot_Alloc_Memory);
    }

#endif

    /* switch off receiver's rxTimeOut, RxAfterTxDelay, delayedRxTime,
     * autoRxEnable, dblBufferMode and autoACK,
     * clear all initial counters, etc.
     * */
    memset(pTwrInfo, 0 , sizeof(twr_info_t));

    /* Tag will receive its configuration, such as
     * panID, tagAddr, node0Addr and TWR delays:
     * pollTx2FinalTxDelay_us and response rx delay from Ranging Config message.
     *
     * But the reception timeouts calculated based on known length of
     * Ranging Config and Response packets.
     * */

    { //pre-calculate all possible two-way ranging frame timings
        dwt_config_t *pCfg = &app.pConfig->dwt_config;    //dwt_config : holds node's UWB mode

        msg_t msg;

        msg.prf             = pCfg->prf;            //Deca define: e.g. DWT_PRF_64M
        msg.dataRate        = pCfg->dataRate;       //Deca define: e.g. DWT_BR_6M8
        msg.txPreambLength  = pCfg->txPreambLength; //Deca define: e.g. DWT_PLEN_128


        msg.msg_len = sizeof(rng_cfg_msg_t);
        calculate_msg_time(&msg, &pTwrInfo->msg_time.ranging_config);

        msg.msg_len = sizeof(poll_msg_t);
        calculate_msg_time(&msg, &pTwrInfo->msg_time.poll);

        msg.msg_len = sizeof(resp_ext_msg_t);
        calculate_msg_time(&msg, &pTwrInfo->msg_time.response);

        msg.msg_len = sizeof(final_msg_accel_t);
        calculate_msg_time(&msg, &pTwrInfo->msg_time.final);
    }

    /* dwt_xx calls in app level Must be in protected mode (DW1000 IRQ disabled) */
    disable_dw1000_irq();

    TWR_ENTER_CRITICAL();

    port_stop_all_UWB();    /**< switch off all UWB and set callbacks to NULL */

    if (dwt_initialise(DWT_LOADUCODE) != DWT_SUCCESS)
    {
        return (_ERR_INIT);   // device initialise has failed
    }

    set_dw_spi_fast_rate(DW_MASTER);

    /* Configure receiver's UWB mode, set power and antenna delays for TWR mode */
    rxtx_tag_configure_t p;
    p.pdwCfg      = &app.pConfig->dwt_config;
    p.frameFilter = DWT_FF_NOTYPE_EN;    //DWT_FF_DATA_EN
    p.txAntDelay  = app.pConfig->s.ant_tx_a;
    p.rxAntDelay  = app.pConfig->s.ant_rx_a;
    p.panId       = 0x5555;//PanID : does not matter : DWT_FF_NOTYPE_EN : will be reconfigured on reception of RI message
    p.shortadd    = 0xAAAA;//ShortAddr : does not matter : DWT_FF_NOTYPE_EN : will be reconfigured on reception of RI message
    rxtx_tag_configure(&p);

    dwt_setleds(3) ;            /**< DEBUG I/O 2&3 : configure the GPIOs which control the LEDs on HW */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);        /**< DEBUG I/O 5&6 : configure TX/RX states to output on GPIOs */

    dwt_setcallbacks(twr_tx_tag_cb, twr_rx_cb, twr_rx_timeout_cb, twr_rx_error_cb);

    dwt_configuresleep(DWT_PRESRV_SLEEP | DWT_CONFIG, DWT_WAKE_CS | DWT_SLP_EN);

    dwt_setinterrupt( DWT_INT_TFRS | DWT_INT_RFCG |
                     (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT |
                      DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);


    init_dw1000_irq();            /**< manually init EXTI DW1000 lines IRQs */

    /* configure non-zero initial values */
    pTwrInfo->env.responseRxTo_sy  = 2 * pTwrInfo->msg_time.response.sy;    //to be able receive long rng_cfg_upd_msg_t relax Response RX timeout time

    pTwrInfo->seqNum    = (uint8_t)(0xff*rand()/RAND_MAX);
    pTwrInfo->eui64     = ((uint64_t)dwt_getlotid()<<32) + dwt_getpartid(); //valid after dwt_initialise()
    pTwrInfo->xtaltrim = dwt_getinitxtaltrim();                             //valid after dwt_initialise()

    TWR_EXIT_CRITICAL();


    return (_NO_ERR);
}


/*
 * */
void tag_process_start(void)
{
    enable_dw1000_irq();         /**< enable DW1000 IRQ to start  */
}


/* @brief   app level
 *          RTOS-independent application level function.
 *          deinitialize the pTwrInfo structure.
 *  This must be executed in protected mode.
 *
 * */
void tag_process_terminate(void)
{
    TWR_ENTER_CRITICAL();

    twr_info_t     *pTwrInfo = getTwrInfoPtr();

    NVIC_DisableIRQ(rtc.irq);


    if (pTwrInfo)
    {
#ifndef TAG_STATIC_TWRINFO
        TAG_FREE(pTwrInfo);
#endif
    }

    TWR_EXIT_CRITICAL();
}
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------

/* @brief
 *          TWR : DISCOVERY PHASE
 *          Tag sends blinks, searching for a Ranging Config message
 *
 *          application layer function
 */
error_e    initiator_send_blink(twr_info_t *p)
{
    error_e       ret;
    tx_pckt_t     txPckt;

    memset(&txPckt, 0, sizeof(txPckt));

    blink_msg_t *pTxMsg = &txPckt.msg.blinkMsg;

    // TWR : PHASE : Initiator Sends Blink to Responder
    txPckt.psduLen              = sizeof(blink_msg_t);
    txPckt.delayedTxTimeH_sy    = 0;
    txPckt.txFlag               = ( DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED );
    txPckt.delayedRxTime_sy     = (uint32_t)util_us_to_sy(app.pConfig->s.rcDelay_us);  //activate receiver this time SY after Blink Tx
    txPckt.delayedRxTimeout_sy  = (uint32_t)util_us_to_sy(app.pConfig->s.rcRxTo_us);     //receiver will be active this time, SY

    pTxMsg->frameCtrl[0]        = Head_Msg_BLINK;
    pTxMsg->seqNum              = p->seqNum;
    memcpy(&pTxMsg->tagID, &p->eui64, sizeof(pTxMsg->tagID));

    p->seqNum++;
    p->txState                  = Twr_Tx_Blink_Sent;

    TWR_ENTER_CRITICAL();

    ret = tx_start(&txPckt);

    TWR_EXIT_CRITICAL();

    if( ret != _NO_ERR)
    {
        p->lateTX++;
    }

    return (ret);
}


/* @brief
 *          TWR: RANGING PHASE
 *          Initiator sends Poll to the Responder
 *
 *          application layer function
 */
error_e    initiator_send_poll(twr_info_t *p)
{
    error_e      ret;
    tx_pckt_t    txPckt;

    poll_msg_t   *pTxMsg  = &txPckt.msg.pollMsg;

    /* Construct TxPckt packet: Send Poll immediate and configure delayed wait for the Response */
    txPckt.psduLen              = sizeof(poll_msg_t);
    txPckt.delayedTxTimeH_sy    = 0;
    txPckt.txFlag               = ( DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED );

    txPckt.delayedRxTime_sy     = p->env.delayRx_sy;        //environment, received from Ranging Init
    txPckt.delayedRxTimeout_sy  = p->env.responseRxTo_sy;   //this is calculated locally based on UWB configuration

    /* Construct TX Final UWB message */

    /* See IEEE frame header description */
    pTxMsg->mac.frameCtrl[0]    = Head_Msg_STD;
    pTxMsg->mac.frameCtrl[1]    = Frame_Ctrl_SS;
    pTxMsg->mac.panID[0]        = p->env.panID & 0xff;
    pTxMsg->mac.panID[1]        = (p->env.panID >> 8) & 0xff;
    pTxMsg->mac.destAddr[0]     = p->env.nodeAddr[0];
    pTxMsg->mac.destAddr[1]     = p->env.nodeAddr[1];
    pTxMsg->mac.sourceAddr[0]   = p->env.tagAddr[0];
    pTxMsg->mac.sourceAddr[1]   = p->env.tagAddr[1];
    pTxMsg->mac.seqNum          = p->seqNum;

    /* Data */
    pTxMsg->poll.fCode          = Twr_Fcode_Tag_Poll;
    pTxMsg->poll.rNum           = p->rangeNum;

    /* Transmit over the air */
    p->txState                  = Twr_Tx_Poll_Sent;
    p->seqNum++;
    p->rangeNum++;

    POLL_ENTER_CRITICAL();

    ret = tx_start(&txPckt);

    POLL_EXIT_CRITICAL();

    if( ret != _NO_ERR)
    {
        p->lateTX++;
    }

    return (ret);
}

/* @brief     Part of twr_initiator_algorithm_tx.
 *
 * Initiator wakes DW1000 chip to send Blink or Poll message.
 *
 * */
void tag_wakeup_dw1000_blink_poll(twr_info_t *p)
{
    set_dw_spi_fast_rate(DW_MASTER);

    p->stationary = p->stationary_imu;
    app.DwCanSleep = 0;

#if 0     // TBD
    if(port_wakeup_dw1000_fast() != _NO_ERR)
    {
        error_handler(0,  _ERR_DEVID);
    }
#endif

    /* Below needs to be restored on the wakeup */
    dwt_setleds(3) ;            /**< DEBUG I/O 2&3 : configure the GPIOs which control the LEDs on HW */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);      /**< DEBUG I/O 5&6 : configure TX/RX states to output on GPIOs */
    //dwt_setrxantennadelay(app.pConfig->s.ant_rx_a);
    dwt_settxantennadelay(app.pConfig->s.ant_tx_a); //Tx antenna delay is not preserved during sleep
}

/* @brief     Part of twr_initiator_algorithm_tx.
 *
 * Initiator sends POLL message.
 *
 * */
void initiator_wake_and_send_poll(twr_info_t *p)
{
    POLL_ENTER_CRITICAL();
    tag_wakeup_dw1000_blink_poll(p);
    POLL_EXIT_CRITICAL();

    if(p->faultyRangesCnt < app.pConfig->s.faultyRanges)
    {
        initiator_send_poll(p);
    }
    else
    {
        POLL_ENTER_CRITICAL();
        NVIC_DisableIRQ(rtc.irq);
        POLL_EXIT_CRITICAL();

        /* This will restart Tag task completely from discovery phase */
        xEventGroupSetBits(app.xStartTaskEvent, Ev_Tag_Task);
    }
}

//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// The most real-time section of TWR algorithm

/* @brief    app layer
 *             Real-time TWR algorithm implementation (Initiator)
 *
 *             prefer to be called from application UWB Rx,
 *            but can be called from ISR, i.e. twr_rx_cb() directly.
 *             if called from ISR layer, then revise/remove
 *             TWR_ENTER_CRITICAL()
 *             TWR_EXIT_CRITICAL()
 *
 * @return     _NO_ERR for no errors
 * */
error_e twr_initiator_algorithm_rx(rx_pckt_t *pRxPckt, twr_info_t *pTwrInfo)
{
    error_e     ret   = _Err_Not_Twr_Frame;
    fcode_e     fcode = Twr_Fcode_Not_Defined;

    std_msg_t   *pMsg = &pRxPckt->msg.stdMsg;

    if(pMsg->mac.frameCtrl[0] == Head_Msg_STD || pMsg->mac.frameCtrl[0] == Head_Msg_STD1)
    {
        /* Only SS and LS MAC headers supported in current application */
        switch (pMsg->mac.frameCtrl[1] & Frame_Ctrl_MASK)
        {
        case Frame_Ctrl_SS:
            if((pRxPckt->rxDataLen == sizeof(resp_ext_msg_t)) ||    /* Response (with Coordinates extension) */
               (pRxPckt->rxDataLen == sizeof(rng_cfg_upd_msg_t)))   /* Ranging Config (update) */
            {
                fcode = ((std_msg_ss_t*)pMsg)->messageData[0] ;
            }
            break;
        case Frame_Ctrl_LS:
            if(pRxPckt->rxDataLen == sizeof(rng_cfg_msg_t))         /* Ranging Config (discovery) */
            {
                fcode = ((std_msg_ls_t*)pMsg)->messageData[0] ;
            }
            break;
        default:
            fcode = Twr_Fcode_Not_Defined ;
            break;
        }
    }

    switch (fcode)
    {
        case Twr_Fcode_Rng_Config :
            /* Initiator received Range Init message from a Responder in discovery process.
             * 1. setup Timing parameters in accordance to Range Init message;
             * 2. configure MCU RTC WKUP timer to expire on next slot time
             * */
            ret = initiator_received_ranging_config(pRxPckt, pTwrInfo);

            if(ret == _NO_ERR)
            {
                ret = _No_Err_Ranging_Config;
            }
            break;

        case Twr_Fcode_Resp_Ext:
            /* Initiator received a Response message.
             * If the message from our PDOA node:
             * 1. Send delayed Final_TX;
             * 2. Adjust the Wakeup timer for next Poll.
             * */
            if (pTwrInfo->env.nodeAddr[0] == ((resp_ext_msg_t*)pMsg)->mac.sourceAddr[0] ||
                pTwrInfo->env.nodeAddr[1] == ((resp_ext_msg_t*)pMsg)->mac.sourceAddr[1])
            {
                //(1)
                ret = initiator_received_response(pRxPckt, pTwrInfo);

                if(ret == _NO_ERR)
                {
                    ret = _No_Err_Response;
                }

                //(2)
                TWR_ENTER_CRITICAL();


                int32_t         slotCorr_ns;
                uint32_t        nextWakeUpPeriod_ns, rtcNow, tmpNow;

                // casting received bytes to the int,
                // this is a signed correction wrt (gRtcSFrameZeroCnt + expected_slot) coming from the Node
                pTwrInfo->env.slotCorr_ns = 1000 * (int32_t)AR2U32(((resp_ext_msg_t*)pMsg)->resp.slotCorr_us);

                slotCorr_ns = pTwrInfo->env.slotCorr_ns;

                //(A)start of calculation
                rtcNow = nrf_drv_rtc_counter_get(&rtc);

                tmpNow = (rtcNow > pTwrInfo->gRtcSFrameZeroCnt)?
                         (rtcNow - pTwrInfo->gRtcSFrameZeroCnt):
                         (rtcNow - pTwrInfo->gRtcSFrameZeroCnt + RTC_WKUP_CNT_OVFLW);

                //Next RTC_Wakeup of Tag will be aligned to exact slot expected by the Node's:
                nextWakeUpPeriod_ns  = pTwrInfo->env.sframePeriod_ns
                                    -(WKUP_RESOLUTION_NS * tmpNow)
                                    -(slotCorr_ns);

                if(nextWakeUpPeriod_ns < (pTwrInfo->env.sframePeriod_ns >>1))
                {
                    nextWakeUpPeriod_ns += (pTwrInfo->env.sframePeriod_ns);
                }

                twr_configure_rtc_wakeup(nextWakeUpPeriod_ns);

                TWR_EXIT_CRITICAL();
            }
            break;

        default:
            /* Initiator received unknown data / reset initiator
             * */
            ret = _Err_Not_Twr_Frame;
            break;
    }

    return (ret);
}


/* @brief check on-the air configuration that it capable
 *           to work on the current hardware
 * */
error_e check_air_config_correct(uint8_t ver, uint16_t delayRx_sy, uint16_t pollTxToFinalTx_us)
{
    error_e ret = _NO_ERR;
    if (ver != RC_VERSION_PDOA)
    {//This Tag knows only one version of Ranging Config format: RC_VERSION_PDOA
        ret = _Err_RC_Version_Unknown;
    }
    else if ((delayRx_sy < MIN_RESPONSE_CAPABILITY_US) ||\
        (pollTxToFinalTx_us < MIN_POLL_TX_FINAL_TX_CAPABILITY_US))
    {//This Tag hardware is too slow and cannot range with parameters, supplied by the Node.
        ret = _Err_Non_Compatible_TWR_Parameters;
    }
    return ret;
}


/* @brief     Part of twr_initiator_algorithm_rx
 *
 * Initiator in discovery phase received RANGING CONFIG message from a Responder.
 * 1. Check that Tag understand and can comply to requested Ranging Config parameters;
 * 2. Setup Range Times as defined in the Ranging Config message;
 * 3. Switch off the Blink Timer.
 * 4. Configure & Start the Poll Timer.
 *
 * @parm    *rx_packet, *control structure
 *
 * @return error code
 * */
error_e initiator_received_ranging_config(rx_pckt_t *pRxPckt, twr_info_t *pTwrInfo)
{
    error_e ret = _NO_ERR;

    rng_cfg_t *pRxMsg;

    if(pRxPckt->rxDataLen == sizeof(rng_cfg_msg_t))
    {
        /* obtain Node address and PanID from the MAC of Ranging Config */
        pTwrInfo->env.panID       = AR2U16(pRxPckt->msg.rngCfgMsg.mac.panID);
        pTwrInfo->env.nodeAddr[0] = pRxPckt->msg.rngCfgMsg.mac.sourceAddr[0];
        pTwrInfo->env.nodeAddr[1] = pRxPckt->msg.rngCfgMsg.mac.sourceAddr[1];

        pRxMsg = &pRxPckt->msg.rngCfgMsg.rngCfg;
    }
    else if (pRxPckt->rxDataLen == sizeof(rng_cfg_upd_msg_t))
    {
        /* update */
        pRxMsg = &pRxPckt->msg.rngCfgUpdMsg.rngCfg;
    }
    else
    {
        return ret;
    }

    ret = check_air_config_correct(pRxMsg->version,
                                   AR2U16(pRxMsg->delayRx_us),
                                   AR2U16(pRxMsg->pollTxToFinalTx_us));

    if( ret == _NO_ERR)
    {
        if(app.blinkTmr.Handle)
        {
            osTimerStop(app.blinkTmr.Handle);    //stop blink timer : RTOS timer
        }

        TWR_ENTER_CRITICAL();

        NVIC_DisableIRQ(rtc.irq);

        /* configure environment parameters from Ranging Config message */
        pTwrInfo->env.version    = pRxMsg->version;

        pTwrInfo->env.tagAddr[0] = pRxMsg->tagAddr[0];
        pTwrInfo->env.tagAddr[1] = pRxMsg->tagAddr[1];

        pTwrInfo->env.sframePeriod_ns       = 1000000 * (uint32_t)(AR2U16(pRxMsg->sframePeriod_ms));
        pTwrInfo->env.pollTx2FinalTxDelay32 = (uint32_t)util_us_to_dev_time(AR2U16(pRxMsg->pollTxToFinalTx_us));//Time from SFD of Poll to SFD of Final.
        pTwrInfo->env.delayRx_sy            = (uint32_t)util_us_to_sy(AR2U16(pRxMsg->delayRx_us));      //time after Poll Tx completed to activate the RX
        pTwrInfo->env.pollMultFast          = AR2U16(pRxMsg->pollMultFast);
        pTwrInfo->env.pollMultSlow          = AR2U16(pRxMsg->pollMultSlow);
        pTwrInfo->env.mode                  = AR2U16(pRxMsg->mode);
        pTwrInfo->env.slotCorr_ns           = 1000 * (int32_t)AR2U32(pRxMsg->slotCorr_us);              //next wakeup correction to fit the slot


        rxtx_tag_configure_t p;
        p.pdwCfg = &app.pConfig->dwt_config;
        p.frameFilter = DWT_FF_NOTYPE_EN;    //DWT_FF_DATA_EN
        p.txAntDelay  = app.pConfig->s.ant_tx_a;
        p.rxAntDelay  = app.pConfig->s.ant_rx_a;
        p.panId       = pTwrInfo->env.panID;
        p.shortadd    = AR2U16(pTwrInfo->env.tagAddr);

        /* Setup configured panID, FrameFiltering and antenna delays */
        rxtx_tag_configure(&p);

        /* Setup New High resolution RTC Wakup Timer : */
        uint32_t    tmp;
        uint32_t    rtcNow = nrf_drv_rtc_counter_get(&rtc);

        tmp = pTwrInfo->env.sframePeriod_ns
                - (WKUP_RESOLUTION_NS * (0xFFFFFF & (rtcNow - pTwrInfo->blinkRtcTimeStamp)))
                - 1000*pTwrInfo->msg_time.poll.us
                - pTwrInfo->env.slotCorr_ns;

        /* configure the RTC Wakup timer with highest RTOS priority.
         * This timer is counting a Super Frame period and signals to poll thread.
         * */
        twr_configure_rtc_wakeup(tmp);

        //NVIC_SetPriority((IRQn_Type)rtc.irq, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
        NVIC_SetPriority((IRQn_Type)rtc.irq, 7);
        NVIC_EnableIRQ((IRQn_Type)rtc.irq);

        TWR_EXIT_CRITICAL();

    }

    return (ret);
}

/* @brief     Part of twr_initiator_algorithm_rx.
 *
 * Initiator received the RESPONSE message.
 * 1. setup and send delayed Final_TX according to control structure
 * 2. if a use case : setup the delayed receive for reception of REPORT TOF
 *
 * @parm    *rx_packet, *control structure
 *
 * @return delayed TxPool error code
 * */
error_e initiator_received_response(rx_pckt_t *prxPckt, twr_info_t *p)
{
    error_e     ret;
    uint64_t    calcFinalTx64 ;

    tx_pckt_t   TxPckt;                 /**< allocate space for Tx Packet */

    final_msg_accel_t      *pTxMsg      = &TxPckt.msg.finalMsg;

    /* Construct TxPckt packet: no reception after Final transmission */
    TxPckt.psduLen                = sizeof(final_msg_accel_t);
    TxPckt.txFlag                 = ( DWT_START_TX_DELAYED );
    TxPckt.delayedRxTime_sy       = 0;
    TxPckt.delayedRxTimeout_sy    = 0;

    /* Calculate timings for transmission of Final Packet */
    // Load Poll Tx time
    TS2U64_MEMCPY(calcFinalTx64, p->pollTx_ts);

    // Add delay from Ranging Config message: PollTx2FinalTx
    calcFinalTx64 = MASK_TXDTS & ((uint64_t)(calcFinalTx64 + p->env.pollTx2FinalTxDelay32));

    // DW1000 will adjust the transmission of Final TxPacket SFD exactly at PollTx+PollTx2FinalTx+AntTxDelay
    TxPckt.delayedTxTimeH_sy    = calcFinalTx64 >>8;

    // Calculate Time Final message will be sent and embed this number to the Final message data.
    // Sending time will be delayedReplyTime, snapped to ~125MHz or ~250MHz boundary by
    // zeroing its low 9 bits, and then having the TX antenna delay added.
    // Getting antenna delay from the config and add it to the Calculated TX Time
    calcFinalTx64 = MASK_40BIT & (uint64_t)(calcFinalTx64 + app.pConfig->s.ant_tx_a);

    /* Construct TX Final UWB message */

    /* See IEEE frame header description */
    pTxMsg->mac.frameCtrl[0]    = Head_Msg_STD;
    pTxMsg->mac.frameCtrl[1]    = Frame_Ctrl_SS;
    pTxMsg->mac.panID[0]        = p->env.panID & 0xff;
    pTxMsg->mac.panID[1]        = (p->env.panID >> 8) & 0xff;
    pTxMsg->mac.destAddr[0]     = p->env.nodeAddr[0];
    pTxMsg->mac.destAddr[1]     = p->env.nodeAddr[1];
    pTxMsg->mac.sourceAddr[0]   = p->env.tagAddr[0];
    pTxMsg->mac.sourceAddr[1]   = p->env.tagAddr[1];
    pTxMsg->mac.seqNum          = p->seqNum;

    /* Data */
    pTxMsg->final.fCode       = (uint8_t)Twr_Fcode_Tag_Accel_Final;
    pTxMsg->final.rNum        = (uint8_t)p->rangeNum;
    TS2TS_UWB_MEMCPY( pTxMsg->final.pollTx_ts,     p->pollTx_ts);           // Embed Poll Tx time to the Final message
    TS2TS_UWB_MEMCPY( pTxMsg->final.responseRx_ts, prxPckt->rxTimeStamp);   // Embed Response Rx time to the Final message
    U642TS_UWB_MEMCPY(pTxMsg->final.finalTx_ts,    calcFinalTx64);          // Embed Calculated Final TX time to the Final message
    pTxMsg->final.flag        = p->stationary;
    pTxMsg->final.acc_x[0]    = p->acc.acc_x[0];
    pTxMsg->final.acc_x[1]    = p->acc.acc_x[1];
    pTxMsg->final.acc_y[0]    = p->acc.acc_y[0];
    pTxMsg->final.acc_y[1]    = p->acc.acc_y[1];
    pTxMsg->final.acc_z[0]    = p->acc.acc_z[0];
    pTxMsg->final.acc_z[1]    = p->acc.acc_z[1];

    /* Transmit over the air */
    p->txState  = Twr_Tx_Final_Sent;    //indicate to TX ISR that the response has been sent
    p->seqNum++;

    TWR_ENTER_CRITICAL();

    ret = tx_start(&TxPckt);

    TWR_EXIT_CRITICAL();

    if( ret != _NO_ERR)
    {
        p->lateTX++;
    }

    return (ret);
}

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/*
 * @brief    setup RTC wakeup timer
 * */
static void twr_configure_rtc_wakeup(uint32_t     period)
{
    ret_code_t err_code;
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    uint32_t rtc_period;

    config.prescaler = RTC_WKUP_PRESCALER;   // 30.517 us counter period
    gRTC_SF_PERIOD = (period / WKUP_RESOLUTION_NS);

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
 *         Use the LED_TAG_Pin to see synchronization of tags
 *         on the oscilloscope if needed
 * */
static void rtcWakeUpTimerEventCallback(nrf_drv_rtc_int_type_t int_type)
{
    static uint32_t  gRTCtickCnt = 1;
    twr_info_t     *pTwrInfo = getTwrInfoPtr();
    static int    count = 0;

    if(gRTCtickCnt == gRTC_SF_PERIOD)    // 100 ms
    {
      nrf_gpio_pin_toggle(LED_RED);

      pTwrInfo->gRtcSFrameZeroCnt = nrf_drv_rtc_counter_get(&rtc);

      if(pTwrInfo)    //RTOS : pTwrInfo can be not allocated yet
      {
        nrf_gpio_pin_toggle(LED_RED);

        if(count == 0)
        {
            twr_configure_rtc_wakeup(pTwrInfo->env.sframePeriod_ns);
        }

        if(pTwrInfo->stationary && !pTwrInfo->stationary_imu )
        {
            pTwrInfo->stationary = false;
            count = pTwrInfo->env.pollMultFast;
        }

        count++;

        if( (pTwrInfo->stationary && (count >= pTwrInfo->env.pollMultSlow)) ||
            (!pTwrInfo->stationary && (count >= pTwrInfo->env.pollMultFast)) )
        {
            if(app.pollTask.Handle)    //RTOS : pollTask can be not started yet
            {
                osMutexWait(app.pollTask.MutexId, osWaitForever);

                if(osSignalSet(app.pollTask.Handle, app.pollTask.Signal) != osOK)
                {
                    error_handler(1, _Err_Signal_Bad);
                }
            }
            count = 0;
        }
      } // if(pTwrInfo)

      gRTCtickCnt = 0;
  } // if(gRTCtickCnt == gRTC_SF_PERIOD)    // 100 ms

    gRTCtickCnt++;
}


//-----------------------------------------------------------------------------
