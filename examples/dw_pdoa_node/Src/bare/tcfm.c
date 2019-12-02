/*
 * @file       tcfm.c
 * @brief      process to run Test Continuous Frame Mode
 *
 *
 *    measure the power:
 *    Spectrum Analyser set:
 *    FREQ to be channel default e.g. 3.9936 GHz for channel 2
 *    SPAN to 1GHz
 *    SWEEP TIME 1s
 *    RBW and VBW 1MHz
 *    measure channel power
 *    measure peak power
 *
 *
 * @author     Decawave Software
 *
 * @attention
 *             Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *             All rights reserved.
 * */

#include "tcfm.h"
#include "msg_time.h"
#include "uwb_frames.h"

#define TEST_DISABLE_SMART_POWER     (1)

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
        0x25456585  //64M prf power
    }
};


/* IMPLEMETATION */

/*
 * @brief     init function initialises all run-time environment allocated by the process
 *             it will be executed once
 *
 * */
void tcfm_process_init(int chip)
{
    msg_t        msg;
    msg_time_t    msg_time;

    //define some test data for the tx buffer
    const uint8_t msg_data[] = "The quick brown fox jumps over the lazy dog";

    tcXm_configure_test_mode(chip);

    /* Setup Tx */

    msg.msg_len = FRAME_CTRL_AND_ADDRESS_S + 3 + FRAME_CRC;    //overal message is 14
    msg.prf = app.pConfig->dwt_config.prf;
    msg.txPreambLength = app.pConfig->dwt_config.txPreambLength;
    msg.dataRate = app.pConfig->dwt_config.dataRate;

    calculate_msg_time(&msg, &msg_time);

    /* tmp is sets a lag between frames in length of 125MHz clock cycles (quarter of 499.2 MHz units).
     * i.e. time from Tx end - to next - Tx start */
    uint32_t tmp = (uint32_t)(msg_time.us * 499.2) / 4;

    dwt_writetxdata(msg.msg_len, (uint8_t*)msg_data, 0);

    dwt_writetxfctrl(msg.msg_len, 0, 0);

    if (tmp > 124800)
    {
        dwt_configcontinuousframemode(8);                /**< will start tx back-to back with 0 lag delay */
    }
    else
    {
        dwt_configcontinuousframemode(124800-tmp);        /**< For frame less than 1ms will transmit 1 frame in 1ms */
    }

    dwt_starttx(0);
}


/*
 * @brief     run function implements continuous process functionality
 * */
void tcfm_process_run(void)
{
    /*do nothing*/
}


/*
 * @brief     stop function implements stop functionality if any
 *             which will be executed on reception of Stop command
 * */
void tcfm_process_terminate(void)
{
    port_stop_all_UWB();
}



void tcXm_configure_test_mode(int chip)
{
    int             result;
    uint32_t        power;
    dwt_txconfig_t    txconfig;
    dwt_config_t     *pdwCfg;

    port_stop_all_UWB();

    set_dw_spi_slow_rate(chip);

    result = dwt_initialise(DWT_LOADUCODE) ;

    if (DWT_SUCCESS != result)
    {
        error_handler(1, _ERR_TCFM );
        return;
    }

    dwt_setleds(3);
    dwt_setdblrxbuffmode(0);     //disable double RX buffer

    pdwCfg = &app.pConfig->dwt_config;

    dwt_configure(pdwCfg);        //configure the channel parameters

    power = txSpectrumConfig.txPwr[pdwCfg->prf - DWT_PRF_16M];

    if(app.pConfig->s.smartTxEn == 1)
    {
        dwt_setsmarttxpower(1);
    }
    else
    {
        power = (power & 0xff) ;
        power |= (power << 8) + (power << 16) + (power << 24);
        dwt_setsmarttxpower(0);
    }

    txconfig.power = power;    /**< If smart power is used, it will only apply when the frame length is < 1ms */
    txconfig.PGdly = txSpectrumConfig.PGdelay ;

    dwt_configuretxrf(&txconfig);    /**< configure the tx spectrum parameters (power and PG delay) */
}
