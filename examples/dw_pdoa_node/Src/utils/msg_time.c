/*
 * @file     msg_time.c
 * @brief    used to calculate frames duration
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#include <msg_time.h>
#include <util.h>
#include "math.h"
#include "deca_param_types.h"

/* @brief Calculates the length of message in us; sy; dt
 *
 * This is a function of data rate, preamble length, PRF and message data length
 * */
void calculate_msg_time(msg_t *msg, msg_time_t *msg_time)
{
    double datalen_ns, phrlen_ns, preamblelen;
    int   sfdlen, tmp;

    tmp = msg->msg_len;
    datalen_ns = tmp*8.0 + 48.0*ceil(tmp*8.0/330.0);//number of bits + 48bits for each Reed-Solomon block (330bits or less)

    // pre-calculated : PHR length is 172308ns for 110k and 21539ns for 850k/6.81M
    // Refer to "Frame Format" of DW1000-Datasheet.pdf
    switch (msg->dataRate)
    {
        case(DWT_BR_110K) :
        {
            datalen_ns *= 8205.13f;        //length of data message in ns for data
            phrlen_ns    = (21*8205.13);//PHR header 110K 21bit*8205.13= 172308ns
            break;
        }
        case(DWT_BR_850K) :
        {
            datalen_ns *= 1025.64f;     //length of data message in ns for data
            phrlen_ns    = (21*1025.64);    //PHR header 850K 21bit*1025.64= 21539ns
            break;
        }
        case(DWT_BR_6M8) :
        {
            datalen_ns *= 128.21f;        //length of data message in ns for data
            phrlen_ns    = (21*1025.64);    //PHR header 6.8M 21bit*1025.64= 21539ns
            break;
        }
        default :
        {
            //ASSERT(0);					// TBD
            break;
        }
    }

    /* sfd and preamble are in symbols */

    // number of Symbols in SFD length
    // 64 Symbols for 110k (always)
    // 8 Symbols for 6.81M and 16 for 850k (can vary)
    sfdlen = dwnsSFDlen[msg->dataRate];     //deca_params_init.c must be included in project

    //number of Symbols in preamble sequence
    switch (msg->txPreambLength)
    {
        case DWT_PLEN_4096 : preamblelen = 4096.0f; break;
        case DWT_PLEN_2048 : preamblelen = 2048.0f; break;
        case DWT_PLEN_1536 : preamblelen = 1536.0f; break;
        case DWT_PLEN_1024 : preamblelen = 1024.0f; break;
        case DWT_PLEN_512  : preamblelen = 512.0f;  break;
        case DWT_PLEN_256  : preamblelen = 256.0f;  break;
        case DWT_PLEN_128  : preamblelen = 128.0f;  break;
        case DWT_PLEN_64   : preamblelen = 64.0f;   break;
        default            : //assert_param(0);    			// TBD   
        break;
    }

    //convert Synchronisation Header (SHR)=PLEN+SFD to us
    switch (msg->prf)
    {
        case DWT_PRF_16M : preamblelen = (sfdlen + preamblelen) * 0.99359f; break; //us
        case DWT_PRF_64M : preamblelen = (sfdlen + preamblelen) * 1.01763f; break; //us
        default          : //assert_param(0);      			// TBD                               
        break;
    }

    msg_time->preamble_us     = (uint32_t)(preamblelen);        //length of Preamble
    msg_time->phr_us         = (uint32_t)(phrlen_ns/1000);    //length of PHR
    msg_time->data_us         = (uint32_t)(datalen_ns/1000);    //length of Data
    msg_time->phrAndData_us = (uint32_t)((datalen_ns+phrlen_ns) /1000);

    msg_time->us = (uint32_t)(preamblelen + msg_time->phrAndData_us);//length of the whole frame

    /*
     * in the application Symbol time and Device time is used
     *
     * 1. Conversion of values to SYMBOL TIME: 1sy = 1.0256us
     * for wait4response timeouts you will need time in "Symbol" time:
     * tmp = (int)((preamblelen + msgdatalen) / 1.0256); //message length in "Symbol" time
     *
     * 2. Conversion of values to DEVICE TIME: 1dt = (uint64_t)((double)(1us/(double)DWT_TIME_UNITS)/1e6)
     *
     * */

    msg_time->sy = (uint32_t)util_us_to_sy(msg_time->us);       //length of the whole frame in Symbol time units
    msg_time->dt64 = util_us_to_dev_time(msg_time->us);         //length of the whole frame in device time, uint64_t

    msg_time->dt[0] = (uint8_t)(msg_time->dt64         &0xFF);  //length of the whole frame in device TimeStamp
    msg_time->dt[1] = (uint8_t)(msg_time->dt64>>8     &0xFF);
    msg_time->dt[2] = (uint8_t)(msg_time->dt64>>16     &0xFF);
    msg_time->dt[3] = (uint8_t)(msg_time->dt64>>24     &0xFF);
    msg_time->dt[4] = (uint8_t)(msg_time->dt64>>32 &0xFF);
}
