/* @file    msg_time.h
 * @brief   used to calculate frames duration
 *
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __MSG_TIME__H__
#define __MSG_TIME__H__ 1

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

typedef struct {
    int     prf;                        //Deca define: e.g. DWT_PRF_16M
    int     dataRate;                    //Deca define: e.g. DWT_BR_110K
    int     txPreambLength;                //Deca define: e.g. DWT_PLEN_4096
    int     msg_len;
}msg_t;

typedef struct {
    uint32_t    preamble_us;
    uint32_t    phr_us;
    uint32_t    data_us;
    uint32_t    phrAndData_us;

    uint32_t    us;
    uint32_t    sy;
    uint64_t    dt64;
    uint8_t        dt[5];
}msg_time_t;

/* exported functions prototypes */
void calculate_msg_time(msg_t *msg, msg_time_t *msg_time);

#ifdef __cplusplus
}
#endif

#endif

