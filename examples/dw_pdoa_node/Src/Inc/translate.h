/* @file    translate.h
 * @brief     translate DW1000 parameters from Deca to Human and from Human to Deca format
 *
 *            return translated value or (-1) as an error
 *
 * @author Decawave
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 */

#ifndef __TRANSLATE__H__
#define __TRANSLATE__H__ 1

#ifdef __cplusplus
 extern "C" {
#endif

/* Channel */
int chan_to_deca(int i);
int deca_to_chan(int i);

/* Bitrate */
int bitrate_to_deca(int i);
int deca_to_bitrate(int i);

/* PRF */
int prf_to_deca(int i);
int deca_to_prf(int i);

/* PAC */
int pac_to_deca(int i);
int deca_to_pac(int i);

/* PLEN */
int plen_to_deca(int i);
int deca_to_plen(int i);


#ifdef __cplusplus
}
#endif

#endif /* __TRANSLATE__H__ */
