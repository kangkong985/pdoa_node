/* @file    deca_translate.c
 * @brief     translate DW1000 parameters from Deca to Human and from Human to Deca format
 *
 *            return translated value or (-1) if out of allowed range
 *
 * @author Decawave
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 */

#include "deca_device_api.h"


/* Channel */
int chan_to_deca(int i)
{
    switch (i)
    {
    case 1 :
        return 1;
    case 2 :
        return 2;
    case 3 :
        return 3;
    case 5 :
        return 5;
    default :
        return -1;
    }
}

int deca_to_chan(int i)
{
    return(chan_to_deca(i));
}


/* Bitrate */
int bitrate_to_deca(int i)
{
    switch (i)
    {
    case 110 :
        return DWT_BR_110K;
    case 850 :
        return DWT_BR_850K;
    case 6810 :
        return DWT_BR_6M8;
    default :
        return -1;
    }
}

int deca_to_bitrate(int i)
{
    switch (i)
    {
    case DWT_BR_110K :
        return 110;
    case DWT_BR_850K :
        return 850;
    case DWT_BR_6M8 :
        return 6810;
    default :
        return -1;
    }
}


/* PRF */
int prf_to_deca(int i)
{
    switch (i)
    {
    case 16 :
        return DWT_PRF_16M;
    case 64 :
        return DWT_PRF_64M;
    default :
        return -1;
    }
}


int deca_to_prf(int i)
{
    switch (i)
    {
    case  DWT_PRF_16M:
        return 16;
    case  DWT_PRF_64M:
        return 64;
    default :
        return -1;
    }
}


/* PAC */
int pac_to_deca(int i)
{
    switch (i)
    {
    case 8 :
        return DWT_PAC8;
    case 16 :
        return DWT_PAC16;
    case 32 :
        return DWT_PAC32;
    case 64 :
        return DWT_PAC64;
    default :
        return -1;
    }
}

int deca_to_pac(int i)
{
    switch (i)
    {
    case DWT_PAC8 :
        return 8;
    case DWT_PAC16 :
        return 16;
    case DWT_PAC32 :
        return 32;
    case DWT_PAC64 :
        return 64;
    default :
        return -1;
    }
}


/* PLEN */
int plen_to_deca(int i)
{
    switch (i)
    {
//    case 4096 :
//        return DWT_PLEN_4096;
    case 2048 :
        return DWT_PLEN_2048;
    case 1536 :
        return DWT_PLEN_1536;
    case 1024 :
        return DWT_PLEN_1024;
    case 512 :
        return DWT_PLEN_512;
    case 256 :
        return DWT_PLEN_256;
    case 128 :
        return DWT_PLEN_128;
    case 64 :
        return DWT_PLEN_64;
    default :
        return -1;
    }
}

int deca_to_plen(int i)
{
    switch (i)
    {
//    case DWT_PLEN_4096 :
//        return 4096;
    case DWT_PLEN_2048 :
        return 2048;
    case DWT_PLEN_1536 :
        return 1536;
    case DWT_PLEN_1024 :
        return 1024;
    case DWT_PLEN_512 :
        return 512;
    case DWT_PLEN_256 :
        return 256;
    case DWT_PLEN_128 :
        return 128;
    case DWT_PLEN_64 :
        return 64;
    default :
        return -1;
    }
}

/* END of translate */
