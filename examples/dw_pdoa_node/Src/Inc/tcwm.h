/*
 * @file     tcwm.h
 * @brief      header file for bare-metal tcwm.c
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __TCWM_H_
#define __TCWM_H_    1

#ifdef __cplusplus
 extern "C" {
#endif


void tcwm_process_init(int chip);
void tcwm_process_run(void);
void tcwm_process_terminate(void);

extern void tcXm_configure_test_mode(int chip);

#ifdef __cplusplus
}
#endif

#endif /* __TCWM_H_ */
