/*
 * @file     tcfm.h
 * @brief      header file for bare-metal tcfm.c
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __TCFM_H_
#define __TCFM_H_    1

#ifdef __cplusplus
 extern "C" {
#endif

void tcfm_process_init(int chip);
void tcfm_process_run(void);
void tcfm_process_terminate(void);

void tcXm_configure_test_mode(int chip);

#ifdef __cplusplus
}
#endif

#endif /* __TCFM_H_ */
