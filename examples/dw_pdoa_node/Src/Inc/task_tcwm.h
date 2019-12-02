/*
 * @file task_tcwm.h
 *
 * @brief  header file for task_tcwm.c
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __INC_TCWM_TASK_H_
#define __INC_TCWM_TASK_H_    1

#ifdef __cplusplus
 extern "C" {
#endif

void StartTcwmTask(void const * arg);
void tcwm_terminate_tasks(void);

#ifdef __cplusplus
}
#endif

#endif /* __INC_TCWM_TASK_H_ */
