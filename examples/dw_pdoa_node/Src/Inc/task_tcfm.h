/*
 * @file tcfm_task.h
 *
 * @brief  header file for tcfm_task.c
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __INC_TCFM_TASK_H_
#define __INC_TCFM_TASK_H_    1

#ifdef __cplusplus
 extern "C" {
#endif

void StartTcfmTask(void const * arg);
void tcfm_terminate_tasks(void);

#ifdef __cplusplus
}
#endif

#endif /* __INC_TCFM_TASK_H_ */
