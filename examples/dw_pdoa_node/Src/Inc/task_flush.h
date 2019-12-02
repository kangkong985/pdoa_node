/*
 * @file    task_flush.h
 * @brief
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __FLUSH_TASK__H__
#define __FLUSH_TASK__H__ 1

#ifdef __cplusplus
 extern "C" {
#endif

//#include "port.h"
#include "port_platform.h"
#include "dw_pdoa_node_common.h"

#define FLUSH_PERIOD_DEFAULT_MS    50

void FlushTask(void const * argument);
void reset_FlushTask(void);
void set_FlushPeriod(int ms);

#ifdef __cplusplus
}
#endif

#endif /* __FLUSH_TASK__H__ */
