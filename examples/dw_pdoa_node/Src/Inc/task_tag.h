/*!----------------------------------------------------------------------------
 * @file    header for task_tag.c
 * @brief   DecaWave Application Layer
 *          RTOS tag implementation
 *
 * @attention
 *
 * Copyright 2016-2017 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author
 *
 * DecaWave
 */

#ifdef __TWR_TASK__H__
#define __TWR_TASK__H__ 1

#ifdef __cplusplus
 extern "C" {
#endif


 typedef struct {
    uint16_t    addr;
    uint16_t    node_addr;
    uint8_t     rNum;
    int16_t     x_cm;
    int16_t     y_cm;
    int16_t     clkOffset_pphm;
 }twr_res_ext_t;

#include "tag.h"			// TBD

void tag_helper(void const *argument);
void tag_terminate_tasks(void);

#ifdef __cplusplus
}
#endif

#endif /* __TWR_TASK__H__ */
