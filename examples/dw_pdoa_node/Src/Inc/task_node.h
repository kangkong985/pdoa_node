/*
 * @file node_task.h
 * @brief
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __TWR_TASK__H__
#define __TWR_TASK__H__ 1

#ifdef __cplusplus
 extern "C" {
#endif

extern void signal_to_pc_new_tag_discovered(uint64_t addr64);

void node_helper(void const *argument);
void node_terminate_tasks(void);
void suspend_node_tasks(void);

#ifdef __cplusplus
}
#endif

#endif /* __TWR_TASK__H__ */
