/* @file    config.h
 * @brief     supports NVM and bss configuration sections:
 *             defaultFConfig : section in NVM, where default parameters are saved and is not re-writabele.
 *                   FConfig : section in NVM, where current parameters are saved and this is re-writabele.
 *                 bssConfig : section in RAM, which is representing FConfig.
 *
 *    application on startup shall init_bssConfig() : this will copy FConfig -> bssConfig
 *    Accessing to variables is by pointer get_pbssConfig();
 *
 *    if application wants to re-write FConfig, use save_bssConfig(*newRamParametersBlock);
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_    1

#ifdef __cplusplus
 extern "C" {
#endif

#include "default_config.h"
#include "error.h"

void load_bssConfig(void);
param_block_t *get_pbssConfig(void);
error_e save_bssConfig(param_block_t *); /**< save to FConfig */
void restore_bssConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* _CONFIG_H_ */
