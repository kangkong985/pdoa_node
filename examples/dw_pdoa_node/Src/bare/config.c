/*
 * @file       config.c
 *
 * @brief      supports NVM and bss configuration sections:
 *             defaultFConfig : section in RAM, where default parameters are saved and is not re-writabele.
 *              FCONFIG_ADDR  : section in NVM, where current parameters are saved and this is re-writabele.
 *                 bssConfig  : section in RAM, which is representing config data exist in FCONFIG_ADDR.
 *
 *             application on startup shall init_bssConfig() : this will copy data from FCONFIG_ADDR -> bssConfig
 *             Accessing to variables is by pointer get_pbssConfig();
 *
 *             if application wants to re-write data in FCONFIG_ADDR, use save_bssConfig(*newRamParametersBlock);
 *
 *             NOTE: The code is very MCU dependent and save will work with nRF52840 only
 *
 * @author     Decawave Software
 *
 * @attention  Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *             All rights reserved.
 *
 * @modified   2018 PathPartner
 */

#include <stdint.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_version.h"
#include "version.h"

#include "config.h"
#include "nrf_nvmc.h"

//------------------------------------------------------------------------------

const param_block_t config_data = DEFAULT_CONFIG;

/* Section ".default_config" is defined in a linker file */
const param_block_t defaultFConfig __attribute__((section(".default_config"))) __attribute__((aligned(0x100))) = DEFAULT_CONFIG; // TBD FCONFIG_SIZE - 0x100

/* Section ".fconfig" is defined in a linker file */
const param_block_t FConfig __attribute__((section(".fconfig"))) __attribute__((aligned(0x100))) = DEFAULT_CONFIG; // TBD FCONFIG_SIZE - 0x100

/* run-time parameters block.
 *
 * This is the RAM image of the FConfig .
 *
 * Accessible from application by app.pConfig pointer after init_bssConfig()
 *
 * */
static param_block_t bssConfig __attribute__((section(".bss"))) __attribute__((aligned(0x100))); // TBD

/*
 * (re-writable block) @ of Page 255, max 2 Kbytes
 *
 * 
 */
#define ADDR_FLASH_PAGE_127                ((uint32_t)0x0006F000UL)

#define FCONFIG_START_ADDR                 (ADDR_FLASH_PAGE_127)
#define FCONFIG_ADDR                       (ADDR_FLASH_PAGE_127 + 0x100)
#define FCONFIG_END_ADDR                   (FCONFIG_ADDR + FCONFIG_SIZE)


void init_nvm_appdata(void)
{

  uint32_t *pFConfig = FCONFIG_ADDR;

  nrf_nvmc_page_erase((uint32_t)&defaultFConfig); // Erase a page in RAM
    // Write consecutive bytes to RAM.
  nrf_nvmc_write_bytes((uint32_t)&defaultFConfig, &config_data, FCONFIG_SIZE);

  /* Copy default config data to FCONFIG_ADDR*/
  if(*pFConfig == 0xFFFFFFFF)
  {
    nrf_nvmc_write_bytes(pFConfig,  &defaultFConfig, FCONFIG_SIZE);
  }
}

//------------------------------------------------------------------------------
// Implementation

/*
 * @brief get pointer to run-time bss param_block_t block
 *
 * */
param_block_t *get_pbssConfig(void)
{
    return(&bssConfig);
}


/* @fn      load_bssConfig
 * @brief   copy parameters from NVM to RAM structure.
 *
 *          assumes that memory model in the MCU of .text and .bss are the same
 * */
void load_bssConfig(void)
{
   memcpy(&bssConfig, FCONFIG_ADDR, sizeof(bssConfig));
}

/* @fn      restore_bssConfig
 * @brief   copy parameters from default RAM section to RAM structure.
 *
 *          assumes that memory model in the MCU of .text and .bss are the same
 * */
void restore_bssConfig(void)
{
    memcpy(&bssConfig, &defaultFConfig, sizeof(bssConfig));
}

/* @brief    save pNewRamParametersBlock to FCONFIG_ADDR
 * @return  _NO_ERR for success and error_e code otherwise
 * */
error_e save_bssConfig(param_block_t * pNewRamParametersBlock)
{
  uint32_t * pbuf = (uint32_t *) pNewRamParametersBlock;

  nrf_nvmc_page_erase((uint32_t)FCONFIG_START_ADDR); // Erase a page in flash
    // Write consecutive bytes to flash.
  nrf_nvmc_write_bytes(FCONFIG_ADDR, pNewRamParametersBlock, FCONFIG_SIZE);

  return (_NO_ERR);
}
