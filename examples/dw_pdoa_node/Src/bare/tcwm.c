/*
 * @file     tcwm.c
 * @brief      process to run Test Continuous Wave Mode
 * @author     Decawave Software
 *
 * @attention
 *             Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *          All rights reserved.
 * */

#include "tcwm.h"
#include "port.h"

/* IMPLEMETATION */

/*
 * @brief     init function initialises all run-time environment allocated by the process
 *             it will be executed once
 * */
void tcwm_process_init(int chip)
{

    tcXm_configure_test_mode(chip);

    dwt_configcwmode(app.pConfig->dwt_config.chan);

}


/*
 * @brief     run function implements continuous process functionality
 * */
void tcwm_process_run(void)
{
    /*do nothing*/
}


/*
 * @brief     stop function implements stop functionality if any
 *             which will be executed on reception of Stop command
 * */
void tcwm_process_terminate(void)
{
    port_stop_all_UWB();
}

