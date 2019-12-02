/*
 *  @file     task_uart2spi.c
 *  @brirf    uart2spi  implementation
 *
 *  
 *
 *
 *
 *
 */

#include "dw_pdoa_node_common.h"
#include "port_platform.h"

#include <error.h>
#include <task_uart2spi.h>
//#include <uart2spi.h>

/* @brief
 *  Kill all tasks and timers related to uart2spi if any
 *
 */

void uart2spi_terminate_tasks(void) {
    if (app.uart2spiTask.Handle)
    {
        osMutexWait(app.uart2spiTask.MutexId, osWaitForever);
        taskENTER_CRITICAL();
        if (osThreadTerminate(app.uart2spiTask.Handle) == osOK)
        {
            uart2spi_process_terminate();

            if (app.uart2spiTask.MutexId)
            {
                osMutexDelete(app.uart2spiTask.MutexId);
            }
            app.uart2spiTask.Handle = NULL;
            app.uart2spiTask.MutexId = NULL;
        }
        else
        {
            error_handler(1, _ERR_Cannot_Delete_uart2spiTask);
        }
        taskEXIT_CRITICAL();
    }
}

/* @          StartUart2SpiTask
 * @brief
 *            This starts the uart2spi functionality
 *
 *            Note: Previous task which can call shared resources must be killed.
 *            This task needs the RAM size of at least uart2spi_t
 *
 */

void StartUart2SpiTask(void const * argument)
{
    error_e tmp;
    dw_name_e chip;

    chip = (*((uint32_t*)argument) == (uint32_t)DW_MASTER)?(DW_MASTER):(DW_SLAVE);

    port_disable_wake_init_dw();

    osMutexDef(u2sMutex);
    app.uart2spiTask.MutexId = osMutexCreate((osMutex(u2sMutex)));

    tmp = uart2spi_process_init(chip);

    if (tmp != _NO_ERR)
    {
        error_handler(1, tmp);
    }

    while(1)
    {
        osMutexRelease(app.uart2spiTask.MutexId);

        osSignalWait(app.uart2spiTask.Signal, osWaitForever);

        osMutexWait(app.uart2spiTask.MutexId, 0);

        uart2spi_process_run();
    }
}