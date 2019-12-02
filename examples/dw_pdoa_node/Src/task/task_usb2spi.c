/*
 * @file     task_usb2spi.c
 * @brief    usb2spi implementation
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#include <error.h>
#include <task_usb2spi.h>
#include <task_flush.h>
#include <usb_uart_rx.h>
#include <usb2spi.h>

//-----------------------------------------------------------------------------
// for Usb2Spi the output should be set much faster than on usual operation
#define FLUSH_PERIOD_USB2SPI_MS    (1)

/* @brief
 *      Kill all tasks and timers related to usb2spi if any
 *      DW1000's RX and IRQ shall be switched off before task termination,
 *      that IRQ will not produce unexpected Signal
 * */
void usb2spi_terminate_tasks(void)
{
    if(app.usb2spiTask.Handle)
    {
        osMutexWait(app.usb2spiTask.MutexId, osWaitForever);

        if(osThreadTerminate(app.usb2spiTask.Handle) == osOK)
        {
            usb2spi_process_terminate();

            osMutexDelete(app.usb2spiTask.MutexId);
            app.usb2spiTask.Handle = NULL;
        }
        else
        {
            error_handler(1, _ERR_Cannot_Delete_usb2spiTask);
        }
    }
}


/* @fn         StartUsb2SpiTask
 * @brief     this starts the usb2spi functionality.
 *
 *             Note: Previous tasks which can call shared resources must be killed.
 *            This task needs the RAM size of at least usb2spi_t
 *
 * */
void StartUsb2SpiTask(void const *argument)
{
    error_e        tmp;
    dw_name_e    chip;

    chip = (*((uint32_t*)argument) == (uint32_t)DW_A)?(DW_A):(DW_B);

    port_disable_wake_init_dw();

    osMutexDef(usb2spiTaskMutex);
    app.usb2spiTask.MutexId = osMutexCreate(osMutex(usb2spiTaskMutex));

    tmp = usb2spi_process_init(chip);

    if(tmp != _NO_ERR)
    {
        error_handler(1, tmp);
    }

    set_FlushPeriod(FLUSH_PERIOD_USB2SPI_MS);

    while(1)
    {
        osMutexRelease(app.usb2spiTask.MutexId);

        osSignalWait(app.usb2spiTask.Signal , osWaitForever);

        osMutexWait(app.usb2spiTask.MutexId, 0);

        usb2spi_process_run();    //app.local_buff:app.local_buff_length has a Usb2Spi protocol sequence
    }
}

