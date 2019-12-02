/*
 * @file  ctrl_task.c
 * @brief
  *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */
#include <cmd.h>
#include <usb_uart_rx.h>
#include <usb_uart_tx.h>

#include <cmsis_os.h>
#include "dw_pdoa_node_common.h"

/* @fn      CtrlTask
 * @brief   this is a core Control task (RX).
 *          this task is activated on the startup
 *          there 2 sources of control data: Uart and Usb.
 *
 * */
void CtrlTask(void const * arg)
{
    uart_data_e res;
    uint8_t str[32] = {0};
    uint8_t cr;
    unsigned int i = 0;

    while(1)
    {
        //printf("wait for uart msg.\r\n");nrf_gpio_pin_clear(31);
        osSignalWait(app.ctrlTask.Signal, osWaitForever);   //signal from UART that some data has been received
		//nrf_gpio_pin_set(31);
        /*mutex if uart2spiTask using the app.local_buff*/
        res = usb_uart_rx();    /**<processes usb/uart input:
                                    copy the input to the app.local_buff[ local_buff_length ]
                                   for future processing */
        
 //       taskEXIT_CRITICAL();

      //  printf("rcv: res=%d\r\n",res);
        if (res == COMMAND_READY)
        {
            int len = MIN(app.local_buff_length - 1, (sizeof(app.local_buff) - 1));
            app.local_buff[len + 1] = 0;
            command_parser((char *)app.local_buff);
			//printf("cmd:%s\r\n",app.local_buff);
        }
        else if(res == DATA_READY)
        {
            if (app.uart2spiTask.Handle)
            {
                osSignalSet(app.uart2spiTask.Handle, app.uart2spiTask.Signal);
            }
        }
    }
}
