/*
 * @file      flush_task.c
 * @brief
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */
#include "dw_pdoa_node_common.h"
#include "circ_buf.h"
#include "usb_uart_rx.h"

#include "usb_uart_tx.h"

#include "port_platform.h"
#include "cmd_fn.h"

//-----------------------------------------------------------------------------
//     USB/UART report section
//
// Currently implemented using statically allocated Report.buf which shall be big enough for
// Usb2Spi top-level application, i.e. >4KB.
// The next available is 8K.

//the size is such long because of usb2spi top-level application
#define REPORT_BUFSIZE          0x1000  /**< the size of report buffer, must be 1<<N, i.e. 0x800,0x1000 etc*/
#define USB_UART_TX_TIMEOUT_MS  10      /**< the timeout in ms to output 64 bytes of data on 115200 bitrate */

static uint8_t ubuf[128]; /**< linear buffer, to transmit next chunk of data */

static struct
{
    dw_hal_lockTypeDef   lock;    /**< locking object  */
    struct
    {
        uint16_t   head;
        uint16_t   tail;
        uint8_t    buf[REPORT_BUFSIZE];    /**< Large USB/UART circular Tx buffer */
    }
    Report;                    /**< circular report buffer, data to transmit */
}
txHandle =
{
    .lock          = DW_HAL_NODE_UNLOCKED,
    .Report.head = 0,
    .Report.tail = 0
};


//-----------------------------------------------------------------------------
// Implementation

int reset_report_buf(void)
{
    __HAL_LOCK(&txHandle);
    txHandle.Report.head = txHandle.Report.tail = 0;
    __HAL_UNLOCK(&txHandle);
    return  _NO_ERR;
}

/* @fn         copy_tx_msg()
 * @brief     put message to circular report buffer
 *             it will be transmitted in background ASAP from flushing thread
 * @return    HAL_BUSY - can not do it now, wait for release
 *             HAL_ERROR- buffer overflow
 *             HAL_OK   - scheduled for transmission
 * */
static error_e copy_tx_msg(uint8_t *str, int len)
{
    error_e  ret = _NO_ERR;
    uint16_t head, tail;
    uint16_t size = sizeof(txHandle.Report.buf) / sizeof(txHandle.Report.buf[0]);

    /* add TX msg to circular buffer and increase circular buffer length */

    __HAL_LOCK(&txHandle); //"return HAL_BUSY;" if locked
 //   do
  //  {
   // 	__HAL_LOCK(&txHandle);osDelay (10);
  //  }while(txHandle.lock == DW_HAL_NODE_UNLOCKED);
    head = txHandle.Report.head;
    tail = txHandle.Report.tail;

    if (CIRC_SPACE(head, tail, size) > len)
    {
        while (len > 0)
        {
            txHandle.Report.buf[head] = *(str++);
            head = (head + 1) & (size - 1);
            len--;
        }

        txHandle.Report.head = head;
    }
    else
    {
        /* if packet can not fit, setup TX Buffer overflow ERROR and exit */
        error_handler(0, _Err_TxBuf_Overflow);
        ret = _Err_TxBuf_Overflow;
    }

    __HAL_UNLOCK(&txHandle);
    return ret;
}

/* @fn         port_tx_msg()
 * @brief     wrap for copy_tx_msg
 *             Puts message to circular report buffer
 *          and instructs output task to send it
 *
 * @return    see copy_tx_msg()
 * */
error_e port_tx_msg(uint8_t* str, int len)
{
    error_e ret;

    if(app.maxMsgLen<len)
    {
        app.maxMsgLen=len;
    }

    ret = copy_tx_msg(str, len);
	//deca_uart_transmit_len(str,len);
    if(app.flushTask.Handle ) //RTOS : usbFlushTask can be not started yet
    {
        osSignalSet(app.flushTask.Handle, app.flushTask.Signal);
    }

    return (ret);
}


//-----------------------------------------------------------------------------
//     USB/UART report : platform - dependent section
//                      can be in platform port file


/* @fn        flush_report_buff()
 * @brief    FLUSH should have higher priority than port_tx_msg()
 *             This shall be called periodically from process, which can not be locked,
 *             i.e. from independent high priority thread / timer etc.
 * */

error_e flush_report_buf(void)			// TBD - flush_report_buf is different for Tag so reimplement
{

    int         size = sizeof(txHandle.Report.buf) / sizeof(txHandle.Report.buf[0]);
    int         chunk;
	int head;
	int tail;
	int len;
    if(txHandle.lock==DW_HAL_NODE_LOCKED) return _NO_ERR;

    __HAL_LOCK(&txHandle); //"return HAL_BUSY;" if locked

transmit_again: 

    head = txHandle.Report.head;
    tail = txHandle.Report.tail;
    len = CIRC_CNT(head, tail, size);

    if(len > 0 )
    {
   
        /* copy MAX allowed length from circular buffer to linear buffer */
        chunk = MIN(sizeof(ubuf), len);

        for (int i = 0; i < chunk; i++)
        {
            ubuf[i] = txHandle.Report.buf[tail];
            tail = (tail + 1) & (size - 1);
        }

        len -= chunk;

        txHandle.Report.tail = tail;
		
        deca_uart_transmit_len(ubuf,chunk);


		
    }
	if(len>0)
	{
		//goto transmit_again;
	}

    __HAL_UNLOCK(&txHandle);
    return _NO_ERR;
}

//     END OF Report section

