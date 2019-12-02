/*
 *  @file    usb_uart_rx.h
 *  @brief
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __INC_USB_UART_RX_H_
#define __INC_USB_UART_RX_H_    1

#ifdef __cplusplus
 extern "C" {
#endif

// typedef enum {
//     NO_DATA = 0,
//     DATA_READY,
//     COMMAND_READY,
//     DATA_SEND,
//     DATA_FLUSH
// }usb_data_e;

typedef enum {
     NO_DATA = 0,
     DATA_READY,
     COMMAND_READY,
     DATA_SEND,
     DATA_FLUSH
}uart_data_e;

uart_data_e usb_uart_rx(void);

#ifdef __cplusplus
}
#endif

#endif /* __INC_USB_UART_TX_H_ */
