/*! ------------------------------------------------------------------------------------------------------------------
 * @file    dw_pdoa_node_common.h
 * @brief   Defines AoA Node related Common Macros, structures, function definitions
 *
 * @attention
 *
 * Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Deepa Gopinath
 */

#ifndef __DW_AOA_NODE_COMMON__H__
#define __DW_AOA_NODE_COMMON__H__

#ifdef __cplusplus
 extern "C" {
#endif

//#include "nrf_drv_usbd.h"
#include "nrf_drv_spi.h"

#include "default_config.h"
#include "error.h"
//#include "deca_uart.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "event_groups.h"

#include "cmsis_os.h"



/* ENABLE_USB_PRINT Macro is uncommented then Segger RTT Print will be enabled*/
//#define ENABLE_USB_PRINT

#define DW1000_CLK_Pin     16 //3
#define DW1000_MISO_Pin    18 //28
#define DW1000_MOSI_Pin    20 //4

#define DW1000_IRQ_A_Pin   19 //23
#define DW1000_RST_A_Pin   24 //22
#define DW1000_WUP_A_Pin   23 //32
#define DW1000_CS_A_Pin    17 //29

#define DW1000_IRQ_B_Pin   13 //42
#define DW1000_RST_B_Pin   14 //34
#define DW1000_CS_B_Pin    11 //33
#define DW1000_WUP_B_Pin   12 //35

//#define DW1000_IRQ_B_Pin   19 //23
//#define DW1000_RST_B_Pin   24 //22
//#define DW1000_WUP_B_Pin   23 //32
//#define DW1000_CS_B_Pin    17 //29

//#define DW1000_IRQ_A_Pin   13 //42
//#define DW1000_RST_A_Pin   14 //34
//#define DW1000_CS_A_Pin    11 //33
//#define DW1000_WUP_A_Pin   12 //35

#define DW1000_SYNC_EN_Pin    10  //45
#define DW1000_SYNC_CLR_Pin   9   //46
#define DW1000_SYNC_Pin       8   //473


#define DW1000_POWER_EN_Pin   22

#define LPS22HB_BR_CS_Pin     11

#define LSM6DSL_I2C_ADDR      0b1101011
#define LIS2MDL_I2C_ADDR      0b0011110
#define LPS22HB_I2C_ADDR      0b1011100

#define GPIO_DW_POWER_EN       LED_1
#define GPIO_DW_WUP_A          LED_2
#define GPIO_DW_RST_A          LED_3
#define GPIO_LED_RUN           LED_4
//default
#define LED_STATIONARY LED_1
#define LED_NODE       LED_2
#define LED_USB        LED_4
#define LED_ERROR      LED_3


#define UART_RX_BUF_SIZE	0x100	/**< Read buffer for UART reception, shall be 1<<X */
#define UART_TX_BUF_SIZE	0x100	/**< Transmit buffer for UART reception, shall be 1<<X */
#define USB_RX_BUF_SIZE		0x100	/**< Read buffer for USB reception, shall be 1<<X */
#define COM_RX_BUF_SIZE		USB_RX_BUF_SIZE /**< Communication RX buffer size */

#define NRF52_FLASH_CONFIG_ADDR     0x70000

#define NORMAL_USE 1
#define USE_ULTRASONIC 1

#if (COM_RX_BUF_SIZE < 64)
#error "COM_RX_BUF_SIZE should be longer than CDC_DATA_FS_MAX_PACKET_SIZE"
#endif


/* System mode of operation. used to
 *
 * 1. indicate in which mode of operation system is running
 * 2. configure the access rights to command handler in control mode
 * */
typedef enum {
    mANY = 0,    /**< Used only for Commands: indicates the command can be executed in any modes below */
    mIDLE,        /**< IDLE mode */
    mTWR,        /**< TWR (active) mode */
    mUSB2SPI,    /**< USB2SPI mode */
    mUART2SPI,   /**< UART2SPI mode*/
    mTCWM,        /**< Transmit Continuous Wave Mode mode */
    mTCFM        /**< Transmit Continuous Frame Mode mode */
}mode_e;

/* events to start/stop tasks : event group */
enum{
    Ev_Tag_Task         = 0x08,
    Ev_Node_A_Task      = 0x10,
    Ev_Node_B_Task      = 0x20,
    Ev_Tcfm_A_Task      = 0x40,
    Ev_Tcfm_B_Task      = 0x80,
    Ev_Tcwm_A_Task      = 0x100,
    Ev_Tcwm_B_Task      = 0x200,
    Ev_Usb2spi_A_Task	= 0x400,
    Ev_Usb2spi_B_Task   = 0x800,
    Ev_Stop_All 		= 0x1000
};

 /* Application tasks handles & corresponded signals structure */
 typedef struct
 {
     osThreadId Handle;     /* Tasks handler */
     osMutexId  MutexId;    /* Tasks mutex */
     int32_t    Signal;     /* Tasks signal */
 }task_signal_t;

/* Application's global parameters structure */
typedef struct
{
    param_block_t   *pConfig;       /**< Current configuration */
    mode_e          mode;           /**< Information: handle the current "mode" of operation */
    int             lastErrorCode;  /**< Saves the error code in the error_handler() function */
    int             maxMsgLen;      /**< See the longest string size to optimize the MAX_STR_SIZE */

    /* USB / CTRL */
    enum {
        USB_DISCONNECTED,
        USB_PLUGGED,
        USB_CONNECTED,
        USB_CONFIGURED,
        USB_UNPLUGGED
    }
    usbState;                                        /**< USB connect state */

    struct
    {
    	uint8_t	   tmpRx;
        int16_t    head;
        int16_t    tail;
        uint8_t    buf[UART_RX_BUF_SIZE];
    }uartRx;                                        /**< circular buffer RX from USART */

    struct
    {
        int16_t    head;
        int16_t    tail;
        uint8_t    buf[USB_RX_BUF_SIZE];
    }usbRx;                                         /**< circular buffer RX from USB */

    uint16_t        local_buff_length;              /**< from usb_uart_rx parser to application */
    uint8_t         local_buff[COM_RX_BUF_SIZE];    /**< for RX from USB/USART */

    /* Tasks section */
    EventGroupHandle_t xStartTaskEvent;     /**< This event group will pass activation to tasks to start */

    osMailQId         (rxPcktPool_q_id);    /**< Mail queue ID for Twr processes: FreeRTOS does not free the resources */

    //defaultTask is always running and is not accepting signals

    task_signal_t    ctrlTask;          /* usb/uart RX: Control task */
    task_signal_t    flushTask;         /* usb/uart TX: Flush task */

    /* top-level tasks for TWR */
    task_signal_t    rxTask;            /* Tag/Node */
    task_signal_t    calcTask;          /* Node only */

    /* top-level tasks for special modes */
    task_signal_t    usb2spiTask;       /* USB2SPI top-level application */
    task_signal_t    tcfmTask;          /* TCFM top-level application */
    task_signal_t    tcwmTask;          /* TCWM top-level application */
    task_signal_t    uart2spiTask;      /* UART2SPI application*/

    task_signal_t    imuTask;           /* Tag/Node */


}__attribute__((packed))
app_t;

extern app_t app;

void error_handler(int block, error_e err);
//int8_t usb_data_receive();



#ifdef __cplusplus
}
#endif

#endif /* __DW_AOA_NODE_COMMON__H__ */
