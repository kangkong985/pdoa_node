/*
 * @file  port.h
 * @brief port headers file to STM32F303 Node board for AOA project
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __PORT__H__
#define __PORT__H__ 1

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

//#include "usbd_cdc.h"
#include "deca_device_api.h"

#include "default_config.h"

#include "cmsis_os.h"

#include "error.h"

#include "circ_buf.h"

//-----------------------------------------------------------------------------


/* SPI/USART/USB buffers */
#define UART_RX_BUF_SIZE	0x100	/**< Read buffer for UART reception, shall be 1<<X */
#define USB_RX_BUF_SIZE		0x100	/**< Read buffer for USB reception, shall be 1<<X */
#define COM_RX_BUF_SIZE		USB_RX_BUF_SIZE /**< Communication RX buffer size */

#if (COM_RX_BUF_SIZE < CDC_DATA_FS_MAX_PACKET_SIZE)
#error "COM_RX_BUF_SIZE should be longer than CDC_DATA_FS_MAX_PACKET_SIZE"
#endif

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

//-----------------------------------------------------------------------------
// common macros

#ifndef SWAP
#define SWAP(a,b) {a^=b;b^=a;a^=b;}
#endif /* SWAP */

#ifndef MIN
#define MIN(a,b)    (((a) < (b)) ? (a) : (b))
#endif /* MIN */

#ifndef MAX
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#endif /* MAX */

#ifndef TRUE
#define TRUE  1
#endif /* TRUE */

#ifndef FALSE
#define FALSE  0
#endif /* FALSE */


#define MASK_40BIT            (0x00FFFFFFFFFFULL)  // DW1000 counter is 40 bits
#define MASK_TXDTS            (0x00FFFFFFFE00ULL)  //The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.

//-----------------------------------------------------------------------------
//    DW1000 description

typedef enum {
    DW_MASTER,
    DW_SLAVE
}dw_name_e;


/* description of spi interface to DW1000 chip */
typedef struct
{
    SPI_HandleTypeDef       *phspi;
    uint32_t                prescaler_slow;
    uint32_t                prescaler_fast;
    DMA_TypeDef             *dmaRx;
    uint32_t                channelRx;
    DMA_TypeDef             *dmaTx;
    uint32_t                channelTx;
    uint32_t                csPin;
    GPIO_TypeDef            *csPort;
    __IO HAL_LockTypeDef    Lock;
    __IO uint32_t           TxComplete;
    __IO uint32_t           RxComplete;
    uint8_t                 *pBuf;
}spi_handle_t;


/* description of connection to the DW1000 chip */
typedef struct
{
    uint16_t        irqPin;
    GPIO_TypeDef    *irqPort;
    IRQn_Type       irqN;
    uint16_t        rstPin;
    GPIO_TypeDef    *rstPort;
    uint16_t        wakeUpPin;
    GPIO_TypeDef    *wakeUpPort;
    IRQn_Type       rstIrqN;
    spi_handle_t    *pSpi;
}dw_t;


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
    mTCWM,        /**< Transmit Continuous Wave Mode mode */
    mTCFM        /**< Transmit Continuous Frame Mode mode */
}mode_e;


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

    task_signal_t    imuTask;           /* Tag/Node */

}__packed
app_t;

/****************************************************************************//**
 *
 * */

extern app_t app;

extern RTC_HandleTypeDef hrtc;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_memtomem_dma1_channel1;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern IWDG_HandleTypeDef hiwdg;

extern const dw_t *pDwMaster;
extern const dw_t *pDwSlave;

extern const dw_t dw_chip_A;
extern const dw_t dw_chip_B;
/****************************************************************************//**
 * port functions prototypes
 *
 * */
void set_SPI_master(void);
void set_SPI_slave(void);
void wakeup_dw1000(dw_name_e chip);
void set_dw_spi_fast_rate(dw_name_e chip);
void set_dw_spi_slow_rate(dw_name_e chip);

void port_wakeup_dw1000(dw_name_e chip);
void error_handler(int block, error_e err);

void enable_dw1000_irq(void);
void disable_dw1000_irq(void);
void reset_DW1000(dw_name_e chip);

void port_stop_all_UWB(void);

/* mutex's */
decaIrqStatus_t decamutexon(void);
void decamutexoff(decaIrqStatus_t s);

/* Time section */
void start_timer(volatile uint32_t * p_timestamp);
bool check_timer(uint32_t timestamp, uint32_t time);
void Sleep( volatile uint32_t );

//for deca_sleep()
#define portGetTickCount()    HAL_GetTick()

/* SYNC section */
void port_set_syncenable(int enable);
void port_set_sync(int enable);
void port_set_syncclear(int enable);

void port_disable_wake_init_dw(void);

void port_reinit_dw_chips(void);

void port_uart_rx_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __PORT__H__ */
