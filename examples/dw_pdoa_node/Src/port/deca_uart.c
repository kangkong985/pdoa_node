/*! ----------------------------------------------------------------------------
 * @file    deca_uart.c
 * @brief   HW specific definitions and functions for UART Interface
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Deepa Gopinath
 */

#include "app_uart.h"
#include "nrf_uart.h"
#include "port_platform.h"
#include "dw_pdoa_node_common.h"
#include "circ_buf.h"

//#define UART_TX_BUF_SIZE 256 /**< UART TX buffer size. */
//#define UART_RX_BUF_SIZE 256 /**< UART RX buffer size. */

extern EventBits_t uxBits;

/******************************************************************************
 *
 *                              APP global variables
 *
 ******************************************************************************/
uint8_t rx_buf[256] = {0}; // RX_BUF used for store received data from UART.
char software_ver[30] = "yREK1101 COMMAND UART 1.1";

extern enum { STAND_ALONE,
  UART_TO_SPI,
  UART_COMMAND
} application_mode;

void UartTask(void const * arg);

/******************************************************************************
 *
 *                              Uart Configuration
 *
 ******************************************************************************/

/* @fn  peripherals_init
 *
 * @param[in] void
 * */
void peripherals_init_uart(void) {
  /* Set application mode as stand alone */
  //    application_mode=STAND_ALONE;

  /* configure systick to 1ms */
  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 1000);

  /* Setup DW1000 IRQ pin */
  //    nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL);     //irq

  /* Function for initializing the UART module. */
  deca_uart_init();

  /**/
osThreadDef(UartTask, UartTask, osPriorityNormal, 0, 128);
osThreadCreate(osThread(UartTask), NULL);
}

/* @fn  uart_error_handle
 *
 * @param[in] void
 * */
void deca_uart_error_handle(app_uart_evt_t *p_event) {
  if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR) {
    APP_ERROR_HANDLER(p_event->data.error_communication);
  } else if (p_event->evt_type == APP_UART_FIFO_ERROR) {
    APP_ERROR_HANDLER(p_event->data.error_code);
  }
}

/* @fn  deca_uart_init
 *
 * @brief Function for initializing the UART module.
 *
 * @param[in] void
 * */
void deca_uart_init(void) {
  uint32_t err_code;
  const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200};//UART_BAUDRATE_BAUDRATE_Baud921600 UART_BAUDRATE_BAUDRATE_Baud115200

  APP_UART_FIFO_INIT(&comm_params,
      UART_RX_BUF_SIZE,
      UART_TX_BUF_SIZE,
      deca_uart_error_handle,
      APP_IRQ_PRIORITY_LOW,
      err_code);
  nrf_gpio_cfg_input(RX_PIN_NUMBER, NRF_GPIO_PIN_PULLUP);
  UNUSED_VARIABLE(err_code); /* TBD */
}
/* @fn  deca_uart_transmit
 *
 * @brief Function for transmitting data on UART
 *
 * @param[in] ptr Pointer is contain base address of data.
 * */
void deca_uart_transmit(char *ptr) {
  uint32_t bit = 0;
  for (bit = 0; ptr[bit] != '\0'; bit++) {
    nrf_delay_ms(10);
    while (app_uart_put(ptr[bit]) != NRF_SUCCESS)
      ;
  }
  while (app_uart_put('\n') != NRF_SUCCESS)
    ;
  while (app_uart_put('\r') != NRF_SUCCESS)
    ;
}
void deca_uart_transmit_len(char *ptr,int len) {

  osMutexRelease(app.flushTask.MutexId);

  uint32_t bit = 0;
  for (bit = 0; bit<len; bit++) {
//    nrf_delay_ms(10);
//    osDelay(3);
    while (app_uart_put(ptr[bit]) != NRF_SUCCESS)
      ;
  }
   osMutexRelease(app.flushTask.MutexId);
}

/* @fn  deca_uart_receive
 *
 * @brief Function for receive data from UART and store into rx_buf
 *        (global array).
 *
 * @param[in] void
 * */
void deca_uart_receive(void) {
  uint32_t index = 0;
  uint32_t err_code;
  while (1) {
    while (!(nrf_uart_event_check(NRF_UART0, NRF_UART_EVENT_RXDRDY)))
      ;
    err_code = app_uart_get(&rx_buf[index]);
    if (rx_buf[index] == '\r') {
      while (app_uart_put('\n') != NRF_SUCCESS)
        ;
      while (app_uart_put('\r') != NRF_SUCCESS)
        ;
      rx_buf[index] = '\0';
      break;
    } else
      while (app_uart_put(rx_buf[index]) != NRF_SUCCESS)
        ;
    index++;
  }
  UNUSED_VARIABLE(err_code); /* TBD */
}
//
extern uint8_t g_rx_tick; 
void UartTask(void const * arg)
{
	uint8_t rxShdw,current_tick,decr;
	int head, tail, size,len,dat_tmp;
    while(1)
    {
    	current_tick=portGetTickCount()&0x00ff;
		if(g_rx_tick)
		{
			// Use the LSB of the sleep timer 
			  decr =current_tick - rxShdw;

			  if (g_rx_tick > decr)
			  {
			    g_rx_tick-= decr;
			  }
			  else
			  {
			    g_rx_tick = 0;
			  }			
		}
		rxShdw =current_tick;
		len=get_fifo_length();
		//printf("g_rx_tick:%d,len=%d\r\n",g_rx_tick,len);
	    if (len && !g_rx_tick)
	    {
			 
            	head = app.uartRx.head;
				tail = app.uartRx.tail;
				size = sizeof(app.uartRx.buf);

				if (CIRC_SPACE(head, tail, size) > len)
				{
					for(int i = 0; i<len; i++)
					{
						app_uart_get(&dat_tmp);
						app.uartRx.buf[head] =dat_tmp;
						head = (head + 1) & (size - 1);
					}

					app.uartRx.head = head;
				}
				else
				{
				/* USB RX packet can not fit free space in the buffer */
				}	    
	      osSignalSet(app.ctrlTask.Handle, app.ctrlTask.Signal);
	    }
		osDelay (10);
	//printf("tick:%d\r\n",);
    }
}


/****************************************************************************/ /**
 *
 *                          End of UART Configuration
 *
 *******************************************************************************/
