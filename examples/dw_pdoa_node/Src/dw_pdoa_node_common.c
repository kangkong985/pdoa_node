/*! ----------------------------------------------------------------------------
 * @file    dw_pdoa_tag_common.c
 * @brief   TBD
 *
 * @author  Decawave 
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @modified Path Partner 2018
 */

/* Includes ------------------------------------------------------------------*/

#include "app_error.h"
#include "app_util.h"
//#include "app_usbd_cdc_acm.h"

#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"

#include "lsm6dsl.h"
#include "lps22hb.h"
#include "lis2mdl.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "port_platform.h"

#include "dw_pdoa_node_common.h"
#include "circ_buf.h"

#include "app_uart.h"
#include "nrf.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "cmd_fn.h"

#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

void interrupts_init(void);

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

void deca_irq_handler(nrf_drv_gpiote_pin_t irqPin, nrf_gpiote_polarity_t irq_action)
{
    process_deca_irq();
}

/* @fn  peripherals_init
 *
 * @param[in] void
 * */
void peripherals_init(void)
{
  ret_code_t ret;
  ret_code_t err_code;

  err_code = nrf_drv_clock_init();
  APP_ERROR_CHECK(err_code);

  nrf_drv_clock_lfclk_request(NULL);   // TBD Need to check

  /* Configure board. */
   bsp_board_init(BSP_INIT_LEDS);

#ifndef ENABLE_USB_PRINT
    ret = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(ret);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\n\rDeca Test Example......");
    NRF_LOG_FLUSH();
#endif 

	//2019/1/17
	nrf_gpio_cfg_output(DW1000_POWER_EN_Pin);
	nrf_gpio_pin_write(DW1000_POWER_EN_Pin, 1);
	
	nrf_gpio_cfg_output(DW1000_SYNC_CLR_Pin);
	nrf_gpio_pin_write(DW1000_SYNC_CLR_Pin, 0);
	
	nrf_gpio_cfg_output(DW1000_SYNC_EN_Pin);
	nrf_gpio_pin_write(DW1000_SYNC_EN_Pin, 0);

	nrf_gpio_cfg_output(DW1000_SYNC_Pin);
	nrf_gpio_pin_write(DW1000_SYNC_Pin, 0);

    peripherals_init_uart();

    interrupts_init();
}


void interrupts_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN; 

    err_code = nrf_drv_gpiote_in_init(DW1000_IRQ_A_Pin, &in_config, deca_irq_handler);
    APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_gpiote_in_init(DW1000_IRQ_B_Pin, &in_config, deca_irq_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(DW1000_IRQ_A_Pin, true);
	nrf_drv_gpiote_in_event_enable(DW1000_IRQ_B_Pin, true);

}
/* @fn  sensor_init
 *
 * @param[in] void
 * */
void sensor_init(void)
{
   ret_code_t err_code;

   /* Init I2C for the intertial sensors */
   nrf_drv_twi_t _twi = NRF_DRV_TWI_INSTANCE(1);
   nrf_drv_twi_config_t twi_conf = {
        .frequency = NRF_TWI_FREQ_400K,
        .scl = ARDUINO_SCL_PIN,
        .sda = ARDUINO_SDA_PIN,
        .clear_bus_init=true,
        .hold_bus_uninit=true
   };
   err_code = nrf_drv_twi_init(&_twi, &twi_conf, NULL, NULL);
   APP_ERROR_CHECK(err_code);

   lps22hb_init(&_twi);
   lis2mdl_init(&_twi, 0);
   lsm6dsl_init(&_twi);
}

#define ERROR_STR_LEN 12
void error_handler(int block, error_e err)
{
    // TBD : WD 
    //nrf_gpio_pin_write(LED_ERROR, 1);
    nrf_gpio_pin_toggle(LED_STATIONARY);
#if 0
	char *str = CMD_MALLOC(ERROR_STR_LEN);
    if(str)
    {
    	memset(str,0,ERROR_STR_LEN);
		sprintf(str,"[errcode]%d",err);
		port_tx_msg((uint8_t*)str, strlen(str));
		CMD_FREE(str);
    }
#endif
	app.lastErrorCode =err;

}

