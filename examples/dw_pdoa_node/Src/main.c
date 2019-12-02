/*! ----------------------------------------------------------------------------
 * @file    main.c
 * @brief   This is the implementation of the PDOA Tag on Nordic nRF52840 on FreeRTOS
 *
 * @author Decawave Software
 *
 * @attention Copyright 2017 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 * @modified PathPartner 2018
 *
 */
/* Includes ------------------------------------------------------------------*/

#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"

#include "dw_pdoa_node_common.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include <config.h>

#include <node.h>
//#include <task_tag.h>
#include <tag_list.h>
#include <task_node.h>
//#include <task_flush.h>
#include <task_uart2spi.h>

//
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
//#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
//#endif
/* When UART is used for communication with the host do not use flow control.*/
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

#define UART_DRV_UPDATE_MS 1//500
extern uint8_t rx_buf[256];

osThreadId defaultTaskHandle;
//task_signal_t monitorTask;
app_t app; /**< All global variables are in the "app" structure */

void DefaultTask(void const *argument);
void FlushTask(void const *argument);
void CtrlTask(void const *arg);

//void MonitorTask(void const *argument);

extern void timer_init(void);

int UARTInitState = 0;

/**
 * @brief Function for application main entry.
 */
 int main(void) {
  int devID = 0, delayCnt = 0, status = 0;
  ret_code_t err_code;

    peripherals_init();

//    timer_init();

    port_init_dw_chips();

    memset(&app,0,sizeof(app));

  init_nvm_appdata();
  load_bssConfig();                          /**< load the RAM Configuration parameters from NVM block */
  app.pConfig = get_pbssConfig();            /**< app.pConfig pointed to the RAM config block */
  app.xStartTaskEvent = xEventGroupCreate(); /**< xStartTaskEvent indicates which tasks to be started */
  app.pConfig->s.uartEn=1;
#if 0 // TBD

   HAL_IWDG_Refresh(&hiwdg);

  //TODO: below needs for low power mode debug.
  Sleep(1000);
  HAL_IWDG_Refresh(&hiwdg);
  Sleep(1000);
  HAL_IWDG_Refresh(&hiwdg);
  Sleep(1000);
  HAL_IWDG_Refresh(&hiwdg);

#endif

  nrf_delay_ms(1000); /**< small pause to startup */ // TBD Debug

  //tag mission
  //port_set_dw1000_slowrate_first_time(DW_MASTER);
  reset_DW1000(DW_MASTER); /**< This will sample DW1000 GPIO5/6 to correct SPI mode (0) */
  reset_DW1000(DW_SLAVE);  /**< This will sample DW1000 GPIO5/6 to correct SPI mode (0) */

//  timer_monitor_init();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes*/

  /* initialize inter-task communication mail queue for Node :
       *
       * The RxTask need to send the rxPckt to the CalcTask.
       *
       * TODO: rxPcktPool_q_id should be a part of twrInfo, but
       * FreeRTOS cannot free resources efficiently on task deletion.
       *
       * Current code has an implementation where twrInfo is statically defined
       * and rxPcktPool_q is a part of FreeRtos Heap.
       *
       * Note, the debug accumulator & diagnostics readings are a part of
       * mail queue. Every rx_mail_t has a size of ~6kB.
       *
       */
  osMailQDef(rxPcktPool_q, RX_MAIL_QUEUE_SIZE, rx_mail_t);
  app.rxPcktPool_q_id = osMailCreate(osMailQ(rxPcktPool_q), NULL);

  if (!app.rxPcktPool_q_id) {
    error_handler(1, _ERR_Cannot_Alloc_Mail);
  }

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, DefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  /* FlushTask is always working and flushing the output buffer to uart/usb */

  osThreadDef(flushTask, FlushTask, osPriorityNormal, 0, 128);
  app.flushTask.Handle = osThreadCreate(osThread(flushTask), NULL);

  /* ctrlTask is always working serving rx from uart/usb */
  //2K for CTRL task: it needs a lot of memory: it uses mallocs(512), sscanf(212bytes)
  osThreadDef(ctrlTask, CtrlTask, osPriorityBelowNormal, 0, 512);
  app.ctrlTask.Handle = osThreadCreate(osThread(ctrlTask), NULL);

//  osThreadDef(monitorTask, MonitorTask, osPriorityLow, 0, 4);
//  monitorTask.Handle = osThreadCreate(osThread(monitorTask), NULL);

  if (!defaultTaskHandle | !app.flushTask.Handle | !app.ctrlTask.Handle) {
    error_handler(1, _Err_Create_Task_Bad);
  }

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void command_stop_received(void) {
  xEventGroupSetBits(app.xStartTaskEvent, Ev_Stop_All);
}

//static str[500] = {0};


void DefaultTask(void const *argument) {
  /* USER CODE BEGIN 5 */
  const EventBits_t bitsWaitForA = (Ev_Node_A_Task | Ev_Tcfm_A_Task | Ev_Tcwm_A_Task | Ev_Usb2spi_A_Task);
  const EventBits_t bitsWaitForB = (Ev_Node_B_Task | Ev_Tcfm_B_Task | Ev_Tcwm_B_Task | Ev_Usb2spi_B_Task);
  const EventBits_t bitsWaitForAny = (bitsWaitForA | bitsWaitForB | Ev_Stop_All);

  EventBits_t uxBits;
  uint32_t chip;

    for(int i=0; i<6; i++)
    {
        nrf_gpio_pin_toggle(LED_STATIONARY);
        nrf_gpio_pin_toggle(LED_NODE);
        //nrf_gpio_pin_toggle(LED_USB);
        nrf_gpio_pin_toggle(LED_ERROR);
        nrf_delay_ms(250);
    }
 
	timer_init();

  app.mode = mIDLE;

  {
    //port_uart_rx_init();    // TBD
  }
//  pdoa_hex_pack_t tmp;
	//printf("pdoa_hex_pack_t=%d,\r\n",sizeof(tmp));
  if (app.pConfig->s.autoStartEn == 1) {
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Node_A_Task); /**< activate Node task */
  } else if (app.pConfig->s.autoStartEn == 2) {
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Node_B_Task); /**< activate Node task */
  }
//  short count = 0;
  /* Infinite loop: this is the helper task, which starts appropriate mode */
  while (1) {

    
    uxBits = xEventGroupWaitBits(app.xStartTaskEvent,
        bitsWaitForAny,
        pdTRUE, pdFALSE,
        UART_DRV_UPDATE_MS / portTICK_PERIOD_MS);
    
    //HAL_IWDG_Refresh(&hiwdg);   // TBD

        uxBits &= bitsWaitForAny;

        chip = (uint32_t)(uxBits & bitsWaitForB)?(DW_SLAVE):(DW_MASTER);

    if (uxBits) {
      /* Turn LEDs off on restart of top-level application */
//     nrf_gpio_pin_write(LED_STATIONARY, 0);
     nrf_gpio_pin_write(LED_NODE, 0);
     nrf_gpio_pin_write(LED_ERROR, 0);
	 //nrf_gpio_pin_write(LED_USB,1);  

      app.lastErrorCode = _NO_ERR;

      /*   need to switch off DW1000's RX and IRQ before killing tasks */
      if (app.mode != mIDLE) {
        taskENTER_CRITICAL();
        disable_dw1000_irq();
        set_dw_spi_slow_rate(DW_SLAVE);
        dwt_forcetrxoff();
        set_dw_spi_slow_rate(DW_MASTER);
        dwt_forcetrxoff();
        taskEXIT_CRITICAL();
      }
//printf("f=%s,l=%d  app.mode=0x%x,uxBits=%x\r\n",__FUNCTION__,__LINE__,app.mode,uxBits);
      /* Event to start/stop task received */
      /* 1. free the resources: kill all user threads and timers */
      node_terminate_tasks();
      //            usb2spi_terminate_tasks();
//      uart2spi_terminate_tasks();
#if 0
            tcfm_terminate_tasks();
            tcwm_terminate_tasks();
            reset_FlushTask();
#endif
     // app.lastErrorCode = _NO_ERR;
    }

    //HAL_IWDG_Refresh(&hiwdg);   // TBD
    osThreadYield(); //force switch of context that the Idle task can free resources

    taskENTER_CRITICAL();

    /* 2. Start appropriate RTOS top-level application or run a usb_vbus_driver() if a dummy loop */
    switch (uxBits) {
    case Ev_Node_A_Task:
    case Ev_Node_B_Task:
      app.mode = mTWR;
      node_helper(&chip); /* call Node helper function which will setup sub-tasks for Node process */
//	  printf("f=%s,l=%d  app.mode=0x%x\r\n",__FUNCTION__,__LINE__,app.mode);
      break;

#if 0
    case Ev_Usb2spi_A_Task:
    case Ev_Usb2spi_B_Task:
      /* Setup a Usb2Spi task : 8K of stack is required to this task */
      app.mode = mUART2SPI;

      osThreadDef(u2sTask, StartUart2SpiTask, osPriorityNormal, 0, 128);
      app.uart2spiTask.Handle = osThreadCreate(osThread(u2sTask), &chip);
      if (!app.uart2spiTask.Handle) {
        error_handler(1, _Err_Create_Task_Bad);
      }
      break;


        case Ev_Tcfm_A_Task:
        case Ev_Tcfm_B_Task:
            /* Setup a TCFM task */
            app.mode = mTCFM;

            osThreadDef(tcfmTask, StartTcfmTask, osPriorityNormal, 0, 128);
            app.tcfmTask.Handle = osThreadCreate(osThread(tcfmTask), &chip);
            if(!app.tcfmTask.Handle)
            {
                error_handler(1, _Err_Create_Task_Bad);
            }
            break;

        case Ev_Tcwm_A_Task:
        case Ev_Tcwm_B_Task:
            /* Setup a TCWM task */
            app.mode = mTCWM;

            osThreadDef(tcwmTask, StartTcwmTask, osPriorityNormal, 0, 128);
            app.tcwmTask.Handle = osThreadCreate(osThread(tcwmTask), &chip);
            if(!app.tcwmTask.Handle)
            {
                error_handler(1, _Err_Create_Task_Bad);
            }
            break;
#endif
    case Ev_Stop_All:
      app.mode = mIDLE;
      break;

    default:

      //HAL_IWDG_Refresh(&hiwdg);
      //usb_vbus_driver();    /**< connection / disconnection of USB interface : on connection activate flush_task & usb_rx_process  */
      break;
    }
//    xEventGroupSetBits(app.xStartTaskEvent, Ev_Usb2spi_A_Task);

    taskEXIT_CRITICAL(); //ready to switch to a created task
    osThreadYield();

#ifdef ENABLE_USB_PRINT
    if (UARTInitState == 0) {
      //pp_usb_init();
      app.usbState = USB_CONFIGURED;
      UARTInitState = 1;
    }
#endif
  }

  /* USER CODE END 5 */
}

//nrf_drv_timer_t TIMER_MONITOR = NRF_DRV_TIMER_INSTANCE(2);
//bool start_not = false;
//bool running = false;
//int32_t monitorSignal;

//static void task_reborn(void)
//{
//	uint32_t chip = DW_MASTER;

//	node_terminate_tasks();
//	osThreadYield();

//	taskENTER_CRITICAL();

//	app.mode = mTWR;
//    node_helper(&chip);

//	taskEXIT_CRITICAL(); //ready to switch to a created task
//    osThreadYield();
//}


//void MonitorTask(void const *argument)
//{
//	nrf_drv_timer_enable(&TIMER_MONITOR);
//	
//	while(1)
//	{
//		osSignalWait(monitorTask.Signal, osWaitForever);
//		task_reborn();
//		start_not = false;
//		running = false;
//	}
//}

//void timer_monitor_event_handler(nrf_timer_event_t event_type, void* p_context)
//{
//	static uint8_t monitor_timer;
//	
//	switch (event_type)
//	{
//		case NRF_TIMER_EVENT_COMPARE0:

//			if (start_not && !running)
//			{
//				reset_FlushTask();
//				xEventGroupSetBits(app.xStartTaskEvent, Ev_Stop_All);

//				xEventGroupSetBits(app.xStartTaskEvent, Ev_Node_A_Task);
//			}
//			else
//				running = false;
//			break;
//		default:
//			break;
//				
//	}
//}

//void timer_monitor_init(void)
//{
//	uint32_t time_ms = 100; //Time(in miliseconds) between consecutive compare events.
//	uint32_t time_ticks;
//	uint32_t err_code = NRF_SUCCESS;
//	
//	nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
//	err_code = nrf_drv_timer_init(&TIMER_MONITOR, &timer_cfg, timer_monitor_event_handler);
//	APP_ERROR_CHECK(err_code);
//	
//	 time_ticks = nrfx_timer_ms_to_ticks(&TIMER_MONITOR, time_ms);
//	
//	nrf_drv_timer_extended_compare(
//		 &TIMER_MONITOR, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
//	
//}

/** @} */
