#include "nrf_drv_timer.h"
#include "nrf_gpio.h"
#include "cmsis_os.h"

#define ECHO_FIRST 	5
#define ECHO_SECOND 28
#define TRIG_FIRST 	6
#define TRIG_SECOND	7

#define CYCLE_TIME 100

enum {
	NO_ECHORESPONSE = 1,
	OVERRANGE,
};

typedef enum {
	TRIG,
	WAIT4ECHO,
	CALCULATING,
	RESPONSE,
	HANGING,
	FAIL,
}ULTRASONIC_STATUS;

const nrf_drv_timer_t TIMER_ULTRASONIC = NRF_DRV_TIMER_INSTANCE(0);
static uint16_t counter_us = 0xffff;
static ULTRASONIC_STATUS status = HANGING;
static uint32_t ori_time = 0xffff;

const nrf_drv_timer_t TIMER_ULTRASONIC_2 = NRF_DRV_TIMER_INSTANCE(1);
static uint16_t counter_us_2 = 0xffff;
static ULTRASONIC_STATUS status_2 = HANGING;
static uint32_t ori_time_2 = 0xffff;

uint16_t dis_1st, dis_2nd;

void timer_ultrasonic_2_event_handler(nrf_timer_event_t event_type, void* p_context)
{

//	printf("status = 0x%x\r\n", status);

	static uint8_t error_code;

	switch (event_type)
	{
		case NRF_TIMER_EVENT_COMPARE0:

			counter_us_2++;

			switch (status_2)
			{
				case HANGING:
					if (0 == counter_us_2)
					{
//						nrf_gpio_pin_set(TRIG_FIRST);
						nrf_gpio_pin_set(TRIG_SECOND);
						status_2 = TRIG;
						break;
					}
					else if (counter_us_2 > CYCLE_TIME)
					{
						counter_us_2 = 0xffff;
						ori_time_2 = 0x0;
						break;
					}
					else
                                        {
                                                osThreadYield();
						break;
                                        }
				case TRIG:
					if (1 == counter_us_2)
					{
//						nrf_gpio_pin_clear(TRIG_FIRST);
						nrf_gpio_pin_clear(TRIG_SECOND);
						status_2 = WAIT4ECHO;
//						break;
					}
					else
						break;
				case WAIT4ECHO:
					if (/* nrf_gpio_pin_read(ECHO_FIRST) && */ nrf_gpio_pin_read(ECHO_SECOND))
					{
						ori_time_2 = counter_us_2;
						status_2 = CALCULATING;
						break;
					}
					else if ((counter_us_2==CYCLE_TIME) &&/* !nrf_gpio_pin_read(ECHO_FIRST) */ !nrf_gpio_pin_read(ECHO_SECOND))
					{
						status_2 = FAIL;
						error_code = NO_ECHORESPONSE;
						break;
					}
                    else
                        break;
				case CALCULATING:
					if (/* (counter_ms<=80) && */ /* !nrf_gpio_pin_read(ECHO_FIRST) */ !nrf_gpio_pin_read(ECHO_SECOND))
					{
						dis_2nd = (counter_us_2-ori_time_2)*17;
//						printf("2nd = %d\r\n", dis_2nd);
						status_2 = HANGING;
						break;
					}
					else if (counter_us_2 >= CYCLE_TIME)
					{
						status_2 = FAIL;
						error_code = OVERRANGE;
						dis_2nd = 0xffff;
						break;
					}
					else
						break;
				case FAIL:
					{
//						SEGGER_RTT_printf("%d Ultra error, code = %d\r\n", __LINE__, error_code);
						status_2 = HANGING;
//						counter_ms = 0xff;
//						ori_time = 0x0;
						break;
					}
				default:
					if (counter_us_2 >= CYCLE_TIME)
					{
						counter_us_2 = 0xffff;
						ori_time_2 = 0x0;
						status_2 = HANGING;
						break;
					}
			}
			
			break;
		default:
			break;
	}
}

void timer_ultrasonic_event_handler(nrf_timer_event_t event_type, void* p_context)
{

//	printf("status = 0x%x\r\n", status);

	static uint8_t error_code;

	switch (event_type)
	{
		case NRF_TIMER_EVENT_COMPARE0:

			counter_us++;

			switch (status)
			{
				case HANGING:
					if (0 == counter_us)
					{
						nrf_gpio_pin_set(TRIG_FIRST);
//						nrf_gpio_pin_set(TRIG_SECOND);
						status = TRIG;
						break;
					}
					else if (counter_us > CYCLE_TIME)
					{
						counter_us = 0xffff;
						ori_time = 0x0;
						break;
					}
					else
                                        {
                                                osThreadYield();
						break;
                                        }
				case TRIG:
					if (1 == counter_us)
					{
						nrf_gpio_pin_clear(TRIG_FIRST);
//						nrf_gpio_pin_clear(TRIG_SECOND);
						status = WAIT4ECHO;
//						break;
					}
					else
						break;
				case WAIT4ECHO:		
					if (nrf_gpio_pin_read(ECHO_FIRST)/* && nrf_gpio_pin_read(ECHO_SECOND) */)
					{
						ori_time = counter_us;
						status = CALCULATING;
						break;
					}
					else if ((counter_us==CYCLE_TIME) && !nrf_gpio_pin_read(ECHO_FIRST)/* && !nrf_gpio_pin_read(ECHO_SECOND) */)
					{
						status = FAIL;
						error_code = NO_ECHORESPONSE;
						break;
					}
                    else
                        break;
				case CALCULATING:
					if (/* (counter_ms<=80) && */ !nrf_gpio_pin_read(ECHO_FIRST)/*  !nrf_gpio_pin_read(ECHO_SECOND) */)
					{
						dis_1st = (counter_us-ori_time)*17;
//						SEGGER_RTT_printf("dis_1st = %d", dis_1st);
//						printf("1st = %d\r\n", dis_1st);
						status = HANGING;
						break;
					}
					else if (counter_us >= CYCLE_TIME)
					{
						status = FAIL;
						error_code = OVERRANGE;
						dis_1st = 0xffff;
						break;
					}
					else
						break;
				case FAIL:
					{
//						SEGGER_RTT_printf("%d Ultra error, code = %d\r\n", __LINE__, error_code);
						status = HANGING;
//						counter_ms = 0xff;
//						ori_time = 0x0;
						break;
					}
				default:
					if (counter_us >= CYCLE_TIME)
					{
						counter_us = 0xffff;
						ori_time = 0x0;
						status = HANGING;
						break;
					}
			}
			
			break;
		default:
			break;
	}
}


void timer_init(void)
{
	uint32_t time_ms = 1; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;
	
	nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&TIMER_ULTRASONIC, &timer_cfg, timer_ultrasonic_event_handler);
    APP_ERROR_CHECK(err_code);

	 time_ticks = nrfx_timer_ms_to_ticks(&TIMER_ULTRASONIC, time_ms);

    nrf_drv_timer_extended_compare(
         &TIMER_ULTRASONIC, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

	err_code = nrf_drv_timer_init(&TIMER_ULTRASONIC_2, &timer_cfg, timer_ultrasonic_2_event_handler);
	APP_ERROR_CHECK(err_code);

	nrf_drv_timer_extended_compare(
		&TIMER_ULTRASONIC_2, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

//	printf("%d\r\n", time_ticks);

    nrf_drv_timer_enable(&TIMER_ULTRASONIC);
	nrf_drv_timer_enable(&TIMER_ULTRASONIC_2);

	nrf_gpio_cfg_output(TRIG_FIRST);
	nrf_gpio_cfg_output(TRIG_SECOND);

	nrf_gpio_cfg_input(ECHO_FIRST, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(ECHO_SECOND, NRF_GPIO_PIN_PULLDOWN);
}
