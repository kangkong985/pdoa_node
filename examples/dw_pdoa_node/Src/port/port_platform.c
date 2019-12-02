/*! ----------------------------------------------------------------------------
 * @file    port_platform.c
 * @brief   HW specific definitions and functions for portability
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

#include "port_platform.h"
#include "deca_device_api.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_spi.h"
#include "dw_pdoa_node_common.h"
#include "sdk_config.h"

/******************************************************************************
 *
 *                              APP global variables
 *
 ******************************************************************************/

spi_handle_t spiA_handler;
spi_handle_t spiB_handler;

spi_handle_t *pgSpiHandler = &spiA_handler;

dw_t dw_chip_A
=
{
    .irqPin    = DW1000_IRQ_A_Pin,
    .rstPin    = DW1000_RST_A_Pin,
    .wkupPin   = DW1000_WUP_A_Pin,
    .pSpi      = &spiA_handler,
};

dw_t dw_chip_B
=
{
    .irqPin    = DW1000_IRQ_B_Pin,
    .rstPin    = DW1000_RST_B_Pin,
    .wkupPin   = DW1000_WUP_B_Pin,
    .pSpi      = &spiB_handler,
};

const dw_t *pDwMaster = &dw_chip_A; /**< by default chip 0 (A) is the "MASTER" */
const dw_t *pDwSlave  = &dw_chip_B; /**< by default chip 1 (B) is the "SLAVE" */

static volatile uint32_t signalResetDone;
uint32_t time32_incr = 0;
uint32_t timer_val = 0;

int readfromspi_uni(uint16 headerLength,
                    const uint8 *headerBuffer,
                    uint32 readlength,
                    uint8 *readBuffer,
                    spi_handle_t *pgSpiHandler);

int writetospi_uni(uint16 headerLength,
                    const uint8 *headerBuffer,
                    uint32 bodylength,
                    uint8 *bodyBuffer,
                    spi_handle_t *pgSpiHandler);

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context);

static int port_init_device(dw_name_e chip_id);


/******************************************************************************
 *
 *                              Time section
 *
 ******************************************************************************/

/* @fn    portGetTickCnt
 * @brief wrapper for to read a SysTickTimer, which is incremented with
 *        CLOCKS_PER_SEC frequency.
 *        The resolution of time32_incr is usually 1/1000 sec.
 * */
__INLINE uint32_t
portGetTickCount(void)
{
    return time32_incr;
}

void disable_dw1000_irq(void)
{
    NVIC_DisableIRQ((IRQn_Type)pDwMaster->irqPin);
    NVIC_DisableIRQ((IRQn_Type)pDwSlave->irqPin);
}

void enable_dw1000_irq(void)
{
    NVIC_EnableIRQ((IRQn_Type)pDwMaster->irqPin);
    NVIC_EnableIRQ((IRQn_Type)pDwSlave->irqPin);
}

/* @fn      reset_DW1000
 * @brief   DW_RESET pin on DW1000 has 2 functions
 *          In general it is output, but it also can be used to reset the
 *          digital part of DW1000 by driving this pin low.
 *          Note, the DW_RESET pin should not be driven high externally.
 * */
void reset_DW1000(dw_name_e chip_id)
{
    uint32 reset_gpio_pin;
    
    if (chip_id == DW_MASTER)
    {
        reset_gpio_pin = DW1000_RST_A_Pin;
    }
    else
    {
        reset_gpio_pin = DW1000_RST_B_Pin;
    }
    nrf_gpio_cfg_output(reset_gpio_pin);
    nrf_gpio_pin_clear(reset_gpio_pin);
    nrf_delay_ms(2);
    nrf_gpio_pin_set(reset_gpio_pin);
    nrf_delay_ms(50);
    nrf_gpio_cfg_input(reset_gpio_pin, NRF_GPIO_PIN_NOPULL);
    nrf_delay_ms(2);
}

/* @fn      port_wakeup_dw1000
 * @brief   "slow" waking up of DW1000 using DW_CS only
 * */
void port_wakeup_dw1000(dw_name_e chip_id)
{
    uint32 wkup_gpio_pin;
    
    if (chip_id == DW_MASTER)
    {
        wkup_gpio_pin = DW1000_WUP_A_Pin;
    }
    else
    {
        wkup_gpio_pin = DW1000_WUP_B_Pin;
    }

    nrf_gpio_pin_clear(wkup_gpio_pin);
    nrf_delay_ms(1);
    nrf_gpio_pin_set(wkup_gpio_pin);
    nrf_delay_ms(7);
}


void port_disable_wake_init_dw(void)
{
    taskENTER_CRITICAL();         

    pDwMaster = &dw_chip_A;
    pDwSlave = &dw_chip_B;

    disable_dw1000_irq();             /**< disable NVIC IRQ until we configure the device */

    port_reinit_dw_chips();

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    port_wakeup_dw1000(DW_MASTER);  //TODO: replace with reset? as the same will be done in the port_init_device
    port_wakeup_dw1000(DW_SLAVE);   //TODO: replace with reset? as the same will be done in the port_init_device

    if( (port_init_device(DW_SLAVE) != 0x00) || \
        (port_init_device(DW_MASTER) != 0x00))
    {
        error_handler(1,  _ERR_INIT);
    }

    taskEXIT_CRITICAL();
}

void port_reinit_dw_chips(void)
{
    nrf_gpio_pin_set(DW1000_CS_A_Pin);
    nrf_gpio_cfg_output(DW1000_CS_A_Pin);

    nrf_gpio_pin_set(DW1000_CS_B_Pin);
    nrf_gpio_cfg_output(DW1000_CS_B_Pin);

    nrf_gpio_pin_clear(DW1000_WUP_A_Pin);
    nrf_gpio_pin_clear(DW1000_WUP_B_Pin);

     /* Setup DW1000 IRQ pin for Master Chip A */
    nrf_gpio_cfg_input(DW1000_IRQ_A_Pin, NRF_GPIO_PIN_PULLDOWN);     //irq

     /* Setup DW1000 IRQ pin for Slave Chip B */
    nrf_gpio_cfg_input(DW1000_IRQ_B_Pin, NRF_GPIO_PIN_PULLDOWN);     //irq

    nrf_gpio_pin_set(DW1000_RST_A_Pin);
    nrf_delay_ms(50);
	nrf_gpio_cfg_input(DW1000_RST_A_Pin, NRF_GPIO_PIN_NOPULL);
//    nrf_gpio_cfg_output(DW1000_RST_A_Pin /*, NRF_GPIO_PIN_NOPULL */);

    nrf_gpio_pin_set(DW1000_RST_B_Pin);
    nrf_delay_ms(50);
	nrf_gpio_cfg_input(DW1000_RST_B_Pin, NRF_GPIO_PIN_NOPULL);
//    nrf_gpio_cfg_output(DW1000_RST_B_Pin /*, NRF_GPIO_PIN_NOPULL */);

}

void init_SPI_master()
{
    nrf_drv_spi_t   *spi_inst;
    nrf_drv_spi_config_t  *spi_config;

    spi_inst = &spiA_handler.spi_inst;
    spi_config = &spiA_handler.spi_config;
    spi_inst->inst_idx = SPI0_INSTANCE_INDEX;
    spi_inst->use_easy_dma = SPI0_USE_EASY_DMA;
    spi_inst->u.spim.p_reg = NRF_SPIM0;
    spi_inst->u.spim.drv_inst_idx = NRFX_SPIM0_INST_IDX;

    spiA_handler.frequency_slow = NRF_DRV_SPI_FREQ_2M;
    spiA_handler.frequency_fast = NRF_DRV_SPI_FREQ_8M;

    spi_config->sck_pin = SPI0_CONFIG_SCK_PIN;
    spi_config->mosi_pin = SPI0_CONFIG_MOSI_PIN;
    spi_config->miso_pin = SPI0_CONFIG_MISO_PIN;
    spi_config->ss_pin = SPI_0_CS_PIN;
    spi_config->irq_priority = (APP_IRQ_PRIORITY_MID - 2);
    spi_config->orc = 0xFF;
    spi_config->frequency = NRF_DRV_SPI_FREQ_2M;
    spi_config->mode = NRF_DRV_SPI_MODE_0;
    spi_config->bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

    spiA_handler.lock = DW_HAL_NODE_UNLOCKED;
}

void init_SPI_slave()
{
    nrf_drv_spi_t   *spi_inst;
    nrf_drv_spi_config_t  *spi_config;

    spi_inst = &spiB_handler.spi_inst;
    spi_config = &spiB_handler.spi_config;

    spi_inst->inst_idx = SPI2_INSTANCE_INDEX;
    spi_inst->use_easy_dma = SPI2_USE_EASY_DMA;
    spi_inst->u.spim.p_reg = NRF_SPIM2;
    spi_inst->u.spim.drv_inst_idx = NRFX_SPIM2_INST_IDX;

    spiB_handler.frequency_slow = NRF_DRV_SPI_FREQ_2M;
    spiB_handler.frequency_fast = NRF_DRV_SPI_FREQ_8M;

    spi_config->sck_pin = SPI2_CONFIG_SCK_PIN;
    spi_config->mosi_pin = SPI2_CONFIG_MOSI_PIN;
    spi_config->miso_pin = SPI2_CONFIG_MISO_PIN;
    spi_config->ss_pin = SPI_2_CS_PIN;
    spi_config->irq_priority = (APP_IRQ_PRIORITY_MID - 1);
    spi_config->orc = 0xFF;
    spi_config->frequency = NRF_DRV_SPI_FREQ_2M;
    spi_config->mode = NRF_DRV_SPI_MODE_0;
    spi_config->bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

    spiB_handler.lock = DW_HAL_NODE_UNLOCKED;
}

void set_SPI_master(void)
{
    //while(pgSpiHandler->lock);    //wait the completion of the last transaction
    pgSpiHandler = pDwMaster->pSpi;
    dwt_setlocaldataptr(0);
}

void set_SPI_slave(void)
{
    //while(pgSpiHandler->lock);    //wait the completion of the last transaction
    pgSpiHandler = pDwSlave->pSpi;
    dwt_setlocaldataptr(1);
}

void port_init_dw_chips(void)
{
    init_SPI_master();
    init_SPI_slave();
}


/* @fn      port_set_dw1000_slowrate
 * @brief   set 2MHz
 *          n
 * */
void port_set_dw1000_slowrate(dw_name_e chip_id)
{

    if (chip_id == DW_MASTER)
    {
        set_SPI_master();

    }
    else
    {
        set_SPI_slave();
    }

    APP_ERROR_CHECK( nrf_drv_spi_init(&pgSpiHandler->spi_inst, 
                                      &pgSpiHandler->spi_config, 
                                      spi_event_handler,
                                      NULL) );
    nrf_delay_ms(2);

}

void set_dw_spi_slow_rate(dw_name_e chip_id)
{

    if (chip_id == DW_MASTER)
    {
        set_SPI_master();

    }
    else
    {
        set_SPI_slave();
    }

    APP_ERROR_CHECK( nrf_drv_spi_init2(&pgSpiHandler->spi_inst, 
                                      &pgSpiHandler->spi_config, 
                                      spi_event_handler,
                                      NULL) );
    nrf_delay_ms(2);

}

/* @fn      port_set_dw1000_slowrate
 * @brief   set 2MHz
 *          n
 * */
void set_dw_spi_fast_rate(dw_name_e chip_id)
{

    if (chip_id == DW_MASTER)
    {
        set_SPI_master();
        pgSpiHandler->spi_config.frequency = spiA_handler.frequency_fast;

    }
    else
    {
        set_SPI_slave();
        pgSpiHandler->spi_config.frequency = spiB_handler.frequency_fast;;
    }


    APP_ERROR_CHECK( nrf_drv_spi_init2(&pgSpiHandler->spi_inst, 
                                      &pgSpiHandler->spi_config, 
                                      spi_event_handler,
                                      NULL) );
    nrf_delay_ms(2);

}

/**
 *  @brief     Bare-metal level
 *          initialise master/slave DW1000 (check if can talk to device and wake up and reset)
 */
static int
port_init_device(dw_name_e chip_id)
{
    static int gRangingStart = 0;

    if(gRangingStart == 0)
    {
      port_set_dw1000_slowrate(chip_id);
      gRangingStart = 1;
    }
    else
    {
      set_dw_spi_slow_rate(chip_id);
    }

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    uint32   devID0 = dwt_readdevid() ;

    if(DWT_DEVICE_ID != devID0) //if the read of device ID fails, the DW1000 could be asleep
    {
        port_wakeup_dw1000(chip_id);

        devID0 = dwt_readdevid();
        // SPI not working or Unsupported Device ID
        if(DWT_DEVICE_ID != devID0)
            return (-1) ;
    }
    //clear the sleep bit in case it is set - so that after the hard reset below the DW does not go into sleep
    dwt_softreset();

    return 0;
}

void port_stop_all_UWB(void)
{
    decaIrqStatus_t s = decamutexon();

    set_dw_spi_slow_rate(DW_SLAVE);
    dwt_forcetrxoff();
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_ARFE | DWT_INT_RFSL |\
                       DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO, 0);
    dwt_softreset();
    dwt_setcallbacks(NULL, NULL, NULL, NULL);

    set_dw_spi_slow_rate(DW_MASTER);
    dwt_forcetrxoff();
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_ARFE | DWT_INT_RFSL |\
                       DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO, 0);

    dwt_softreset();
    dwt_setcallbacks(NULL, NULL, NULL, NULL);

    decamutexoff(s);
}

//-----------------------------------------------------------------------------
// Sync section
/**
 * @brief Used to synchronise the system clocks of two DW1000 ICs
 */
void port_set_syncenable(int enable)
{
    nrf_gpio_cfg_output(DW1000_SYNC_EN_Pin);

    if(enable)
    {
        nrf_gpio_pin_write(DW1000_SYNC_EN_Pin, 1);
    }
    else
    {
        nrf_gpio_pin_write(DW1000_SYNC_EN_Pin, 0);
    }
}

void port_set_sync(int enable)
{
    nrf_gpio_cfg_output(DW1000_SYNC_Pin);

    if(enable)
    {
        nrf_gpio_pin_write(DW1000_SYNC_Pin, 1);
    }
    else
    {
        nrf_gpio_pin_write(DW1000_SYNC_Pin, 0);
    }
}

void port_set_syncclear(int enable)
{
    nrf_gpio_cfg_output(DW1000_SYNC_CLR_Pin);

    if(enable)
    {
        nrf_gpio_pin_write(DW1000_SYNC_CLR_Pin, 1);
    }
    else
    {
        nrf_gpio_pin_write(DW1000_SYNC_CLR_Pin, 0);
    }
}

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
}

//==============================================================================

void close_spi(nrf_drv_spi_t *p_instance)
{

    NRF_SPIM_Type * p_spi = p_instance->u.spim.p_reg;
    nrf_spim_disable(p_spi);
}

void open_spi(nrf_drv_spi_t *p_instance)
{

    NRF_SPIM_Type * p_spi = p_instance->u.spim.p_reg;
    nrf_spim_enable(p_spi);
}

int readfromspi(uint16 headerLength,
                const uint8 *headerBuffer,
                uint32 readlength,
                uint8 *readBuffer)
{
    return readfromspi_uni(headerLength, headerBuffer,
                            readlength, readBuffer, pgSpiHandler);

}

int writetospi( uint16 headerLength,
                const uint8 *headerBuffer,
                uint32 bodylength,
                const uint8 *bodyBuffer)
{
    return writetospi_uni(headerLength, headerBuffer,
                          bodylength, bodyBuffer, pgSpiHandler);

}


int readfromspi_uni(uint16 headerLength,
                    const uint8 *headerBuffer,
                    uint32 readlength,
                    uint8 *readBuffer,
                    spi_handle_t *pgSpiHandler)
{
    uint8 idatabuf[DATALEN1]={0};
    uint8 itempbuf[DATALEN1]={0};

    uint8 * p1;
    uint32 idatalength=0;

    while(pgSpiHandler->lock);

    __HAL_LOCK(pgSpiHandler);

    open_spi(&pgSpiHandler->spi_inst);

    memset(idatabuf, 0, DATALEN1);
    memset(itempbuf, 0, DATALEN1);

    p1=idatabuf;
    memcpy(p1,headerBuffer, headerLength);

    p1 += headerLength;
    memset(p1,0x00,readlength);

    idatalength= headerLength + readlength;

    spi_xfer_done = false;
    nrf_drv_spi_transfer(&pgSpiHandler->spi_inst, idatabuf, idatalength, itempbuf, idatalength); // TBD
    while(!spi_xfer_done);
    p1=itempbuf + headerLength;
    memcpy(readBuffer, p1, readlength);

    close_spi(&pgSpiHandler->spi_inst);

    __HAL_UNLOCK(pgSpiHandler);

    return 0;
}

int writetospi_uni(uint16 headerLength,
                    const uint8 *headerBuffer,
                    uint32 bodylength,
                    uint8 *bodyBuffer,
                    spi_handle_t *pgSpiHandler)
{
    uint8 idatabuf[DATALEN1]={0};
    uint8 itempbuf[DATALEN1]={0};

    uint8 * p1;
    uint32 idatalength=0;

    while(pgSpiHandler->lock);

    __HAL_LOCK(pgSpiHandler);

    open_spi(&pgSpiHandler->spi_inst);

    memset(idatabuf, 0, DATALEN1);
    memset(itempbuf, 0, DATALEN1);

    p1=idatabuf;
    memcpy(p1,headerBuffer, headerLength);
    p1 += headerLength;
    memcpy(p1,bodyBuffer,bodylength);

    idatalength= headerLength + bodylength;

    spi_xfer_done = false;
    nrf_drv_spi_transfer(&pgSpiHandler->spi_inst, idatabuf, idatalength, itempbuf, idatalength);// TBD
    while(!spi_xfer_done);

    close_spi(&pgSpiHandler->spi_inst);

     __HAL_UNLOCK(pgSpiHandler);

    return 0;
}

#if 0       // TBD
/* @fn    usleep
 * @brief precise usleep() delay
 * */
int usleep(useconds_t usec)
{
    int i,j;
#pragma GCC ivdep
    for(i=0;i<usec;i++)
    {
#pragma GCC ivdep
        for(j=0;j<2;j++)
        {
            __NOP();
            __NOP();
        }
    }
    return 0;
}

/* @fn    Sleep
 * @brief Sleep delay in ms using SysTick timer
 * */
__INLINE void
Sleep(uint32_t x)
{
    HAL_Delay(x);
}
#endif

/**@brief Systick handler
 *
 * @param[in] void
 */
void SysTick_Handler (void) {
        time32_incr++;
}
/******************************************************************************
 *
 *                              END OF Time section
 *
 ******************************************************************************/

/******************************************************************************
 *
 *                              Configuration section
 *
 ******************************************************************************/

#if 0
/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITEnStatus(uint32_t x)
{
    return ((NVIC->ISER[(((uint32_t)x) >> 5UL)] &\
            (uint32_t)(1UL << (((uint32_t)x) & 0x1FUL)) ) == \
            (uint32_t)RESET)?(RESET):(SET);
}
/******************************************************************************
 *
 *                          End of configuration section
 *
 ******************************************************************************/

/******************************************************************************
 *
 *                          DW1000 port section
 *
 ******************************************************************************/

/* @fn      setup_DW1000RSTnIRQ
 * @brief   setup the DW_RESET pin mode
 *          0 - output Open collector mode
 *          !0 - input mode with connected EXTI0 IRQ
 */
void setup_DW1000RSTnIRQ(int enable)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if(enable)
    {
        // Enable GPIO used as DECA RESET for interrupt
        GPIO_InitStruct.Pin = DW_RESET_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);

        HAL_NVIC_EnableIRQ(EXTI0_IRQn);     //pin #0 -> EXTI #0
        HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
    }
    else
    {
        HAL_NVIC_DisableIRQ(EXTI0_IRQn);    //pin #0 -> EXTI #0

        //put the pin back to tri-state ... as
        //output open-drain (not active)
        GPIO_InitStruct.Pin = DW_RESET_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);
        HAL_GPIO_WritePin(DW_RESET_GPIO_Port, DW_RESET_Pin, GPIO_PIN_SET);
    }
}

/* @fn      port_wakeup_dw1000_fast
 * @brief   waking up of DW1000 using DW_CS and DW_RESET pins.
 *          The DW_RESET signalling that the DW1000 is in the INIT state.
 *          the total fast wakeup takes ~2.2ms and depends on crystal startup
 *          time
 * */
void port_wakeup_dw1000_fast(void)
{
    #define WAKEUP_TMR_MS   (10)

    uint32_t x = 0;
    uint32_t timestamp = HAL_GetTick(); //protection

    setup_DW1000RSTnIRQ(0);         //disable RSTn IRQ
    signalResetDone = 0;            //signalResetDone connected to RST_PIN_IRQ
    setup_DW1000RSTnIRQ(1);         //enable RSTn IRQ
    port_SPIx_clear_chip_select();  //CS low

    /* need to poll to check when the DW1000 is in the IDLE, the CPLL interrupt
       is not reliable
       when RSTn goes high the DW1000 is in INIT, it will enter IDLE after PLL
       lock (in 5 us) */
    while((signalResetDone == 0) && \
          ((HAL_GetTick() - timestamp) < WAKEUP_TMR_MS))
    {
        x++;     //when DW1000 will switch to an IDLE state RSTn pin will high
    }
    setup_DW1000RSTnIRQ(0);         //disable RSTn IRQ
    port_SPIx_set_chip_select();    //CS high

    /* it takes ~35us in total for the DW1000 to lock the PLL, download AON and
       go to IDLE state */
    usleep(35);
}
#endif

void deca_sleep(unsigned int time_ms)
{
#if 0           // TBD
    /* This assumes that the tick has a period of exactly one millisecond. See CLOCKS_PER_SEC define. */
    unsigned long end = portGetTickCount() + time_ms;
    while ((signed long)(portGetTickCount() - end) <= 0);

#else
    nrf_delay_ms(time_ms);
#endif
}

/**@brief timer_event_handler
 *
 * @param[in] void
 */
void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    timer_val++;
    // Enable SysTick Interrupt
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}
/******************************************************************************
 *
 *                          End APP port section
 *
 ******************************************************************************/



/******************************************************************************
 *
 *                              IRQ section
 *
 ******************************************************************************/
/*! ----------------------------------------------------------------------------
 * Function: decamutexon()
 *
 * Description: This function should disable interrupts.
 *
 *
 * input parameters: void
 *
 * output parameters: uint16
 * returns the state of the DW1000 interrupt
 */

decaIrqStatus_t decamutexon(void)
{
    uint32_t s = NVIC_GetPendingIRQ((IRQn_Type)DW1000_IRQ_A_Pin);// TBD
    if(s)
    {
        NVIC_DisableIRQ((IRQn_Type)DW1000_IRQ_A_Pin);
    }
    return 0; // TBD
}
/*! ----------------------------------------------------------------------------
 * Function: decamutexoff()
 *
 * Description: This function should re-enable interrupts, or at least restore
 *              their state as returned(&saved) by decamutexon
 * This is called at the end of a critical section
 *
 * input parameters:
 * @param s - the state of the DW1000 interrupt as returned by decamutexon
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
void decamutexoff(decaIrqStatus_t s)
{
    if(s)
    {
        NVIC_EnableIRQ((IRQn_Type)DW1000_IRQ_A_Pin); // TBD
    }
}

/* @fn      port_CheckEXT_IRQ
 * @brief   wrapper to read DW_IRQ input pin state
 * */
uint32_t port_CheckEXT_IRQ(void)
{
    return nrf_gpio_pin_read(DW1000_IRQ_A_Pin) | nrf_gpio_pin_read(DW1000_IRQ_B_Pin);
}

/* @fn      process_deca_irq
 * @brief   main call-back for processing of DW1000 IRQ
 *          it re-enters the IRQ routing and processes all events.
 *          After processing of all events, DW1000 will clear the IRQ line.
 * */
void process_deca_irq(void)
{
    while(port_CheckEXT_IRQ() != 0)
    {
        set_SPI_master();
        dwt_isr();

    } //while DW1000 IRQ line active
}


#if 0
/* @fn      HAL_GPIO_EXTI_Callback
 * @brief   IRQ HAL call-back for all EXTI configured lines
 *          i.e. DW_RESET_Pin and DW_IRQn_Pin
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DW_RESET_Pin)
    {
        signalResetDone = 1;
    }
    else if (GPIO_Pin == DW_IRQn_Pin)
    {
        process_deca_irq();
    }
    else
    {
    }
}

/* @fn      process_deca_irq
 * @brief   main call-back for processing of DW1000 IRQ
 *          it re-enters the IRQ routing and processes all events.
 *          After processing of all events, DW1000 will clear the IRQ line.
 * */
__INLINE void process_deca_irq(void)
{
    while(port_CheckEXT_IRQ() != 0)
    {

        dwt_isr();

    } //while DW1000 IRQ line active
}


/* @fn      port_DisableEXT_IRQ
 * @brief   wrapper to disable DW_IRQ pin IRQ
 *          in current implementation it disables all IRQ from lines 5:9
 * */
__INLINE void port_DisableEXT_IRQ(void)
{
    NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn);
}

/* @fn      port_EnableEXT_IRQ
 * @brief   wrapper to enable DW_IRQ pin IRQ
 *          in current implementation it enables all IRQ from lines 5:9
 * */
__INLINE void port_EnableEXT_IRQ(void)
{
    NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn);
}


/* @fn      port_GetEXT_IRQStatus
 * @brief   wrapper to read a DW_IRQ pin IRQ status
 * */
__INLINE uint32_t port_GetEXT_IRQStatus(void)
{
    return EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn);
}


/* @fn      port_CheckEXT_IRQ
 * @brief   wrapper to read DW_IRQ input pin state
 * */
__INLINE uint32_t port_CheckEXT_IRQ(void)
{
    return HAL_GPIO_ReadPin(DECAIRQ_GPIO, DW_IRQn_Pin);
}

#endif
/******************************************************************************
 *
 *                              END OF IRQ section
 *
 ******************************************************************************/


#if 0

/* DW1000 IRQ handler definition. */
port_deca_isr_t port_deca_isr = NULL;

/*! ----------------------------------------------------------------------------
 * @fn port_set_deca_isr()
 *
 * @brief This function is used to install the handling function for DW1000 IRQ.
 *
 * NOTE:
 *   - As EXTI9_5_IRQHandler does not check that port_deca_isr is not null, the
 *     user application must ensure that a
 *     proper handler is set by calling this function before any DW1000 IRQ
 *     occurs!
 *   - This function makes sure the DW1000 IRQ line is deactivated while the
 *     handler is installed.
 *
 * @param deca_isr function pointer to DW1000 interrupt handler to install
 *
 * @return none
 */
void port_set_deca_isr(port_deca_isr_t deca_isr)
{
    /* Check DW1000 IRQ activation status. */
    ITStatus en = port_GetEXT_IRQStatus();

    /* If needed, deactivate DW1000 IRQ during the installation of the new
       handler. */
    if (en)
    {
        port_DisableEXT_IRQ();
    }
    port_deca_isr = deca_isr;
    if (en)
    {
        port_EnableEXT_IRQ();
    }
}

#endif

/******************************************************************************
 *
 ******************************************************************************/

