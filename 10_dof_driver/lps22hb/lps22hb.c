// Inspired by Kris Winer (https://github.com/kriswiner/LSM6DSM_LIS2MDL_LPS22HB)
// Adapted for NRF52 by Niklas Casaril <niklas@loligoelectronics.com>

#define NRF_LOG_MODULE_NAME LP22HB

#include "boards.h"

#include "nrf.h"
//#include "nrf_drv_gpiote.h"
#include "nrf_log.h"

NRF_LOG_MODULE_REGISTER();

#include "lps22hb.h"
#include "lps22hb_reg.h"
#include "dw_pdoa_node_common.h"

static nrf_drv_twi_t *_lps22hb_twi_inst = NULL;

static uint8_t lps22hb_read(uint8_t reg)
{
    uint8_t out[1] = {reg};
    uint8_t in[1];
    uint8_t device_address = LPS22HB_I2C_ADDR;

    nrf_drv_twi_enable(_lps22hb_twi_inst);
    nrf_drv_twi_tx(_lps22hb_twi_inst, device_address, out, sizeof(out), false);
    nrf_drv_twi_rx(_lps22hb_twi_inst, device_address, in, sizeof(in));
    nrf_drv_twi_disable(_lps22hb_twi_inst);

    return in[0];
}

static void lps22hb_write(uint8_t reg, uint8_t value)
{
    uint8_t out[2] = {reg, value};
    uint8_t device_address = LPS22HB_I2C_ADDR;
    nrf_drv_twi_enable(_lps22hb_twi_inst);
    nrf_drv_twi_tx(_lps22hb_twi_inst, device_address, out, sizeof(out), false);
    nrf_drv_twi_disable(_lps22hb_twi_inst);
}

int32_t lps22hb_hwtest(void)
{
    uint8_t device_id = lps22hb_read(LPS22HB_WHO_AM_I);
    uint8_t result = (device_id == LPS22HB_WHO_AM_I_RES);
    if (!result)
    {
        NRF_LOG_ERROR("HWTEST: %u, got=%X, expected=%X", result, device_id,
                     LPS22HB_WHO_AM_I_RES);
    }
    return (result) ? NRF_SUCCESS : 1;
}

void lps22hb_init(nrf_drv_twi_t *twi)
{
    _lps22hb_twi_inst = twi;

    lps22hb_write(LPS22HB_CTRL_REG2, LPS22HB_REG2_RESET);
    lps22hb_write(LPS22HB_CTRL_REG3, LPS22HB_REG3_DRDY);

    nrf_gpio_cfg_input(LPS22HB_BR_CS_Pin, GPIO_PIN_CNF_PULL_Pullup);
}

void lps22hb_oneshot()
{
    lps22hb_write(LPS22HB_CTRL_REG2, LPS22HB_REG2_ONESHOT);
}

lps22hb_result_t lps22hb_poll(void)
{
    lps22hb_result_t result;

    uint8_t temp_h = lps22hb_read(LPS22HB_TEMP_OUT_H);
    uint8_t temp_l = lps22hb_read(LPS22HB_TEMP_OUT_L);
    result.temperature = ((int32_t)temp_h << 8) | (int32_t)temp_l;

    uint8_t press_l = lps22hb_read(LPS22HB_PRESS_OUT_L);
    uint8_t press_h = lps22hb_read(LPS22HB_PRESS_OUT_H);
    uint8_t press_xl = lps22hb_read(LPS22HB_PRESS_OUT_XL);
    result.pressure = (((uint32_t)press_h << 16) | ((uint32_t)press_l << 8) |
                       ((uint32_t)press_xl)) *
                      100 / 4096;

    return result;
}
