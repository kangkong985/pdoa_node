// Inspired by Kris Winer (https://github.com/kriswiner/LSM6DSM_LIS2MDL_LPS22HB)
// Adapted for NRF52 by Niklas Casaril <niklas@loligoelectronics.com>

#ifndef LPS22HB_H
#define LPS22HB_H

#include "nrf_drv_twi.h"

typedef struct {
    int32_t temperature;
    uint32_t pressure;
} lps22hb_result_t;

void lps22hb_init(nrf_drv_twi_t *twi);
int32_t lps22hb_hwtest(void);
void lps22hb_oneshot(void);
lps22hb_result_t lps22hb_poll(void);

#endif
