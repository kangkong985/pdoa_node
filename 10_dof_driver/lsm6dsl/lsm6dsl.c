// Inspired by Kris Winer (https://github.com/kriswiner/LSM6DSM_LIS2MDL_LPS22HB)
// Adapted for NRF52 by Niklas Casaril <niklas@loligoelectronics.com>

#define NRF_LOG_MODULE_NAME LSM6DSL

#include "boards.h"
#include "nrf_assert.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "dw_pdoa_node_common.h"

NRF_LOG_MODULE_REGISTER();
#include "nrf_drv_twi.h"

#include "lsm6dsl.h"

static nrf_drv_twi_t *_twi_inst = NULL;
static lsm6dsl_inst_t _inst; 

uint8_t read_bytes(uint8_t address, uint8_t *buffer, uint16_t length)
{
    uint8_t out[1] = {address};

    uint8_t twi_addr = LSM6DSL_I2C_ADDR;

    nrf_drv_twi_enable(_twi_inst);
    ret_code_t ret = nrf_drv_twi_tx(_twi_inst, twi_addr, out, sizeof(out), false);
    nrf_drv_twi_rx(_twi_inst, twi_addr, buffer, length);
    nrf_drv_twi_disable(_twi_inst);

    // return 1 for error, 0 for success
    return ret;
}



uint8_t write_bytes(uint8_t address, uint8_t *buffer, uint16_t length)
{
    uint8_t twi_addr = LSM6DSL_I2C_ADDR;
    uint8_t out[16] = {address};
    ASSERT(length < 15);
    memcpy(out+1,buffer,length);

    nrf_drv_twi_enable(_twi_inst);
    ret_code_t ret = nrf_drv_twi_tx(_twi_inst, twi_addr, out, length+1, false);
    nrf_drv_twi_disable(_twi_inst);

    // return 1 for error, 0 for success
    return ret;
}

uint8_t write_byte(uint8_t address, uint8_t byte)
{
    uint8_t out[2] = {address, byte};
    uint8_t device_address = LSM6DSL_I2C_ADDR;
    nrf_drv_twi_enable(_twi_inst);
    nrf_drv_twi_tx(_twi_inst, device_address, out, sizeof(out), false);
    nrf_drv_twi_disable(_twi_inst);
    return 0;
}

void lsm6dsm_reset()
{
    // reset device
    uint8_t temp;
    read_bytes(LSM6DSM_CTRL3_C, &temp, 1);
    temp |= 0x01;
    write_bytes(LSM6DSM_CTRL3_C, &temp, 1); // Set bit 0 to 1 to reset LSM6DSM
    nrf_delay_ms(10); // Wait for all registers to reset 
}

void lsm6dsl_init(nrf_drv_twi_t *twi_inst)
{
    _twi_inst = twi_inst;

    lsm6dsl_hwtest();
    //lsm6dsm_reset();
    
    uint8_t temp;
    _inst.acc_sensitivity  = 16;
    write_byte(LSM6DSM_CTRL1_XL,(LSM6DSM_AODR_6660Hz << 4) | (LSM6DSM_AFS_16G<<2));
  
    _inst.gyro_sensitivity = 2000;
    write_byte(LSM6DSM_CTRL2_G, (LSM6DSM_GODR_6660Hz << 4) | (LSM6DSM_GFS_2000DPS<<2));

    write_byte(LSM6DSM_CTRL3_C, 0x04); 
    read_bytes(LSM6DSM_CTRL3_C, &temp, 1);
    // enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
    write_byte(LSM6DSM_CTRL3_C, temp| 0x40 | 0x04); 
    // by default, interrupts active HIGH, push pull, little endian data 
    // (can be changed by writing to bits 5, 4, and 1, resp to above register)
    read_bytes(LSM6DSM_CTRL3_C, &temp, 1);

    // enable accel LP2 (bit 7 = 1), set LP2 tp ODR/9 (bit 6 = 1), enable input_composite (bit 3) for low noise
    write_byte(LSM6DSM_CTRL8_XL, 0x80 | 0x40 | 0x08 );

    // Continous fifo, 104hz
    //write_byte(LSM6DSM_FIFO_CTRL1, 0x43);
    
    // interrupt handling
    write_byte(LSM6DSM_DRDY_PULSE_CFG, 0x80); // latch interrupt until data read
    write_byte(LSM6DSM_INT1_CTRL, 0x03);      // enable accel/gyro interrupts on INT1
}


uint8_t readRawData(int16_t *acc_gyro)
{    
    int16_t rawdata[7];
    uint8_t r = read_bytes(LSM6DSM_OUT_TEMP_L, (uint8_t*)rawdata, 14);
    for (int i=0;i<7;i++)
        acc_gyro[i] = rawdata[i];
    return r;
}

uint8_t lsm6dsl_readData_i(int16_t *gyro, int16_t *acc)
{    
    int16_t rawdata[7];
    uint8_t r = readRawData(rawdata);
    gyro[0] = rawdata[1];
    gyro[1] = rawdata[2];
    gyro[2] = rawdata[3];
    acc[0] = rawdata[4];
    acc[1] = rawdata[5];
    acc[2] = rawdata[6];
    return r;
}

uint8_t lsm6dsl_readData_f(float *dest1, float *dest2)
{    
    int16_t rawdata[7];
    uint8_t r = readRawData(rawdata);

    float gRes = _inst.gyro_sensitivity / 32768.0f;
    float aRes = _inst.acc_sensitivity  / 32768.0f;
    dest1[0] = rawdata[1]*gRes;
    dest1[1] = rawdata[2]*gRes;
    dest1[2] = rawdata[3]*gRes;
    dest2[0] = rawdata[4]*aRes;
    dest2[1] = rawdata[5]*aRes;
    dest2[2] = rawdata[6]*aRes;

    return r;
}


void lsm6dsl_offsetBias(float * dest1, float * dest2)
{
    int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
    int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};
    
    NRF_LOG_INFO("Calculate accel and gyro offset biases: keep sensor flat and motionless!");
    NRF_LOG_FLUSH();

    for (int ii = 0; ii < 128; ii++)
    {
        readRawData(temp);
        sum[1] += temp[1];
        sum[2] += temp[2];
        sum[3] += temp[3];
        sum[4] += temp[4];
        sum[5] += temp[5];
        sum[6] += temp[6];
        nrf_delay_ms(50);
    }

    float gRes = _inst.gyro_sensitivity / 32768.0f;
    float aRes = _inst.acc_sensitivity  / 32768.0f;
    
    dest1[0] = sum[1]*gRes/128.0f;
    dest1[1] = sum[2]*gRes/128.0f;
    dest1[2] = sum[3]*gRes/128.0f;
    dest2[0] = sum[4]*aRes/128.0f;
    dest2[1] = sum[5]*aRes/128.0f;
    dest2[2] = sum[6]*aRes/128.0f;
    
    if(dest2[0] > 0.8f)  {dest2[0] -= 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
    if(dest2[0] < -0.8f) {dest2[0] += 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
    if(dest2[1] > 0.8f)  {dest2[1] -= 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
    if(dest2[1] < -0.8f) {dest2[1] += 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
    if(dest2[2] > 0.8f)  {dest2[2] -= 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation
    if(dest2[2] < -0.8f) {dest2[2] += 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation
    
}



int32_t lsm6dsl_hwtest()
{
    uint8_t value = 0x0;

    uint8_t r = read_bytes(LSM6DSM_WHO_AM_I, &value, 1);
    if (r) return r;
    if (value != 0x6A)
    {
        NRF_LOG_ERROR("HWTEST: %u, got=%X, expected=%X", r, value,
                     0x6A);

    }
    return (value == 0x6A) ? NRF_SUCCESS : 1;
}
