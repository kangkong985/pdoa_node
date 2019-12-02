/* 09/23/2017 Copyright Tlera Corporation

   Created by Kris Winer

   The LIS2MDL is a low power magnetometer, here used as 3 DoF in a 9 DoF absolute orientation solution.

   Library may be used freely and without limit with attribution.

   Adapted for NRF52 by Niklas Casaril <niklas@loligoelectronics.com>
*/

#define NRF_LOG_MODULE_NAME LIS2MDL

#include <stdint.h>
#include "boards.h"
#include "lis2mdl.h"

#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "dw_pdoa_node_common.h"

NRF_LOG_MODULE_REGISTER();
static const nrf_drv_twi_t *_lis2mdl_twi_inst;

// I2C read/write functions for the LIS2MDL
static void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	uint8_t out[2] = {subAddress, data};

	nrf_drv_twi_enable(_lis2mdl_twi_inst);
	nrf_drv_twi_tx(_lis2mdl_twi_inst, address, out, sizeof(out), false);
	nrf_drv_twi_disable(_lis2mdl_twi_inst);
}


static uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t out[1] = {subAddress};
	uint8_t in[1];
	nrf_drv_twi_enable(_lis2mdl_twi_inst);
	nrf_drv_twi_tx(_lis2mdl_twi_inst, address, out, sizeof(out), false);
	nrf_drv_twi_rx(_lis2mdl_twi_inst, address, in, sizeof(in));
	nrf_drv_twi_disable(_lis2mdl_twi_inst);
	return in[0];
}


static void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) {
	uint8_t out[1] = {subAddress};

	nrf_drv_twi_enable(_lis2mdl_twi_inst);
	nrf_drv_twi_tx(_lis2mdl_twi_inst, address, out, sizeof(out), false);
	nrf_drv_twi_rx(_lis2mdl_twi_inst, address, dest, count);
	nrf_drv_twi_disable(_lis2mdl_twi_inst);
}


void lis2mdl_init(const nrf_drv_twi_t *twi_inst, uint8_t intPin)
{
	_lis2mdl_twi_inst = twi_inst;
	//pinMode(intPin, INPUT);
	//_intPin = intPin;

	// enable temperature compensation (bit 7 == 1), continuous mode (bits 0:1 == 00)
	writeByte(LIS2MDL_I2C_ADDR, LIS2MDL_CFG_REG_A, 0x80 | MODR<<2);	
	
	// enable low pass filter (bit 0 == 1), set to ODR/4
	writeByte(LIS2MDL_I2C_ADDR, LIS2MDL_CFG_REG_B, 0x01);  
	
	// enable data ready on interrupt pin (bit 0 == 1), enable block data read (bit 4 == 1)
	writeByte(LIS2MDL_I2C_ADDR, LIS2MDL_CFG_REG_C, 0x01 | 0x10);	 
	
}


int32_t lis2mdl_hwtest()
{
	uint8_t c = readByte(LIS2MDL_I2C_ADDR, LIS2MDL_WHO_AM_I);

    if (c!=0b01000000)
    {
        NRF_LOG_ERROR("LIS2MDL_WHO_AM_I: %x, expected: %x", c, 0b01000000);
    }
	
	return (c==0b01000000) ? 0 : 1;
}


void lis2mdl_reset()
{
	// reset device
	uint8_t temp = readByte(LIS2MDL_I2C_ADDR, LIS2MDL_CFG_REG_A);
	writeByte(LIS2MDL_I2C_ADDR, LIS2MDL_CFG_REG_A, temp | 0x20); // Set bit 5 to 1 to reset LIS2MDL
	nrf_delay_ms(1);
	writeByte(LIS2MDL_I2C_ADDR, LIS2MDL_CFG_REG_A, temp | 0x40); // Set bit 6 to 1 to boot LIS2MDL
	nrf_delay_ms(100); // Wait for all registers to reset 
}


uint8_t lis2mdl_status()
{
	// Read the status register of the altimeter  
	uint8_t temp = readByte(LIS2MDL_I2C_ADDR, LIS2MDL_STATUS_REG);	
	return temp;
}


void lis2mdl_readData(int16_t * destination)
{
	uint8_t rawData[6];	 // x/y/z mag register data stored here
	readBytes(LIS2MDL_I2C_ADDR, (0x80 | LIS2MDL_OUTX_L_REG), 8, &rawData[0]);  // Read the 6 raw data registers into data array
	
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;		 // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;	
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}


int16_t lis2mdl_readTemperature()
{
	uint8_t rawData[2];	 // x/y/z mag register data stored here
	readBytes(LIS2MDL_I2C_ADDR, (0x80 | LIS2MDL_TEMP_OUT_L_REG), 2, &rawData[0]);  // Read the 8 raw data registers into data array
	
	int16_t temp = ((int16_t)rawData[1] << 8) | rawData[0] ;	   // Turn the MSB and LSB into a signed 16-bit value
	return temp;
}

void lis2mdl_offsetBias(float * dest1, float * dest2)
{
	int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
	float _mRes = 0.0015f;
  
	NRF_LOG_INFO("Calculate mag offset bias: move all around to sample the complete response surface!");
	nrf_delay_ms(4000);

	for (int ii = 0; ii < 4000; ii++)
	{
		lis2mdl_readData(mag_temp);
		for (int jj = 0; jj < 3; jj++) {
			if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		nrf_delay_ms(12);
	}

	_mRes = 0.0015f; // fixed sensitivity
	// Get hard iron correction
	mag_bias[0]	 = (mag_max[0] + mag_min[0])/2;	 // get average x mag bias in counts
	mag_bias[1]	 = (mag_max[1] + mag_min[1])/2;	 // get average y mag bias in counts
	mag_bias[2]	 = (mag_max[2] + mag_min[2])/2;	 // get average z mag bias in counts
	
	dest1[0] = (float) mag_bias[0] * _mRes;	 // save mag biases in G for main program
	dest1[1] = (float) mag_bias[1] * _mRes;	  
	dest1[2] = (float) mag_bias[2] * _mRes;	 
	   
	// Get soft iron correction estimate
	mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
	mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
	mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0f;

	dest2[0] = avg_rad/((float)mag_scale[0]);
	dest2[1] = avg_rad/((float)mag_scale[1]);
	dest2[2] = avg_rad/((float)mag_scale[2]);
  
	NRF_LOG_INFO("Mag Calibration done!");
}

int32_t* lis2mdl_selfTest()
{
	int16_t temp[3] = {0, 0, 0};
	float magTest[3] = {0., 0., 0.};
	float magNom[3] = {0., 0., 0.};
	static int32_t sum[3] = {0, 0, 0};
	float _mRes = 0.0015f;
	
	// first, get average response with self test disabled
	for (int ii = 0; ii < 50; ii++)
	{
		lis2mdl_readData(temp);
		sum[0] += temp[0];
		sum[1] += temp[1];
		sum[2] += temp[2];
		nrf_delay_ms(50);
	}
  
	magNom[0] = (float) sum[0] / 50.0f;
	magNom[1] = (float) sum[1] / 50.0f;
	magNom[2] = (float) sum[2] / 50.0f;
  
	uint8_t c = readByte(LIS2MDL_I2C_ADDR, LIS2MDL_CFG_REG_C);
	writeByte(LIS2MDL_I2C_ADDR, LIS2MDL_CFG_REG_C, c | 0x02); // enable self test
	nrf_delay_ms(100); // let mag respond
  
	sum[0] = 0;
	sum[1] = 0;
	sum[2] = 0;
	for (int ii = 0; ii < 50; ii++)
	{
		lis2mdl_readData(temp);
		sum[0] += temp[0];
		sum[1] += temp[1];
		sum[2] += temp[2];
		nrf_delay_ms(50);
	}

    for (int i=0;i<3;i++)
    {
        magTest[i] = (float) sum[i] / 50.0f;
        sum[i] = (int32_t)((magTest[i] - magNom[i]) * _mRes * 1000.0);
    }
  
	writeByte(LIS2MDL_I2C_ADDR, LIS2MDL_CFG_REG_C, c); // return to previous settings/normal mode
	nrf_delay_ms(100); // let mag respond

	NRF_LOG_INFO("Mag Self Test:");
	NRF_LOG_INFO("Mx results: %d mG", sum[0]);
	NRF_LOG_INFO("My results: %d mG", sum[1]);
	NRF_LOG_INFO("Mz results: %d mG", sum[2]);
	NRF_LOG_INFO("Should be between 15 and 500 mG (or uT)");

    return sum;
}
