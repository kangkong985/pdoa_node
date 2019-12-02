/*! ----------------------------------------------------------------------------
 * @file    lsm9ds1_spi.h
 * @brief   header for LSM9DS1 dedicated SPI access functions
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#ifndef LSM9DS1_SPI_H_
#define LSM9DS1_SPI_H_ 1

#include <stdint.h>

/* Enum used to access LSM9DS's different register sets. */
typedef enum
{
    LSM9DS1_CS_M,
    LSM9DS1_CS_XG,
} lsm9ds1_cs_e;


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn lsm9ds1_read()
 *
 * @brief This function reads data from LSM9DS1.
 *
 * input parameters
 * @param cs - Accel/Gyro or Magnet cs to use
 *        data[0...] - buffer for data to read, including register_address in the data[0]
 *        length     - length of read operation, including reg_addr field, in bytes
 *
 * output
 * data[1...] - buffer where to put the data (should be of length size)
 *
 * no return value
 */
void lsm9ds1_read(lsm9ds1_cs_e cs, uint8_t *data, uint16_t length);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn lsm9ds0_write()
 *
 * @brief This function writes some data to LSM9DS0.
 *
 * input parameters
 * @param cs - Accel/Gyro or Magnet cs to use
 *        data[0...] - buffer for data to write, including register_address in the data[0]
 *        length     - length of write operation, including reg_addr field, in bytes
 *
 * no return value
 */
 void lsm9ds1_write(lsm9ds1_cs_e cs, uint8_t *data, uint16_t length);

#endif /* LSM9DS1_SPI_H_ */
