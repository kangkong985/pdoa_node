/*
 * @file usb2spi_task.h
 * @brief  header file for usb2spi_task.c
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __INC_USB2SPI_TASK_H_
#define __INC_USB2SPI_TASK_H_    1

#ifdef __cplusplus
 extern "C" {
#endif

void StartUsb2SpiTask(void const * arg);
void usb2spi_terminate_tasks(void);

#ifdef __cplusplus
}
#endif

#endif /* __INC_USB2SPI_TASK_H_ */
