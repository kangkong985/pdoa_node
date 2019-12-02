/*
 * @file     usb2spi_process.c
 * @brief      process to run USB2SPI Mode
 *
 *             can be used in bare-metal or RTOS systems
 *             1. select the CHIP to drive: setup_usb2spi_process(CHIP);
 *             2. execute once init_usb2spi_process();
 *             3. continuously execute the run_usb2spi_process() on reception of data;
  *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#include "dw_pdoa_node_common.h"
#include "port_platform.h"

#include <usb_uart_rx.h>
#include <usb_uart_tx.h>

#include <uart2spi.h>
////-----------------------------------------------------------------------------
///* Definitions for compatibility */
//#define USB2SPI_ENTER_CRITICAL()    taskENTER_CRITICAL()
//#define USB2SPI_EXIT_CRITICAL()     taskEXIT_CRITICAL()
//
//#define USB2SPI_MALLOC              pvPortMalloc
//#define USB2SPI_FREE                vPortFree
////-----------------------------------------------------------------------------
//
//#define SOFTWARE_VER_STRINGUSB      "PDOATAG USB2SPI 2.0"
//#define SOFTWARE_VER_STRINGUSB_EVB  "EVB1000 USB2SPI 2.0"
//
//#define SPI_ACC_BUF_SIZE    0x1100  /**< Maximum size of buffer to read whole accumulator in the the USB2SPI */
//
///* Typedefs & Enumerations */
//enum {
//    Usb_Msg_Header	= 0x02,
//    Usb_Msg_Footer  = 0x03
//};
//
///* Constants */
//const char deca[]="deca?";
//const char dwSTOP[]="stop";
//
//extern void command_stop_received(void);
//
////-----------------------------------------------------------------------------
//// local data
//typedef struct
//{
//    uint8_t     rx_buf[USB_RX_BUF_SIZE];
//    uint8_t     tx_buf[SPI_ACC_BUF_SIZE]; /**< need a long buffer for accumulator read */
//    size_t      tx_buf_length;
//    dw_name_e	chip;
//}usb2spi_t;
//
////-----------------------------------------------------------------------------
//// local data
//static usb2spi_t    *pUsb2Spi = NULL;
//
////-----------------------------------------------------------------------------
//// Implementation
//
///*
// * @brief    setup the speed for usb2spi:
// *             0 for slow spi speed and 1 for high
// **/
//static void configSPIspeed(int rate)
//{
//    static int localSPIspeed = -1;
//
//    if(localSPIspeed != rate)
//    {
//        localSPIspeed = rate;
//
//        if(rate) {
//            set_dw_spi_fast_rate(pUsb2Spi->chip);
//        }else{
//            set_dw_spi_slow_rate(pUsb2Spi->chip);
//        }
//    }
//}
//
//
///*
// * @brief     Decawave usb2spi standard protocol
// * */
//static usb_data_e usb2spi(uint8_t * pBuf , uint16_t len)
//{
//    usb_data_e result = NO_DATA;
//    //first byte specifies the SPI speed and SPI read/write operation
//    //bit 0 = 1 for write, 0 for read
//    //bit 1 = 1 for high speed, 0 for low speed
//
//    //<STX>   <ETX>
//    //
//    //to read from the device (e.g. 4 bytes), total length is = 1 + 1 + length_of_command (2) + length_of_data_to_read (2) + header length (1/2) + 1
//    //
//    //to write to the device (e.g. 4 bytes),  total length is = 1 + 1 + length_of_command (2) + length_of_data_to_write (2) + header length (1/2) + data length + 1
//    //
//    //LBS comes first:   0x2, 0x2, 0x7, 0x0, 0x04, 0x00, 0x3
//
//    if(len)
//    {
//        //0x2 = STX - start of SPI transaction data
//        if(pBuf[0] == Usb_Msg_Header)
//        {
//            configSPIspeed((pBuf[1]>>1) & 0x1);    //configure SPI speed
//
//            if((pBuf[1] & 0x1) == 0)    //SPI read
//            {
//                int msglength  = pBuf[2] + (pBuf[3]<<8);
//                int datalength = pBuf[4] + (pBuf[5]<<8);
//
//                if((datalength + 3) > (sizeof(pUsb2Spi->tx_buf)-1))
//                {
//                    return(result);
//                }
//
//                pUsb2Spi->tx_buf[0] = Usb_Msg_Header;
//                pUsb2Spi->tx_buf[1] = 0x0; // no error
//                pUsb2Spi->tx_buf[datalength+2] = Usb_Msg_Footer;
//
//                //max data we can read in a single SPI transaction is BUFFLEN as the USB/VCP tx buffer is only 4096 bytes long
//                if(pBuf[msglength-1] != Usb_Msg_Footer)
//                {
//                    pUsb2Spi->tx_buf[1] = 0x1; // if no ETX (0x3) indicate error
//                    pUsb2Spi->tx_buf[2] = Usb_Msg_Footer;
//                    datalength = 0;
//                }
//                else
//                {
//                    // do the read from the SPI
//                    readfromspi(msglength-7, &pBuf[6], datalength, &pUsb2Spi->tx_buf[2]);  // result is stored in the tx_buf
//                }
//
//                pUsb2Spi->tx_buf_length = datalength + 3;
//                result = DATA_SEND;
//            }
//            else
//            if((pBuf[1] & 0x1) == 1) //SPI write
//            {
//                int msglength = pBuf[2] + (pBuf[3]<<8);
//                int datalength = pBuf[4] + (pBuf[5]<<8);
//                int headerlength = msglength - 7 - datalength;
//
//                pUsb2Spi->tx_buf[0] = Usb_Msg_Header;
//                pUsb2Spi->tx_buf[1] = 0x0;         // no error
//                pUsb2Spi->tx_buf[2] = Usb_Msg_Footer;
//
//                if((pBuf[msglength-1] != Usb_Msg_Footer) ||
//                   (headerlength < 0))
//                {
//                    pUsb2Spi->tx_buf[1] = 0x1; // if no ETX (0x3) indicate error
//                }
//                else
//                {
//                    // do the write to the SPI
//                    writetospi(headerlength, &pBuf[6], datalength, &pBuf[6 + headerlength]);  // result is stored in the buffer
//                }
//
//                pUsb2Spi->tx_buf_length = 3;
//                result = DATA_SEND;
//            }
//            else
//            {
//            }
//        }
//
//        if (len == 5 && memcmp(pBuf , deca , 5) == 0)
//        {
//            //send a reply 'y'
//            pUsb2Spi->tx_buf[0] = 'y';
//            if(app.pConfig->s.emuEVB)
//            {
//                strcpy((char*)&pUsb2Spi->tx_buf[1], SOFTWARE_VER_STRINGUSB_EVB);
//                pUsb2Spi->tx_buf_length = strlen(SOFTWARE_VER_STRINGUSB_EVB) + 2;
//            }
//            else
//            {
//                strcpy((char*)&pUsb2Spi->tx_buf[1], SOFTWARE_VER_STRINGUSB);
//                pUsb2Spi->tx_buf_length = strlen(SOFTWARE_VER_STRINGUSB) + 2;
//            }
//
//            result = DATA_SEND;
//        }
//        else
//        if (len == 4 && memcmp(pBuf , dwSTOP , 4) == 0)
//        {
//            /* this should execute the fn_stop to exit the current process */
//
//            command_stop_received();    //communication to the user application
//
//            result = NO_DATA;
//        }
//        else
//        if(len == 1 && pBuf[0] == 114)    //r - flush the USB buffers...
//        {
//            result = DATA_FLUSH;
//        }
//    }
//
//    return result;
//}
//
//
///* @fn         usb2spi_protocol_check
// * @brief     ensure the full USB packet has been received
// * @return     DATA_READY if buffer looks like usb2spi protocol
// * */
//usb_data_e usb2spi_protocol_check(uint8_t *p, uint16_t len)
//{
//    usb_data_e ret = NO_DATA;
//
//    if((p[0] == Usb_Msg_Header &&  (p[2] + (p[3]<<8)) == len && p[len-1] == Usb_Msg_Footer) || /*general usb2spi command*/
//       (len == 5 && memcmp(p , deca , 5) == 0)       || /* "deca?" */
//       (len == 4 && memcmp(p , dwSTOP , 4) == 0))      /* "stop" */
//    {
//        ret = DATA_READY;
//    }
//
//    return    ret;
//}
//
////-----------------------------------------------------------------------------
//// IMPLEMETATION : interface to the process : bare-metal implementation
//
///*
// * @brief     exporting functions to setup the bare-metal process.
// *             fn_init - executed once.
// *             fn_run  - executed on reception of data.
// *            fn_stop - executed to terminate the process.
// * */
//
//error_e usb2spi_process_init(dw_name_e    chip)
//{
//    error_e    ret = _NO_ERR;
//
//    if(pUsb2Spi)
//    {
//        ret = _Err_Usb2Spi_ptr_busy;
//    }
//    else
//    {
//        pUsb2Spi = USB2SPI_MALLOC(sizeof(usb2spi_t));
//
//        if(!pUsb2Spi)
//        {
//            ret = _Err_Usb2Spi_ptr_alloc;
//        }
//        else
//        {
//            app.local_buff_length = 0;
//            memset(&app.local_buff, 0, sizeof(app.local_buff));
//
//            pUsb2Spi->chip = chip;
//        }
//    }
//    return (ret);
//}
//
//
///*
// * @brief     the "run" function implements usb2spi process functionality:
// *             it parses the app.local_buff: app.local_buff_length
// *
// * */
//void usb2spi_process_run(void)
//{
//    usb_data_e tmp;
//    tmp = usb2spi(app.local_buff, app.local_buff_length);
//
//    if (tmp != NO_DATA)
//    {
//        app.local_buff_length = 0;
//
//        if (pUsb2Spi->tx_buf_length)
//        {
//            port_tx_msg(pUsb2Spi->tx_buf, pUsb2Spi->tx_buf_length);
//        }
//    }
//}
//
//
///*
// * @brief     stop function implements the stop functionality if any suitable for current process
// *             which will be executed on reception of Stop command
// * */
//void usb2spi_process_terminate(void)
//{
//    if(pUsb2Spi)
//    {
//        USB2SPI_ENTER_CRITICAL();
//
//        port_stop_all_UWB();
//
//        strcpy((char*)pUsb2Spi->tx_buf, "Ok\r\n");
//
//        pUsb2Spi->tx_buf_length = strlen((char*)pUsb2Spi->tx_buf);
//
//        port_tx_msg(pUsb2Spi->tx_buf, pUsb2Spi->tx_buf_length);
//
//        USB2SPI_FREE(pUsb2Spi);
//        pUsb2Spi = NULL;
//
//        USB2SPI_EXIT_CRITICAL();
//    }
//}
//
////-----------------------------------------------------------------------------

#define UART2SPI_ENTER_CRITICAL() taskENTER_CRITICAL()
#define UART2SPI_EXIT_CRITICAL() taskEXIT_CRITICAL()

#define UART2SPI_MALLOC pvPortMalloc
#define UART2SPI_FREE vPortFree

#define SOFTWARE_VER_STRINGUSB "PD NODE USB2SPI 2.0"
#define SOFTWARE_VER_STRINGUSB_EVB "EVB1000 USB2SPI 2.0"

#define SPI_ACC_BUF_SIZE 0x1100 /*Maxium size of buffer to read whole accumulator in the UART2SPI*/

enum {
  Uart_Msg_Header = 0x02,
  Uart_Msg_Footer = 0x03,
};

const char deca[] = "deca?";
const char dwSTOP[] = "stop";

extern void command_stop_received(void);

#ifndef UART_RX_BUF_SIZE
#define UART_RX_BUF_SIZE 64
#endif

typedef struct
{
  uint8_t rx_buf[UART_RX_BUF_SIZE];
  uint8_t tx_buf[SPI_ACC_BUF_SIZE]; /*need a long buffer for accumulator read*/
  size_t tx_buf_length;
  dw_name_e chip;
} uart2spi_t;

//local data
static uart2spi_t *pUart2Spi = NULL;

//Implementation

/*
 *  @brief  setup the speed for uart2spi:
 *          0 for slow spi speed and 1 for high
 *
 */
static void configSPIspeed(int rate) {
  static int localSPIspeed = -1;
  if (localSPIspeed != rate) {
    localSPIspeed = rate;

    if (rate) {
      set_dw_spi_fast_rate(pUart2Spi->chip);
    } else {
      set_dw_spi_slow_rate(pUart2Spi->chip);
    }
  }
}

/*
 *  @brief  Decawanve uart2spi standard protocol
 *
 *
 */
static uart_data_e uart2spi(uint8_t *pBuf, uint16_t len) {
  uart_data_e result = NO_DATA;
  //first byte specifies the SPI speed and SPI read/write operation
  //bit 0 = 1 for write, 0 for read
  //bit 1 = 1 for high speed, 0 for low speed

  //<STX>   <ETX>
  //
  //to read from the device (e.g. 4 bytes), total length is = 1 + 1 + length_of_command (2) + length_of_data_to_read (2) + header length (1/2) + 1
  //
  //to write to the device (e.g. 4 bytes),  total length is = 1 + 1 + length_of_command (2) + length_of_data_to_write (2) + header length (1/2) + data length + 1
  //
  //LBS comes first:   0x2, 0x2, 0x7, 0x0, 0x04, 0x00, 0x3

  if (len) {
    if (pBuf[0] == Uart_Msg_Header) {
      configSPIspeed((pBuf[1] >> 1) & 0x1);

      if ((pBuf[1] & 0x1) == 0) //SPI read
      {
        int msgLength = pBuf[2] + (pBuf[3] << 8);
        int dataLength = pBuf[4] + (pBuf[5] << 8);

        if ((dataLength + 3) > (sizeof(pUart2Spi->tx_buf) - 1)) {
          return result;
        }

        pUart2Spi->tx_buf[0] = Uart_Msg_Footer;
        pUart2Spi->tx_buf[1] = 0x0; //no error
        pUart2Spi->tx_buf[dataLength + 2] = Uart_Msg_Footer;

        //max data we can read in a single SPI transaction is BUFFLEN as the tx buff is only 4096 bytes long
        if (pBuf[msgLength - 1] != Uart_Msg_Footer) {
          pUart2Spi->tx_buf[1] = 0x1; //if no ETX (0x3) indicate error
          pUart2Spi->tx_buf[2] = Uart_Msg_Footer;
          dataLength = 0;
        } else {
          readfromspi(msgLength - 7, &pBuf[6], dataLength, &pUart2Spi->tx_buf[2]);
        }

        pUart2Spi->tx_buf_length = dataLength + 3;
        result = DATA_SEND;
      } else if ((pBuf[1] & 0x1) == 1) { //spi write
        int msgLength = pBuf[2] + (pBuf[3] << 8);
        int dataLength = pBuf[4] + (pBuf[5] << 8);
        int headerLength = msgLength - 7 - dataLength;

        pUart2Spi->tx_buf[0] = Uart_Msg_Header;
        pUart2Spi->tx_buf[1] = 0x0;
        pUart2Spi->tx_buf[2] = Uart_Msg_Footer;

        if ((pBuf[msgLength - 1] != Uart_Msg_Footer) || (headerLength < 0)) {
          pUart2Spi->tx_buf[1] = 0x1;
        } else {
          //do the write to the SPI
          writetospi(headerLength, &pBuf[6], dataLength, &pBuf[6 + headerLength]);
        }

        pUart2Spi->tx_buf_length = 3;
        result = DATA_SEND;
      } else {
      }
    }

    if (len == 5 && memcmp(pBuf, deca, 5) == 0) {
      //send a reply 'y'
      pUart2Spi->tx_buf[0] = 'y';
      if (app.pConfig->s.emuEVB) {
        strcpy((char *)&pUart2Spi->tx_buf[1], SOFTWARE_VER_STRINGUSB_EVB);
        pUart2Spi->tx_buf_length = strlen(SOFTWARE_VER_STRINGUSB_EVB) + 2;
      } else {
        strcpy((char *)&pUart2Spi->tx_buf[1], SOFTWARE_VER_STRINGUSB);
        pUart2Spi->tx_buf_length = strlen(SOFTWARE_VER_STRINGUSB) + 2;
      }
      result = DATA_SEND;
    } else if (len == 4 && memcmp(pBuf, dwSTOP, 4) == 0) {
      //this should execute the fn_stop to exit the current process
      command_stop_received(); //communication to the user application
      result = NO_DATA;
    } else if (len == 1 && pBuf[0] == 114) {
      result = DATA_FLUSH;
    }
  }
}

/*  @fn   uart2spi_protocol_check
 *  @brief ensure the full UART packet has been received
 *  @return DATA_READY if buffer looks like uart2spi protocol
 */
uart_data_e uart2spi_protocol_check(uint8_t *p, uint16_t len)
{
    uart_data_e ret = NO_DATA;

    if ((p[0] == Uart_Msg_Header && (p[2] + (p[3] << 8)) == len && p[len - 1] == Uart_Msg_Footer) || 
        (len == 5 && memcmp(p, deca, 5) == 0) ||
        (len == 4 && memcmp(p, dwSTOP, 4) == 0))
    {
        ret = DATA_READY;
    }

    return ret;
}

//IMPLEMENTATION : interface to the process : bare-metal inplementation

/*
 *  @brief    exporting functions to setup the bare-metal process
 *            fn_init - executed once
 *            fn_run  - executed on reception of data
 *            fn_stop - executedto terminate the process
 **/
error_e uart2spi_process_init(dw_name_e chip)
{
    error_e ret = _NO_ERR;

    if (pUart2Spi)
    {
        ret = _Err_Uart2Spi_ptr_busy;
    }
    else
    {
        pUart2Spi = UART2SPI_MALLOC(sizeof(uart2spi_t));

        if (!pUart2Spi)
        {
            ret = _Err_Uart2Spi_ptr_alloc;
        }
        else
        {
            app.local_buff_length = 0;
            memset(&app.local_buff, 0, sizeof(app.local_buff));

            pUart2Spi->chip = chip;
        }
    }
    return ret;
}

/**
 *    @brief    the "run" function implements uart2spi process functionality:
 *              it parses the app.local_buff: app.local_buff_length
 *
 *
 **/

void uart2spi_process_run(void)
{
    uart_data_e tmp;
    tmp = uart2spi(app.local_buff, app.local_buff_length);

    if (tmp != NO_DATA)
    {
        app.local_buff_length = 0;

        if (pUart2Spi->tx_buf_length)
        {
            port_tx_msg(pUart2Spi->tx_buf, pUart2Spi->tx_buf_length);
            printf("hello");
        }
    }
}

/*
 *  @brief    stop function implements the stop functionality if any suitable for current process
 *            which will be executed on reception of stop command
 */
void uart2spi_process_terminate(void)
{
    if (pUart2Spi)
    {
        UART2SPI_ENTER_CRITICAL();
        
        port_stop_all_UWB();
                
        strcpy((char *)pUart2Spi->tx_buf, "OK\r\n");

        pUart2Spi->tx_buf_length = strlen((char *)pUart2Spi->tx_buf);

        port_tx_msg(pUart2Spi->tx_buf, pUart2Spi->tx_buf_length);

        UART2SPI_FREE(pUart2Spi);
        pUart2Spi = NULL;

        UART2SPI_EXIT_CRITICAL();
    }
}