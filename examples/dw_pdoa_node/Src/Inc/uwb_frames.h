/* @file    uwb_frames.h
 * @brief
 *          UWB message frames definitions and typedefs
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __UWB_FRAMES__H__
#define __UWB_FRAMES__H__ 1

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define STANDARD_FRAME_SIZE         127

#define ADDR_BYTE_SIZE_L            (8)
#define ADDR_BYTE_SIZE_S            (2)

#define TS_40B_SIZE                 (5)
#define TS_UWB_SIZE                 (5)

#define FRAME_CONTROL_BYTES         2
#define FRAME_SEQ_NUM_BYTES         1
#define FRAME_PANID                 2
#define FRAME_CRC                   2
#define FRAME_SOURCE_ADDRESS_S      (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S        (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L      (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L        (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP                 (FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID)         /* 5 */
#define FRAME_CTRL_AND_ADDRESS_L    (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP)     /* 21 bytes for 64-bit addresses) */
#define FRAME_CTRL_AND_ADDRESS_S    (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP)     /* 9 bytes for 16-bit addresses) */
#define FRAME_CTRL_AND_ADDRESS_LS   (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP)     /* 15 bytes for 1 16-bit address and 1 64-bit address) */
#define MAX_USER_PAYLOAD_STRING_LL  (STANDARD_FRAME_SIZE-FRAME_CTRL_AND_ADDRESS_L-FRAME_CRC)          /* 127 - 21 - 2 = 104 */
#define MAX_USER_PAYLOAD_STRING_SS  (STANDARD_FRAME_SIZE-FRAME_CTRL_AND_ADDRESS_S-FRAME_CRC)          /* 127 - 9 - 2 = 116 */
#define MAX_USER_PAYLOAD_STRING_LS  (STANDARD_FRAME_SIZE-FRAME_CTRL_AND_ADDRESS_LS-FRAME_CRC)         /* 127 - 15 - 2 = 110 */

#define FRAME_DEST_ADDRESS_S_IDX     (FRAME_CTRLP)
#define FRAME_SRC_ADDRESS_S_IDX     (FRAME_CTRLP + ADDR_BYTE_SIZE_S)
//NOTE: the user payload assumes that there are only 88 "free" bytes to be used for the user message (it does not scale according to the addressing modes)
#define MAX_USER_PAYLOAD_STRING        MAX_USER_PAYLOAD_STRING_LL

#define RC_VERSION_PDOA            (3)


/* UWB packet types : MAC headers */
typedef struct
{
        uint8_t frameCtrl[2];                   //  frame control bytes 00-01
        uint8_t seqNum;                         //  sequence_number 02
        uint8_t panID[2];                       //  PAN ID 03-04
        uint8_t destAddr[ADDR_BYTE_SIZE_S];     //  05-06
        uint8_t sourceAddr[ADDR_BYTE_SIZE_S];   //  07-08
}__attribute__((packed)) mac_header_ss_t;

typedef struct
{
        uint8_t frameCtrl[2];                   // frame control bytes 00-01
        uint8_t seqNum;                         // sequence_number 02
        uint8_t panID[2];                       // PAN ID 03-04
        uint8_t destAddr[ADDR_BYTE_SIZE_L];     // 05-12 or using 64 bit addresses (05-12)
        uint8_t sourceAddr[ADDR_BYTE_SIZE_L];   // 13-20 or using 64 bit addresses (13-20)
}__attribute__((packed))
mac_header_ll_t;

typedef struct
{
        uint8_t frameCtrl[2];                   // frame control bytes 00-01
        uint8_t seqNum;                         // sequence_number 02
        uint8_t panID[2];                       // PAN ID 03-04
        uint8_t destAddr[ADDR_BYTE_SIZE_L];     // 05-12 using 64 bit addresses
        uint8_t sourceAddr[ADDR_BYTE_SIZE_S];   // 13-14
}__attribute__((packed))
mac_header_ls_t;

typedef struct
{
        uint8_t frameCtrl[2];                   // frame control bytes 00-01
        uint8_t seqNum;                         // sequence_number 02
        uint8_t panID[2];                       // PAN ID 03-04
        uint8_t destAddr[ADDR_BYTE_SIZE_S];     // 05-06
        uint8_t sourceAddr[ADDR_BYTE_SIZE_L];   // 7-14 using 64 bit addresses
}__attribute__((packed))
mac_header_sl_t;

/* General UWB packet types : Messages */
typedef struct
{
    mac_header_ll_t mac;
    uint8_t         messageData[MAX_USER_PAYLOAD_STRING_LL];// 21-124 (application data and any user payload)
    uint8_t         fcs[2]; // 125-126  we allow space for the CRC as it is logically part of the message.
                            // DW1000 calculates and adds these bytes.
}__attribute__((packed))
std_msg_ll_t;

typedef struct
{
    mac_header_ss_t mac;
    uint8_t         messageData[MAX_USER_PAYLOAD_STRING_SS];// 09-124 (application data and any user payload)
    uint8_t         fcs[2];
}__attribute__((packed))
std_msg_ss_t;

typedef struct
{
    mac_header_ls_t mac;
    uint8_t         messageData[MAX_USER_PAYLOAD_STRING_LS];// 15-124 (application data and any user payload)
    uint8_t         fcs[2];
}__attribute__((packed))
std_msg_ls_t;

//12 octets for Minimum IEEE ID blink
typedef struct
{
    uint8_t         frameCtrl[1];           //  frame control bytes 00
    uint8_t         seqNum;                 //  sequence_number 01
    uint8_t         tagID[ADDR_BYTE_SIZE_L];//  02-09 64 bit addresses
    uint8_t         fcs[2];
}__attribute__((packed))
blink_msg_t;


/* UWB packet types : Application-specific Data fields */

/* Ranging Config rest of configuration.
 * The Node's address and PanId, are in the MAC header */
typedef struct
{
    /* Compatibility to the EVK's Ranging Init */
    uint8_t fCode;
    uint8_t tagAddr[ADDR_BYTE_SIZE_S];  //tag's short address
    uint8_t ANC_RESP_DLY[2];            //backward compatibility to EVK1000 RI message : delayRx_us + poll_us, coded in us with bit 15 == 1
    uint8_t TAG_RESP_DLY[2];            //backward compatibility to EVK1000 RI message : , coded in us with bit 15 == 1

    /* PDOA TWR unique message */
    uint8_t version;                    //version

    uint8_t sframePeriod_ms   [2];  //Super Frame period, ms
    uint8_t pollTxToFinalTx_us[2];  //roof time from the RMARKER of Poll to the RMARKER of Final that the Tag shall set.
    uint8_t delayRx_us   [2];       //roof time from end of transmission of the Poll to start of reception of Response, that the Tag shall set.
    uint8_t slotCorr_us  [4];       //Slot correction for current reception (i.e. for blink), us
    uint8_t pollMultFast [2];       //multiplier for fast ranging in Super Frame counts
    uint8_t pollMultSlow [2];       //multiplier for slow ranging in Super Frame counts

    union
    {
        uint8_t mode     [2];       //bitfields for mode of operation: IMU on/off, etc.
        struct	
        {
            uint8_t imuOn : 1;      //currently only IMU ON switch is defined/used
        };
    };
}__attribute__((packed))
rng_cfg_t;

typedef struct
{
    uint8_t fCode;                          // msgdata+0
    uint8_t rNum;
}__attribute__((packed))
poll_t;

typedef struct
{
        uint8_t fCode;                      // msgdata+0
        uint8_t slotCorr_us     [4];
        uint8_t rNum;
        uint8_t x_cm            [2];        // X coordinate of Tag wrt Node in centimeters [0..65535cm]
        uint8_t y_cm            [2];        // Y coordinate of Tag wrt Node in centimeters [0..65535cm]
        uint8_t clkOffset_pphm  [2];        // part per (hundreds of millions) = (100*ppm)
}__attribute__((packed))
resp_tag_t;

typedef struct
{
        uint8_t fCode;                      // msgdata+0
        uint8_t slotCorr_us     [4];
        uint8_t rNum;
        uint8_t x_cm            [2];        // X in centimeters [0..65535cm]
        uint8_t y_cm            [2];        // Y in centimeters [0..65535cm]
        uint8_t clkOffset_pphm  [2];        //part per (hundreds of millions) = (100*ppm)
}__attribute__((packed))
resp_ext_t;

typedef struct
{
    uint8_t fCode;                          //msgdata+0
    uint8_t rNum;
    uint8_t pollTx_ts    [TS_UWB_SIZE];
    uint8_t responseRx_ts[TS_UWB_SIZE];
    uint8_t finalTx_ts   [TS_UWB_SIZE];
    uint8_t flag;                           //1 data bytes bitfields. IMU:0
    uint8_t acc_x [2];                      //Normalized accel data X from the Tag, mg
    uint8_t acc_y [2];                      //Normalized accel data Y from the Tag, mg
    uint8_t acc_z [2];                      //Normalized accel data Z from the Tag, mg
}__attribute__((packed))
final_accel_t;


/* UWB packet types : Application-specific messages */

/* Ranging Config message during Discovery
 * the Node's address and PanId are in the MAC header */
typedef struct
{
    mac_header_ls_t mac;
    rng_cfg_t       rngCfg;
    uint8_t         fcs[2];
}__attribute__((packed))
rng_cfg_msg_t;

typedef struct
{
    mac_header_ss_t mac;
    rng_cfg_t       rngCfg;
    uint8_t         fcs[2];
}__attribute__((packed))
rng_cfg_upd_msg_t;

typedef struct
{
    mac_header_ss_t mac;
    poll_t          poll;
    uint8_t         fcs[2];
}__attribute__((packed))
poll_msg_t;

typedef struct
{
    mac_header_ss_t mac;
    resp_ext_t      resp;
    uint8_t         fcs[2];
}__attribute__((packed))
resp_ext_msg_t;

typedef struct
{
    mac_header_ss_t mac;
    resp_tag_t     resp;
    uint8_t         fcs[2];
}__attribute__((packed))
resp_pdoa_msg_t;

typedef struct
{
    mac_header_ss_t mac;
    final_accel_t   final;
    uint8_t         fcs[2];
}__attribute__((packed))
final_msg_accel_t;


typedef std_msg_ll_t std_msg_t;
typedef std_msg_ss_t twr_msg_t;

#ifdef __cplusplus
}
#endif

#endif /* __UWB_FRAMES__H__ */
