/*! ---------------------------------------------------------------------------
 * @file    node.h
 * @brief   DecaWave
 *             bare implementation layer
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#ifndef __NODE__H__
#define __NODE__H__ 1

#ifdef __cplusplus
 extern "C" {
#endif

#include "uwb_frames.h"
#include "msg_time.h"
#include "tag_list.h"

#include "port_platform.h"
#include "dw_pdoa_node_common.h"

//typedef struct
//{
//	uint16_t head;	   
//	uint8_t  len;   
//	uint8_t cmd;
//	uint16_t x_cm;
//	uint16_t y_cm;
//	uint16_t d_cm;
//	uint16_t degre;
//    uint8_t	 battery_power; //HHHHHH
//	uint8_t	 button_pushed; //HHHHHH
//	uint16_t tag_ID;		//HHHHHH
//	uint8_t  base_status;	//HHHHHH
//	uint8_t  data_quality;	//HHHHHH
//	uint16_t crc16;
//}pdoa_hex_pack_t;

typedef struct
{
	uint16_t head;	   
	uint8_t  len;   
	uint8_t	 cmd;
	uint16_t x_cm;
	uint16_t y_cm;
//	uint8_t	 signal_strength[2];
	uint8_t  data_quality;	//HHHHHH
    uint8_t	 battery_power; //HHHHHH
	uint8_t	 key_value; //HHHHHH
	uint16_t tag_ID;		//HHHHHH
	uint8_t  base_status;	//HHHHHH
	uint8_t  reserved[2];
	uint16_t crc16;
}__attribute__ ((packed))pdoa_hex_pack_t;

//-----------------------------------------------------------------------------
// Definitions
#define FULL_ACC_LEN      (1016)
#define ACC_OFFSET        (300)

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/* WKUP timer counts Super Frame period.
 * The WKUP timer resolution is (30517.5) counts in 1 ns.
 */
#define WKUP_RESOLUTION_NS          (1e9f/32768.0f)

/* RTC WKUP timer counts Super Frame period.
 * The RTC WKUP timer prescaler is configured as each Tick count is 30.517 us.
 */
#define RTC_WKUP_PRESCALER          (0)

/* RTC WKUP timer counts Super Frame period.
 * The RTC WKUP timer is 24 bit counter. Counter oveflows at 2^24 - 16777216
 */
#define RTC_WKUP_CNT_OVFLW          (16777216)
#define SPEED_OF_LIGHT              (299702547.0)     // in m/s in the air
#define DWT_DIAGNOSTIC_LOG_REV_5    (5)
#define INVALID_TOF                 (0xABCDFFFF)

/* Bitmask for result_t flag's.
 * Note, byte_0 is bitmask from a tag;
 * byte_1 is other flags:
 * */
#define RES_FLAG_PDOA_OFFSET_ZERO_BIT   (1<<15) /**< This bit will be set to the report when the "pdoa_offset" is zero */
#define RES_FLAG_RANGE_OFFSET_ZERO_BIT  (1<<14) /**< This bit will be set to the report when the "rng_offset" is zero */

//-----------------------------------------------------------------------------
/*
 * Rx Events circular buffer : used to transfer RxPckt from ISR to APP
 * 0x02, 0x04, 0x08, 0x10, etc.
 * As per design, the amount of RxPckt in the buffer at any given time shall not be more than 1.
 * */
#define EVENT_BUF_SIZE      (0x02)


//-----------------------------------------------------------------------------
/*
 * RX_MAIL_QUEUE_SIZE : used to transfer RxPckt from rxTask() to calkTask()
 * it shall be smaller than bare-metal circular buffer EVENT_BUF_SIZE
 * See note to osMailQDef(rxPcktPool_q, RX_MAIL_QUEUE_SIZE, rx_mail_t);
 * */
#define RX_MAIL_QUEUE_SIZE  (0x01)

//-----------------------------------------------------------------------------
// Typedefs

typedef struct
{
    uint8_t sfdangle_master;
    uint8_t sfdangle_slave;
    uint8_t acc_master[5];
    uint8_t acc_slave[5];
}pdoa_info_t;


/* */
typedef struct
{
    uint16_t    addr16;
    uint8_t     rangeNum;       //number from Tag Poll and Final messages, which indicates the current range number
    uint32_t    resTime_us;     //reception time of the end of the Final from the Tag wrt node's SuperFrame start, microseconds
    float       pdoa_raw_deg;  //pdoa_raw: phase differences in degrees without any correction [-180 .. 180]
    float       dist_cm;        //FIXME:remove: distance to the tag in cm, corrected to a rngOffset_mm
    float       x_cm;           //X of the tag wrt to the node, cm
    float       y_cm;           //Y of the tag wrt to the node, cm
    float       clockOffset_pphm;//clock offset in hundredths of ppm (i.e. 1ppm is 100)
    uint16_t    flag;           //service message data from the tag (low byte) and node (high byte), bitmask (defined as "RES_FLAG_")
    int16_t     acc_x;          //Normalized accel data X from the Tag, mg: acc_x
    int16_t     acc_y;          //Normalized accel data Y from the Tag, mg: acc_y
    int16_t     acc_z;          //Normalized accel data Z from the Tag, mg: acc_z
    int8_t      tMaster_C;      //temperature of Master in degree centigrade
    int8_t      tSlave_C;       //temperature of Master in degree centigrade
}result_t;


/* Standard Diagnostics v5 */
typedef struct
{
    //NOTE: diagnostics data format rev 5 (DWT_DIAGNOSTIC_LOG_REV_5)
    uint8_t        header;  //00 this could be a header (format version number)
    uint8_t        r0F[ 5]; //01 register 0xF  - length 5 bytes
    uint8_t        r10[ 4]; //06 register 0x10 - length 4 bytes
    uint8_t        r12[ 8]; //10 register 0x12 - length 8 bytes
    uint8_t        r13[ 4]; //18 register 0x13 - length 4 bytes
    uint8_t        r14[ 5]; //22 register 0x14 - length 5 bytes
    uint8_t        r15[14]; //27 register 0x15 - length 14 bytes (5 TS, 2 FP, 2 Diag, 5 TSraw)
    uint8_t        r25[16]; //41 register 0x25 @FP (first path) -> 16 bytes starting at FP + 1 dummy
    uint8_t        r2E[ 2]; //58 register 0x2E (0x1000) - 2 bytes
    uint8_t        r27[ 4]; //60 register 0x27 (0x28)   - 4 bytes
    uint8_t        r2E2[2]; //64 register 0x2E (0x1002) - 2 bytes
    uint8_t        dummy;
    //66 total
}__attribute__((packed))
diag_v5_t;


/* TxPckt */
typedef struct {
    int16_t        psduLen;     // Length of msg to send

    union {
        std_msg_t           stdMsg;
        twr_msg_t           twrMsg;
        rng_cfg_msg_t       rngCfgMsg;
        rng_cfg_upd_msg_t   rngCfgUpdMsg;
        resp_pdoa_msg_t     respMsg;
    } msg;

    uint8_t        txFlag;      /* Holds Tx sending parameters:
                                 * DWT_START_TX_IMMEDIATE DWT_START_TX_DELAYED & DWT_RESPONSE_EXPECTED
                                 */
    uint32_t    delayedTxTimeH_sy;  // Delayed transmit time (high32)
    uint32_t    delayedRxTime_sy;   // Delay after Tx when to switch on receiver
    uint16_t    delayedRxTimeout_sy;// How long the receiver will be switched on after Tx

}tx_pckt_t;


/* RxPckt */
typedef struct {
    int16_t        rxDataLen;

    union {
        std_msg_t           stdMsg;
        twr_msg_t           twrMsg;
        blink_msg_t         blinkMsg;
        rng_cfg_msg_t       rngCfgMsg;
        poll_msg_t          pollMsg;
        resp_pdoa_msg_t     respMsg;
        final_msg_accel_t   finalMsg;
    } msg;

    tag_addr_slot_t        *tag;                /* the tag, to which the current range exchange is performing */

    uint8_t     nodePollRx_ts[TS_40B_SIZE];     /**< Placeholder to copy current range exchange pollRx time on reception of Final message */
    uint8_t     nodeRespTx_ts[TS_40B_SIZE];     /**< Placeholder to copy current range exchange respTx time on reception of Final message */

    uint8_t     timeStamp_Master[TS_40B_SIZE];  /* TimeStamp of current "Master" chip: blinkRx, pollRx, finalRx */
    uint8_t     timeStamp_Slave [TS_40B_SIZE];  /* TimeStamp of current "Slave" chip : finalRx. This is != TimeStamp of FinalRx (Master) */
    uint32_t    rtcTimeStamp;                   /* MCU Rx RTC timestamp */

    pdoa_info_t pdoa_info;
    uint8_t     temperature_Master;             /* raw reading of temperature: valid at reception of Final */
    uint8_t     temperature_Slave;              /* raw reading of temperature: valid at reception of Final */

    /* Below is Decawave's diagnostics information */
    uint32_t    status_Master;
    uint32_t    status_Slave;
    diag_v5_t   diagnostics[2];                         /* 2x 66 bytes */
    uint8_t     acc[2][(FULL_ACC_LEN-ACC_OFFSET)*4 +1]; /* Will be started with offset ACC_OFFSET :
                                                         * for ACC_OFFSET = 0
                                                         * 2x ((1016-0)*4 + 1)= 2x 4065 = 8130 bytes for raw ACC
                                                         *
                                                         * for ACC_OFFSET = 300
                                                         * 2x ((1016-300)*4 + 1)= 2x 2865 = 5730 bytes for raw ACC
                                                         * */
}rx_pckt_t;

/* Mail from RxTask to CalcTask*/
typedef struct
{
    result_t    res;

    tag_addr_slot_t *tag;       /* the tag, to which the current range was performed */

    uint8_t     tagPollTx_ts    [TS_40B_SIZE];
    uint8_t     tagRespRx_ts    [TS_40B_SIZE];
    uint8_t     tagFinalTx_ts   [TS_40B_SIZE];

    uint8_t     nodePollRx_ts   [TS_40B_SIZE];
    uint8_t     nodeRespTx_ts   [TS_40B_SIZE];
    uint8_t     nodeFinalRx_ts  [TS_40B_SIZE];

    pdoa_info_t pdoa_info;

    /* Below is Decawave's diagnostics information */
    diag_v5_t   diagnostics[2];                         /* 2x 66 bytes */
    uint8_t     acc[2][(FULL_ACC_LEN-ACC_OFFSET)*4 +1]; /* Will be started with offset ACC_OFFSET :
                                                         * for ACC_OFFSET = 0
                                                         * 2x ((1016-0)*4 + 1)= 2x 4065 = 8130 bytes for raw ACC
                                                         *
                                                         * for ACC_OFFSET = 300
                                                         * 2x ((1016-300)*4 + 1)= 2x 2865 = 5730 bytes for raw ACC
                                                         * */
}rx_mail_t;


/* This structure holds Node's TWR application parameters */
typedef struct
{
    /* Unique short Address, uses at the ranging phase
     * valid for low-endian compiler.
     * */
    union    {
       uint8_t     euiShort[2];
       uint16_t    eui16;
    };

    /* circular Buffer of received Rx packets :
     * uses in transferring of the data from ISR to APP level.
     * */
    struct {
        rx_pckt_t   buf[EVENT_BUF_SIZE];
        int         head;
        int         tail;
    } rxPcktBuf;

    /* ranging run-time variables */
    struct {
        /* MAC sequence number, increases on every tx_start */
        uint8_t        seqNum;

        /* Node's Discovery phase : Tx time structures for DW_TX_IRQ callback */
        struct {
            uint8_t  rangeInitTx_ts[TS_40B_SIZE];   /**< node: rangeInitTx_ts, rangeInitRtcTimeStamp */
            uint32_t rangeInitRtcTimeStamp;         /**< handles the MCU RTC time at the DW_IRQ */
        };

        /* Node's Ranging phase : Tx time structures for DW_TX_IRQ callback */
        struct {
            uint8_t  nodeRespTx_ts[TS_40B_SIZE];    /**< node: nodeRespTx_ts, respRtcTimeStamp */
            uint32_t respRtcTimeStamp;          /**< handles the MCU RTC time at the DW_IRQ */
        };

        /* Node's Ranging phase : Rx time structures for DW_TX_IRQ callback */
        struct {
            uint8_t  nodePollRx_ts[TS_40B_SIZE];    /**< temporary for current range exchange: received time of poll message */
            uint32_t pollRtcTimeStamp;              /**< temporary for current range exchange: received time of poll message RTC time */
        };

        /* node is ranging to a single tag in a range exchange sequence */
        tag_addr_slot_t *pDestTag;              /**< tag, with whom the current range is performing */

       /* Application DW_TX_IRQ source indicator */
        enum {
            Twr_Tx_Blink_Sent,          //tag sends blink
            Twr_Tx_Range_Config_Sent,   //node sends ranging config
            Twr_Tx_Poll_Sent,           //tag sends poll
            Twr_Tx_Resp_Sent,           //node sends response
            Twr_Tx_Final_Sent,          //tag sends final
        }
        txState;
    };

    /* pre-calculated times for different messages */
    struct {
        msg_time_t    blink;
        msg_time_t    ranging_config;
        msg_time_t    poll;
        msg_time_t    response;
        msg_time_t    final;
    }msg_time;

    uint16_t    panID;
    sfConfig_t  *pSfConfig;             //superFrame configuration

    uint64_t    newTag_addr64;          //new discovered tag address

    result_t    result[MAX_KNOWN_TAG_LIST_SIZE]; /* range/angle/offset result to the tag */

    volatile uint32_t gRtcSFrameZeroCnt;//SuperFrame Cnt RTC timestamp

    uint32_t    lateTxCount;            //indicate that Delayed Tx timings failed

    struct{
        int imuOn         : 1;
        int stationary    : 1;
    };

    uint8_t     TmeasMaster;            //copy of Temperature calibration value for current Master chip
    uint8_t     TmeasSlave;             //copy of Temperature calibration value for current Slave chip
} twr_info_t;

typedef enum {
	TURNING_ZERO = 0,
	TURNING_ONE,
	TURNING_TWO,
	TURNING_THREE,
	TURNING_FOUR,
	TURNING_FIVE,
}TURNING_LEVEL;

typedef struct {
	TURNING_LEVEL level;
	uint8_t head_X;
	uint8_t cmd_X;
	uint8_t data_X;
	uint8_t verify_X;
	uint8_t head_x;
	uint8_t cmd_x;
	uint8_t data_x;
	uint8_t verify_x;
	uint8_t head_Y;
	uint8_t cmd_Y;
	uint8_t data_Y;
	uint8_t verify_Y;
	uint8_t head_y;
	uint8_t cmd_y;
	uint8_t data_y;
	uint8_t verify_y;
	uint8_t end;
}__attribute__ ((packed))wheelchair_t;

/* enumeration of function codes used in TWR protocol */
typedef enum {
    Twr_Fcode_Not_Defined       = 0xFF, // Special : nothing
    Twr_Fcode_Blink             = 0xEE, // Special : Blink
    Twr_Fcode_Rng_Config        = 0x20, // Responder (Node) Ranging Config message          : reply to blink
    Twr_Fcode_Tag_Poll          = 0x84, // Initiator (Tag)  Poll message                    : twr start message
    Twr_Fcode_Resp_Ext          = 0x72, // Responder (Node) Response Extended               : reply to Poll with X/Y previous results
    Twr_Fcode_Tag_Final         = 0x88, // Initiator (Tag)  Final message back to Responder : reply to Response
    Twr_Fcode_Tag_Accel_Final   = 0x89, // Initiator (Tag)  Final message back to Responder : reply to Response + Accelerometer data
}fcode_e;


enum {
    Head_Msg_BLINK      =   0xC5,
    Head_Msg_STD        =   (0x40 | 0x01),
    Head_Msg_STD1       =   (0x40 | 0x20 | 0x01),
    Frame_Ctrl_SS       =   (0x80 | 0x08),    //Message addressing SS
    Frame_Ctrl_LS       =   (0x80 | 0x0C),    //Dest addr long, Source address short
    Frame_Ctrl_MASK     =   0xCC
};


typedef enum {
    Sync_Mode_Off,
    Sync_Mode_Ostr,
    Sync_Mode_Osts
}sync_mode_e;


//-----------------------------------------------------------------------------
// exported functions prototypes
//
extern twr_info_t * getTwrInfoPtr(void);

//-----------------------------------------------------------------------------
// exported functions prototypes
//

/* responder (node) */
error_e twr_responder_algorithm_rx(rx_pckt_t *pRxPckt, twr_info_t *pTwrInfo);
void    twr_configure_rtc_wakeup(uint32_t     period);
void    synchronize_DW1000clocks(void);

error_e node_process_init(int masterChip);
void    node_process_start(void);
void    node_process_terminate(void);

error_e tof2range(param_block_t*, float*, int32_t);
void    pdoa2XY(result_t *, float);
float   calcPD(pdoa_info_t *pA);

//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif

#endif /* __NODE__H__ */
