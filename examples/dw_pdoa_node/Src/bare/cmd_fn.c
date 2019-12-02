/*
 * @file     cmd_fn.c
 * @brief    collection of executables functions from defined known_commands[]
 *
 * @author   Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */

#include "cmd_fn.h"
#include "translate.h"
#include "version.h"
#include "deca_version.h"
#include "config.h"
#include "task_flush.h"

#include "deca_regs.h"
#include "deca_device_api.h"

#include "util.h"

//#include "iot_file.h"

//-----------------------------------------------------------------------------
const char CMD_FN_RET_OK[] = "ok\r\n";

const char CMD_FN_RET_FAIL[] = "failed\r\n";

REG_FN(f_temp);
/****************************************************************************//**
 *
 *                          f_xx "command" FUNCTIONS
 *
 * REG_FN(f_node) macro will create a function
 *
 * const char *f_node(char *text, param_block_t *pbss, int val)
 *
 * */

//-----------------------------------------------------------------------------
// Operation Mode change section

/* @brief    helperTask will start
 * 0 - NODE thread using chip A as master
 * 1 - NODE thread using chip B as master
 * app.mode will be mTwr
 * */
REG_FN(f_node)
{
    (val == 0)?
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Node_A_Task):
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Node_B_Task);
    return (CMD_FN_RET_OK);
}

/* @brief    helperTask will start
 * 0 - USB2SPI thread using chip A
 * 1 - USB2SPI thread using chip B
 * app.mode will be mUspi
 * */
REG_FN(f_uspi)
{
    (val == 0)?
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Usb2spi_A_Task):
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Usb2spi_B_Task);

    return(CMD_FN_RET_OK);
}

/* @brief    helperTask will start
 * 0 - TCWM thread using chip A
 * 1 - TCWM thread using chip B
 * app.mode will be mTcwm
 *
 * */
REG_FN(f_tcwm)
{
    (val == 0)?
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Tcwm_A_Task):
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Tcwm_B_Task);

    return(CMD_FN_RET_OK);
}

/* @brief    helperTask will start
 * 0 - TCFM thread using chip A
 * 1 - TCFM thread using chip B
 * app.mode will be mTcfm
 * */
REG_FN(f_tcfm)
{
    (val == 0)?
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Tcfm_A_Task):
    xEventGroupSetBits(app.xStartTaskEvent, Ev_Tcfm_B_Task);

    return(CMD_FN_RET_OK);
}

/* @brief    defaultTask will stop all working threads
 * app.mode will be mIDLE
 * */
REG_FN(f_stop)
{
    reset_FlushTask();

    xEventGroupSetBits(app.xStartTaskEvent, Ev_Stop_All);
    return (CMD_FN_RET_OK);
}

//-----------------------------------------------------------------------------
// Parameters change section : allowed only in app.mode = mIdle

REG_FN(f_addr)
{
    pbss->s.addr = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_panid)
{
    pbss->s.panID = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_numSlots)
{
    //it is only allowed to decrease the proportion of KNOWN_TAGS/NUM_SLOTS during a run-time.
    if((uint16_t)val>DEFAULT_NUM_SLOTS)
    {
      pbss->s.sfConfig.numSlots = (uint16_t)(val);
    }
    else
    {
      pbss->s.sfConfig.numSlots = DEFAULT_NUM_SLOTS;
    }
    return (CMD_FN_RET_OK);
}
REG_FN(f_slotPeriod)
{
    pbss->s.sfConfig.slotPeriod = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_sfPeriod)
{
    pbss->s.sfConfig.sfPeriod_ms = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_tag_replyDly_us)
{
    pbss->s.sfConfig.tag_replyDly_us = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_tag_pollTxFinalTx_us)
{
    pbss->s.sfConfig.tag_pollTxFinalTx_us = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_ant_tx_a)
{
    pbss->s.antTx_a = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_ant_rx_a)
{
    pbss->s.antRx_a = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_ant_tx_b)
{
    pbss->s.antTx_b = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_ant_rx_b)
{
    pbss->s.antRx_b = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_pdoa_offset)
{
    pbss->s.pdoaOffset_deg = (int16_t)val;
    return (CMD_FN_RET_OK);
}
REG_FN(f_rng_offset)
{
    pbss->s.rngOffset_mm = (int16_t)val;
    return (CMD_FN_RET_OK);
}
REG_FN(f_phase_corr_enable)
{
    pbss->s.phaseCorrEn = (uint8_t)val;
    return (CMD_FN_RET_OK);
}
REG_FN(f_pdoa_temp_coeff)
{
    pbss->s.pdoa_temp_coeff_mrad = (int16_t)val;
    return (CMD_FN_RET_OK);
}
REG_FN(f_acc)
{
    pbss->s.accEn = (uint8_t)val;
    return (CMD_FN_RET_OK);
}
REG_FN(f_dbg)
{
    pbss->s.debugEn = (uint8_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_diag)
{
    pbss->s.diagEn = (uint8_t)val;
    return (CMD_FN_RET_OK);
}
REG_FN(f_uart)
{
    pbss->s.uartEn = (uint8_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_auts)
{
    pbss->s.autoStartEn = (uint8_t)val;
    return (CMD_FN_RET_OK);
}
REG_FN(f_twr_report)
{
    pbss->s.reportLevel = (uint8_t)val;
    return (CMD_FN_RET_OK);
}
REG_FN(f_rc_delay)
{
    pbss->s.rcDelay_us = (uint16_t)val;
    return (CMD_FN_RET_OK);
}
REG_FN(f_smart_tx_en)
{
    pbss->s.smartTxEn = (uint8_t)val;
    return (CMD_FN_RET_OK);
}
REG_FN(f_rbc_en)
{
    pbss->s.rbcEn = (uint8_t)val;
    return (CMD_FN_RET_OK);
}
REG_FN(f_restore)
{
    restore_bssConfig();
    return (CMD_FN_RET_OK);
}
REG_FN(f_acc_threshold)
{
    pbss->s.acc_threshold = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_acc_stat_sense)
{
    pbss->s.acc_stationary_ms = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_acc_moving_sens)
{
    pbss->s.acc_moving_ms = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_emuEVB)
{
    pbss->s.emuEVB = (uint8_t)(val);
    return (CMD_FN_RET_OK);
}

//-----------------------------------------------------------------------------
//Service/debug commands
REG_FN(f_system_prf_mode)
{
    if(val == 16)
    {
        pbss->dwt_config.prf = DWT_PRF_16M;
        pbss->dwt_config.rxCode = 3;
        pbss->dwt_config.txCode = 3;
    }
    else if(val == 64)
    {
        pbss->dwt_config.prf = DWT_PRF_64M;
        pbss->dwt_config.rxCode = 9;
        pbss->dwt_config.txCode = 9;
    }
    return (CMD_FN_RET_OK);
}
REG_FN(f_tag_nSlow)
{
    pbss->s.sfConfig.tag_mSlow = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_tag_nFast)
{
    pbss->s.sfConfig.tag_mFast = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
REG_FN(f_tag_nMode)
{
    pbss->s.sfConfig.tag_Mode = (uint16_t)(val);
    return (CMD_FN_RET_OK);
}
/*
 * @brief add all discovered tags to knownTagList
 *        this is autoamatic function for manual adding:
 *        it assign low u16 from addr64 as a new addr16 for every tag
 *        uses tag_Mode, tag_mSlow, tag_mFast as default parameters
 */
REG_FN(f_add_all_to_list)
{
    const char  *ret = "All tags were added.";

    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {

        tag_addr_slot_t    *tag;
        uint64_t        *pAddr64;
        uint16_t        size = getDList_size();

        pAddr64 = getDList();

        while( (*pAddr64 != 0) && (size>0) )
        {
            tag = add_tag_to_knownTagList( *pAddr64, (uint16_t)(*pAddr64));

            if (!tag)
            {
                sprintf(str,"Cannot add: list is full.");
                sprintf(&str[strlen(str)],"\r\n");
                port_tx_msg((uint8_t*)str, strlen(str));

                ret = (NULL);    //cannot add new tag, the knownTagList is full.
                break;
            }
            else
            {
                tag->reqUpdatePending = 1;    //enable update of Tag's configuration on its next Poll
            }

            size--;
            pAddr64++;
        }
        CMD_FREE(str);
    }
    initDList();    //clear Discovered Tag List

    return (const char *)(ret);
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// JSON format for TagList section

/* JSON reporting:
 *
 * Root elements:
 * "NewTag": string (addr64)           | on discovering of new Tag
 * "DList" : array of strings (addr64) | reply to getDlist
 * "KList" : array of tag objects      | reply to getKlist
 * "TagAdded" : tag object             | reply to Add2List
 * "TagDeleted" : string (addr64)      | reply to delTag
 * "TWR" : twr object                  | on calculation of new range
 * "SN"  : service object(accel)       | on receiving data from IMU sensor
 *
 *
 * */

/* @brief Discovered List
 *
 * 'JSxxxx{"DList": ["addr64_1","addr64_2","addr64_3"]}'
 *
 * */
REG_FN(f_get_discovered_list)
{
    uint64_t    *pAddr64;
    uint16_t     size;
    int          jlen, tmp;

    pAddr64     = getDList();
    size        = getDList_size();

    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        CMD_ENTER_CRITICAL();

        /*  11+2 - minimum JSON length
         *  tmp=16+2 bytes per discovered tag: address64 plus quotes
         *  ',\r\n'=3 bytes per separation, if more than 1 elements
         *  we need to pre-calculate the length of json string for DList & KList : in order to keep malloc() small
         */
        jlen = (11+2);
        if(size>0)
        {
            tmp = 0;
            tmp = strlen("\"1122334455667788\"");    //16+2 for every addr64
            jlen += (tmp+3)*size -3;
        }

        sprintf(str,"JS%04X", jlen);                   // print pre-calculated length of JS object
        sprintf(&str[strlen(str)],"{\"DList\":[ ");    //+11 to json.
        port_tx_msg((uint8_t*)str, strlen(str));

        //DList cannot be with gaps
        // if changed, will need to change the calculation of jlen
        while(size>0 )
        {
            sprintf(str,"\"%08lX%08lX\"", (uint32_t)(*pAddr64>>32), (uint32_t)(*pAddr64));//+18 to json

            if(size > 1)
            {
                sprintf(&str[strlen(str)], ",\r\n");    //+3 to json
            }

            port_tx_msg((uint8_t*)str, strlen(str));

            pAddr64++;
            size--;
        }

        port_tx_msg((uint8_t*)"]}\r\n", 4);             //+2 to json

        CMD_EXIT_CRITICAL();

        initDList();                                    //clear the Discovered list

        CMD_FREE(str);
    }

    return (CMD_FN_RET_OK);
}

/* @brief This function shall return fixed length tag object staring
 *         66 characters : do not change : see f_get_known_list
 * */
static void fill_json_tag(char *str, tag_addr_slot_t *tag)
{
    sprintf(str,
            "{\"slot\":\"%04X\",\"a64\":\"%08lX%08lX\",\"a16\":\"%04X\","
            "\"F\":\"%04X\",\"S\":\"%04X\",\"M\":\"%04X\"}",
            tag->slot,(uint32_t)(tag->addr64>>32),(uint32_t)(tag->addr64),tag->addr16,
            tag->multFast, tag->multSlow, tag->mode );
}



/* @brief Known List
 *
 *  'JSxxxx{"KList":
 *  [
 *  {
 *   "slot":<string>,   //hex
 *   "a64":<string>,    //address64, string
 *   "a16":<string>,    //address16, string
 *   "F":<string>,      //multFast, hex
 *   "S":<string>,      //multSlow, hex
 *   "M":<string>       //mode, hex
 *  },
 *  {
 *   "slot":<string>,   //hex
 *   "a64":<string>,    //address64, string
 *   "a16":<string>,    //address16, string
 *   "F":<string>,      //multFast, hex
 *   "S":<string>,      //multSlow, hex
 *   "M":<string>       //mode, hex
 *  }
 *  ]}'
 *
 */
REG_FN(f_get_known_list)
{
    tag_addr_slot_t    *tag;
    int                jlen, size, tmp;

    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        CMD_ENTER_CRITICAL();

        /*  16 bytes overhead for JSON
         *  66 bytes per known tag
         *  3 bytes per separation
         *  we need to pre-calculate the length of json string for DList & KList : in order to keep malloc() small
         */
        tag = get_knownTagList();
        size = get_knownTagList_size();

        jlen = (10+2);

        if(size > 0)
        {
            tmp = 0;
            fill_json_tag(str, tag);
            tmp = strlen(str);                            //+NN to json for every known tag
            jlen += (tmp+3)*size -3;
        }

        sprintf(str,"JS%04X", jlen);                        // 6 print pre-calculated length of JS object
        sprintf(&str[strlen(str)],"{\"KList\":[");          // 10
        port_tx_msg((uint8_t*)str, strlen(str));

        //KList can be with gaps, so need to scan it whole
        for(int i = 0; i< MAX_KNOWN_TAG_LIST_SIZE; i++)
        {
            if(tag->slot != (uint16_t)(0))
            {
                fill_json_tag(str, tag);                    //NN

                if(size > 1)    //last element should not have ',\r\n'
                {
                    sprintf(&str[strlen(str)], ",\r\n");    //3
                }

                size--;

                port_tx_msg((uint8_t*)str, strlen(str));
            }

            tag++;
        }

        port_tx_msg((uint8_t*)"]}\r\n", 4);

        CMD_EXIT_CRITICAL();

        CMD_FREE(str);
    }

    return (CMD_FN_RET_OK);
}


/*
 * add tag from incoming command to knownTagList
 *
 * report:
 * 'JSxxxx{"TagAdded":
 *  {
 *   "slot":1,
 *   "a64":<string>,//address64, string
 *   "a16":<string>,//address16, string
 *   "F":<int>,        //multFast, int
 *   "S":<int>,        //multSlow, int
 *   "M":<int>        //mode, int
 *  }
 * }'
 *
 * Note: sscanf needs at least 212 bytes from stack
 *          and it uses malloc but not RTOS' malloc
 *
 * */
REG_FN(f_add_tag_to_list)
{
    const char *ret = NULL;

    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        tag_addr_slot_t    *tag;
        uint64_t           addr64=0;
        unsigned int       addr1=0, addr2=0;
        unsigned int       addr16=0, multFast=0, multSlow=0, mode=0, n=1, hlen;
        char               tmp[10];
        /* "addtag 11AABB4455FF7788 10AA 1 2 1" */
        n = sscanf(text, "%8s %08x%08x %x %x %x %x",
                          tmp, &addr1, &addr2, &addr16, &multFast, &multSlow, &mode);

        if (!(multFast == 0) && !(multSlow == 0) && (n == 7))
        {
            addr64 = (uint64_t)((((uint64_t)addr1)<<32) | addr2);

            tag = add_tag_to_knownTagList(addr64, (uint16_t)addr16);

            if(tag)
            {
                tag->multFast = (uint16_t)multFast;
                tag->multSlow = (uint16_t)multSlow;
                tag->mode = (uint16_t)mode;
                tag->reqUpdatePending = 1;                //update Tag's configuration on its next Poll

                hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object
                sprintf(&str[strlen(str)],"{\"TagAdded\": ");

                fill_json_tag(&str[strlen(str)], tag);

                sprintf(&str[strlen(str)],"}"); //\r\n

                sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will kill first '{'
                str[hlen]='{';                            //restore the start bracket

                sprintf(&str[strlen(str)],"\r\n");
                port_tx_msg((uint8_t*)str, strlen(str));

                ret = CMD_FN_RET_OK;
            }
        }

        CMD_FREE(str);
    }

    return (ret);
}


/*
 * @brief delete the tag addr64 from knownTagList
 *          the function will always report the tag was deleted.
 * report:
 * 'JSxxxx{"TagDeleted":
 *             <string> //address64, string
 *        }'
 *
 * */
REG_FN(f_del_tag_from_list)
{
    const char *ret = NULL;

    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        uint64_t        addr64=0;
        unsigned int    addr1=0, addr2=0, hlen;
        char            tmp[10];

        /* "delTag 11AABB4455FF7788" */
        sscanf(text, "%8s %08x%08x", tmp, &addr1, &addr2);

        addr64 = (uint64_t)((((uint64_t)addr1)<<32) | (uint64_t)addr2);
        if(addr64 > 0xFFFF)
        {
            del_tag64_from_knownTagList(addr64);
        }
        else
        {
            del_tag16_from_knownTagList((uint16_t)addr64);
        }

        hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object
        sprintf(&str[strlen(str)],"{\"TagDeleted\": \"%08x%08x\"}", addr1 , addr2);

        sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will erase first '{'
        str[hlen]='{';                            //restore the start bracket
        sprintf(&str[strlen(str)],"\r\n");
        port_tx_msg((uint8_t*)str, strlen(str));

        ret = CMD_FN_RET_OK;

        CMD_FREE(str);
    }

    return (const char *)(ret);
}


//-----------------------------------------------------------------------------
// Communication section


/* @brief
 * */
REG_FN(f_decaPDOA)
{
    const char *ret = NULL;
    const char ver[] = FULL_VERSION;

    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        int  hlen;

        hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object

        sprintf(&str[strlen(str)],"{\"Info\":{\r\n");
        sprintf(&str[strlen(str)],"\"Device\":\"PDOA Node\",\r\n");
        sprintf(&str[strlen(str)],"\"Version\":\"%s\",\r\n", ver);
        sprintf(&str[strlen(str)],"\"Build\":\"%s %s\",\r\n", __DATE__, __TIME__ );
        sprintf(&str[strlen(str)],"\"Driver\":\"%s\"}}", DW1000_DEVICE_DRIVER_VER_STRING );

        sprintf(&str[2],"%04X",strlen(str)-hlen);   //add formatted 4X of length, this will erase first '{'
        str[hlen]='{';                            //restore the start bracket
        sprintf(&str[strlen(str)],"\r\n");
        port_tx_msg((uint8_t*)str, strlen(str));

        CMD_FREE(str);
        ret = CMD_FN_RET_OK;
    }

    return (ret);
}

//-----------------------------------------------------------------------------

/*
 * @brief   show current mode of operation,
 *          version, and the configuration in JSON format
 *          Should be executed from STOP as
 *
 * */
REG_FN(f_jstat)
{
    const char *ret = NULL;

    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        CMD_ENTER_CRITICAL();

        int  hlen;

        /* System Config object */
        hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object
        sprintf(&str[strlen(str)],"{\"System Config\":{\r\n");
        sprintf(&str[strlen(str)],"\"ADDR\":\"%04X\",\r\n", pbss->s.addr);
        sprintf(&str[strlen(str)],"\"PANID\":\"%04X\",\r\n",pbss->s.panID);
        sprintf(&str[strlen(str)],"\"NUMSLOT\":%d,\r\n",pbss->s.sfConfig.numSlots);
        sprintf(&str[strlen(str)],"\"SLOTPER\":%d,\r\n",pbss->s.sfConfig.slotPeriod);
        sprintf(&str[strlen(str)],"\"SFPER\":%d,\r\n",pbss->s.sfConfig.sfPeriod_ms);
        sprintf(&str[strlen(str)],"\"REPDEL\":%d,\r\n",pbss->s.sfConfig.tag_replyDly_us);
        sprintf(&str[strlen(str)],"\"P2FDEL\":%d,\r\n",pbss->s.sfConfig.tag_pollTxFinalTx_us);
        sprintf(&str[strlen(str)],"\"RCDEL\":%d,\r\n",pbss->s.rcDelay_us);
        sprintf(&str[strlen(str)],"\"SMTX\":%d}}",pbss->s.smartTxEn);

        sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will erase first '{'
        str[hlen]='{';                            //restore the start bracket
        sprintf(&str[strlen(str)],"\r\n");
        port_tx_msg((uint8_t*)str, strlen(str));

        /* Run Time object */
        hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object
        sprintf(&str[strlen(str)],"{\"Run Time\":{\r\n");
        sprintf(&str[strlen(str)],"\"UART\":%d,\r\n",pbss->s.uartEn);
        sprintf(&str[strlen(str)],"\"AUTO\":%d,\r\n",pbss->s.autoStartEn);
        sprintf(&str[strlen(str)],"\"PCREP\":%d}}",pbss->s.reportLevel);

        sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will erase first '{'
        str[hlen]='{';                            //restore the start bracket
        sprintf(&str[strlen(str)],"\r\n");
        port_tx_msg((uint8_t*)str, strlen(str));

        /* Calibration object */
        hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object
        sprintf(&str[strlen(str)],"{\"Calibration\":{\r\n");
        sprintf(&str[strlen(str)],"\"ANTTXA\":%d,\r\n",pbss->s.antTx_a);
        sprintf(&str[strlen(str)],"\"ANTRXA\":%d,\r\n",pbss->s.antRx_a);
        sprintf(&str[strlen(str)],"\"ANTTXB\":%d,\r\n",pbss->s.antTx_b);
        sprintf(&str[strlen(str)],"\"ANTRXB\":%d,\r\n",pbss->s.antRx_b);
        sprintf(&str[strlen(str)],"\"PDOAOFF\":%d,\r\n",(int)pbss->s.pdoaOffset_deg);
        sprintf(&str[strlen(str)],"\"RNGOFF\":%d,\r\n",(int)pbss->s.rngOffset_mm);
        sprintf(&str[strlen(str)],"\"ACCTHR\":%d,\r\n",pbss->s.acc_threshold);
        sprintf(&str[strlen(str)],"\"ACCSTAT\":%d,\r\n",pbss->s.acc_stationary_ms);
        sprintf(&str[strlen(str)],"\"ACCMOVE\":%d}}",pbss->s.acc_moving_ms);

        sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will erase first '{'
        str[hlen]='{';                            //restore the start bracket
        sprintf(&str[strlen(str)],"\r\n");
        port_tx_msg((uint8_t*)str, strlen(str));

        CMD_FREE(str);

        CMD_EXIT_CRITICAL();

        ret = CMD_FN_RET_OK;
    }
    return (ret);
}


/*
 * @brief   show current UWB parameters in JSON format
 *
 * */
REG_FN(f_uwb)
{
    const char *ret = NULL;

    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        int  hlen;

        hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object
        sprintf(&str[strlen(str)],"{\"UWB PARAM\":{\r\n");

        sprintf(&str[strlen(str)],"\"CHAN\":%d,\r\n",deca_to_chan(pbss->dwt_config.chan));
        sprintf(&str[strlen(str)],"\"PRF\":%d,\r\n", deca_to_prf (pbss->dwt_config.prf));
        sprintf(&str[strlen(str)],"\"PLEN\":%d,\r\n",deca_to_plen(pbss->dwt_config.txPreambLength));
        sprintf(&str[strlen(str)],"\"PAC\":%d,\r\n", deca_to_pac (pbss->dwt_config.rxPAC));
        sprintf(&str[strlen(str)],"\"TXCODE\":%d,\r\n",pbss->dwt_config.txCode);
        sprintf(&str[strlen(str)],"\"DATARATE\":%d}}",deca_to_bitrate(pbss->dwt_config.dataRate));

        sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will erase first '{'
        str[hlen]='{';                            //restore the start bracket
        sprintf(&str[strlen(str)],"\r\n");
        port_tx_msg((uint8_t*)str, strlen(str));

        CMD_FREE(str);
        ret = CMD_FN_RET_OK;
    }
    return (ret);
}

/*
 * @brief show current mode of operation,
 *           version, and the configuration
 *
 * */
REG_FN(f_stat)
{
    const char * ret = CMD_FN_RET_OK;

    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        sprintf(str,"MODE: %s\r\n"
                    "LAST ERR CODE: %d\r\n"
                    "MAX MSG LEN: %d\r\n",
          (app.mode==mIDLE)?("STOP"):
          (app.mode==mTWR)?( "NODE"):
          (app.mode==mTCWM)?("TCWM"):
          (app.mode==mTCFM)?("TCFM"):
          (app.mode==mUSB2SPI)?("USB2SPI"):
          ("unknown"),
          app.lastErrorCode,
          app.maxMsgLen);

        port_tx_msg((uint8_t*)str, strlen(str));

        CMD_FREE(str);
        app.lastErrorCode = 0;
        app.maxMsgLen = 0;

        f_decaPDOA(NULL, pbss, 0);
        f_jstat(NULL, pbss, 0);
        f_get_discovered_list(NULL,NULL,0);
        f_get_known_list(NULL,NULL,0);
    }

    ret = CMD_FN_RET_OK;
    return (ret);
}


/*
 * @brief Show current temperature reading on both chips
 *        should be called only from STOP condition.
 *
 * */
REG_FN(f_temp)
{
    const char * ret = NULL;

    char *str = CMD_MALLOC(MAX_STR_SIZE);
    float  temp;

    if(str)
    {
        CMD_ENTER_CRITICAL();

        set_dw_spi_fast_rate(DW_MASTER);
        temp = (uint8_t)(dwt_readtempvbat(1) >> 8);
        temp = 23.0f + (float)(temp - (uint8_t)dwt_gettmeas())*1.14f;

        sprintf(str,"master chip: %i \r\n", (int)temp);
        port_tx_msg((uint8_t*)str, strlen(str));


        set_dw_spi_fast_rate(DW_SLAVE);
        temp = (uint8_t)(dwt_readtempvbat(1) >> 8);
        temp = 23.0f + (float)(temp - (uint8_t)dwt_gettmeas())*1.14f;

        sprintf(str,"slave chip: %d \r\n", (int)temp);
        port_tx_msg((uint8_t*)str, strlen(str));

        set_dw_spi_fast_rate(DW_MASTER);

        CMD_FREE(str);
        CMD_EXIT_CRITICAL();
    }

    ret = CMD_FN_RET_OK;
    return (ret);
}

/*
 * @brief Show all available commands
 *
 * */
REG_FN(f_help_app)
{
    int        indx = 0;
    const char * ret = NULL;
    char *str = CMD_MALLOC(MAX_STR_SIZE);
	
    if(str)
    {memset(str,0,MAX_STR_SIZE);
        CMD_ENTER_CRITICAL();

        while (known_commands[indx].name != NULL)
        {
            sprintf(str,"%s \r\n", known_commands[indx].name);

            port_tx_msg((uint8_t*)str, strlen(str));

            indx++;
        }

        CMD_EXIT_CRITICAL();

        CMD_FREE(str);
        ret = CMD_FN_RET_OK;
    }

    return (ret);
}

//-----------------------------------------------------------------------------
// Communication change section

/*
 * @brief save configuration
 *
 * */
REG_FN(f_save)
{
    error_e    err_code;

    err_code = save_bssConfig(pbss);    // TBD

    if(err_code != _NO_ERR)
    {
        error_handler(0, err_code);    //not a fatal error
        return (NULL);
    }

    return (CMD_FN_RET_OK);
}

REG_FN(f_tmode)
{
	 const char * ret = CMD_FN_RET_OK;
	char buf[10];
	memset(buf,0,sizeof(buf));

	/* "addtag 11AABB4455FF7788 10AA 1 2 1" */
	   sscanf(text, "TMODE %2s",buf);
	//printf("text:%s buf =%s \r\n",text,buf);
	if(strcmp("JS",buf)==0)
	{
		app.pConfig->s.reportLevel=1;
	}
	else if(strcmp("HE",buf)==0)
	{
		app.pConfig->s.reportLevel=0;
	}
	else
	{
	
	}
	//printf("app.pConfig->s.reportLevel =%d\r\n",app.pConfig->s.reportLevel);

	return (CMD_FN_RET_OK);
}

/*
 * brief : base battery's power
 * date  : 2018-12-19
 *         HHHHHH
 */

uint8_t BATTERY_SPEED;

REG_FN(f_base_battery)
{
	char buf[2] = {0};
	sscanf(text, "BATTERY %2s", buf);
	BATTERY_SPEED = atoi(buf);				//TODO: Processing battery
	return (CMD_FN_RET_OK);
}

/*
 * brief : base's speed
 * date  : 2018-12-19
 *         HHHHHH
 */
REG_FN(f_base_speed)
{
	char buf[10] = {0};
	sscanf(text, "SPEED %4s", buf);
	printf("speed = %s\r\n", buf);
	float speed_temp = atof(buf);			//TODO: Processing speed
	printf("%f\r\n", speed_temp);
	BATTERY_SPEED = speed_temp * 10 + 100;
	return (CMD_FN_RET_OK);
}

/*
 * brief : Reaction to event
 * date  : 2018-12-19
 *         HHHHHH
 */
REG_FN(f_tag_alarm)
{
	char buf[3] = {0};
	sscanf(text, "ALARM %3s", buf);
	
	if (buf[0] == '0')
	{
		if (buf[1] == '0')
		{
			if (buf[2] == '0')
			{
				BATTERY_SPEED = 0xC9;
			}
			else
			{
				BATTERY_SPEED = 0xCA;
			}
		}
		else	//buf[1] == '1'
		{
			if (buf[2] == '0')
			{
				BATTERY_SPEED = 0xCB;
			}
			else
			{
				BATTERY_SPEED = 0xCC;
			}
		}
	}
	else	//buf[0] == '1'
	{
		if (buf[1] == '0')
		{
			if (buf[2] == '0')
			{
				BATTERY_SPEED = 0xCD;
			}
			else
			{
				BATTERY_SPEED = 0xCE;
			}
		}
		else	//buf[1] == '1'
		{
			if (buf[2] == '0')
			{
				BATTERY_SPEED = 0xCF;
			}
			else
			{
				BATTERY_SPEED = 0xD0;
			}
		}
	}
	return (CMD_FN_RET_OK);
}

REG_FN(f_base_reboot)
{
 	NRF_WDT->CRV=32768;
	NRF_WDT->CONFIG |= 1<<0 | 1<<3;
	NRF_WDT->RREN |= 1<<0;
	NRF_WDT->RR[0]=0x6E524635;
	NRF_WDT->TASKS_START = 1U;
	return (CMD_FN_RET_OK);
}

uint8_t dist_or_PDOA = 0xf;

REG_FN(f_change_node_addr)
{
        unsigned char buf[4] = {0};
	sscanf(text, "USE %4s", buf);

//        int tmp_addr = (buf[0]-'0')*10 + (buf[1]-'0');

//        printf("tmp_addr = %d\r\n", tmp_addr);

//        app.pConfig->s.addr = tmp_addr;

//        printf("buf[0] = %c\r\n", buf[0]);
//        printf("buf[1] = %c\r\n", buf[1]);

	if(strcmp("PDOA",buf)==0)
	{
		app.pConfig->s.addr = 0x0001;
		dist_or_PDOA = 0xff;
	}
	else if(strcmp("DIST",buf)==0)
	{
		app.pConfig->s.addr = 0xffff;
		dist_or_PDOA = 0x00;
	}
	else
	{
		return (CMD_FN_RET_FAIL);
	}

        printf("Node addr = 0x%04x\r\n", app.pConfig->s.addr);

	return (CMD_FN_RET_OK);
}



//-----------------------------------------------------------------------------

/* end f_xx command functions */

//-----------------------------------------------------------------------------
/* list of known commands:
 * NAME, allowed_MODE,     REG_FN(fn_name)
 * */
const command_t known_commands []= {
    /* CMDNAME   MODE   fn     */
    /* 1. commands to set system config, run-time and calibration variables */
    {"ADDR",    mIDLE,  f_addr},
    {"PANID",   mIDLE,  f_panid},
    {"NUMSLOT", mIDLE,  f_numSlots},
    {"SLOTPER", mIDLE,  f_slotPeriod},
    {"SFPER",   mIDLE,  f_sfPeriod},
    {"REPDEL",  mIDLE,  f_tag_replyDly_us},
    {"P2FDEL",  mIDLE,  f_tag_pollTxFinalTx_us},
    {"RCDEL",   mIDLE,  f_rc_delay},

    {"UART",    mIDLE,  f_uart},
    {"AUTO",    mIDLE,  f_auts},

    {"ANTTXA",  mIDLE,  f_ant_tx_a},
    {"ANTRXA",  mIDLE,  f_ant_rx_a},
    {"ANTTXB",  mIDLE,  f_ant_tx_b},
    {"ANTRXB",  mIDLE,  f_ant_rx_b},
    {"ACCTHR",  mIDLE,  f_acc_threshold},
    {"ACCSTAT", mIDLE,  f_acc_stat_sense},
    {"ACCMOVE", mIDLE,  f_acc_moving_sens},

    /* 2. anytime commands */
    {"STOP",    mANY,   f_stop},
    {"STAT",    mANY,   f_stat},
    {"HELP",    mANY,   f_help_app},
    {"?",       mANY,   f_help_app},
    {"SAVE",    mANY,   f_save},
	//{"DUMP",    mANY,   f_dump},
	{"TMODE",    mANY,   f_tmode},	
    /* 3. app start commands */
    {"NODE",    mIDLE,  f_node},
    {"USPI",    mIDLE,  f_uspi},
    {"TCWM",    mIDLE,  f_tcwm},
    {"TCFM",    mIDLE,  f_tcfm},

    /* 4. node application commands */
    {"DECA$",   mANY,   f_decaPDOA},
    {"JSTAT",   mANY,   f_jstat},
    {"GETDLIST",mANY,   f_get_discovered_list},
    {"GETKLIST",mANY,   f_get_known_list},
    {"ADDTAG",  mANY,   f_add_tag_to_list},
    {"DELTAG",  mANY,   f_del_tag_from_list},
    {"PDOAOFF", mANY,   f_pdoa_offset},
    {"RNGOFF",  mANY,   f_rng_offset},
    {"PDOATEMP", mANY,  f_pdoa_temp_coeff},
    {"PHCORREN", mANY,  f_phase_corr_enable},

    /* 5. service commands */
    {"RESTORE", mIDLE,  f_restore},
    {"PCREP",   mIDLE,  f_twr_report},
    {"RBC",     mIDLE,  f_rbc_en},
    {"SMTX",    mIDLE,  f_smart_tx_en},
    {"D2K",     mTWR,   f_add_all_to_list},
    {"MFAST",   mIDLE,  f_tag_nFast},
    {"MSLOW",   mIDLE,  f_tag_nSlow},
    {"MMODE",   mIDLE,  f_tag_nMode},
    {"SPRFMODE",mIDLE,  f_system_prf_mode},
    {"DEBUGLED",mIDLE,  f_dbg},
    {"DIAG",    mIDLE,  f_diag},
    {"ACCUM",   mIDLE,  f_acc},
    {"EMUEVB",  mIDLE,  f_emuEVB},
    {"UWBCFG",  mIDLE,  f_uwb},
    {"TEMP",    mIDLE,  f_temp},

    {"BATTERY", mANY, 	f_base_battery},		//HHHHHH
    {"SPEED", 	mTWR,	f_base_speed},			//HHHHHH
    {"ALARM",	mANY, 	f_tag_alarm},           //HHHHHH
    {"REBOOT",	mIDLE, 	f_base_reboot},           //HHHHHH

    {"USE",     mIDLE, 	f_change_node_addr},

    {NULL,      NULL,   NULL}
};
