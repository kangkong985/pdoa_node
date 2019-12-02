/*
 * @file    cmd.c
 * @brief     command string as specified in document SWxxxx version X.x.x
 * @param    *text - string
 *             source - is an input stream source: this will be used to reply to the specified direction
 *
 * CODEWORD:
 *      "XXXX YYY" : set appropriate parameter XXXX to value YYY, allowed as per permission.
 *      "ZZZZ"     : change a mode of operation to ZZZZ, allowed only from IDLE except of "STOP".
 *      "STOP"     : allowed at any time and put application to the IDLE
 *
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) DecaWave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */
#include <cmd.h>
#include <cmd_fn.h>
#include <config.h>
#include <usb_uart_tx.h>
/*
 *    Command interface
 */

/* IMPLEMENTATION */

/*
 * @brief "error" will be sent if error during parser or command execution returned error
 * */
static void cmd_onERROR(const char *err, control_t *pcmd)
{
    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        strcpy(str, "error \r\n");
        if ( strlen(err)< (MAX_STR_SIZE-6-3-1)) {
            strcpy(&str[6], err);
            strcpy(&str[6 + strlen(err)], "\r\n");
        }
        port_tx_msg((uint8_t*)str, strlen(str));

        CMD_FREE(str);
    }
}


/* @fn         command_parser
 * @brief    checks if input "text" string in known "COMMAND" or "PARAMETER VALUE" format,
 *             checks their execution permissions, a VALUE range if restrictions and
 *             executes COMMAND or sets the PARAMETER to the VALUE
 * */
void command_parser(char *text)
{
    control_t   mcmd_console;
    control_t   *pcmd = &mcmd_console;
    command_t   *pk = NULL;

    memset (pcmd, 0 , sizeof(control_t));

    do{
        text[pcmd->indx]=(char)toupper((int)text[pcmd->indx]);
    }while(text[ ++pcmd->indx ]);

    /* Assume text may have more than one command inside.
     * For example "getKLIST\nnode 0\n" : this will execute 2 commands.
     * */
    pcmd->token = strtok(text, "\n");    //get first token

    do {
        sscanf(pcmd->token,"%9s %d", pcmd->cmd, &pcmd->val); //check MAX_COMMAND_SIZE if format will be changed

        pcmd->indx = 0;
        pcmd->equal = _NO_COMMAND;

        while (known_commands[pcmd->indx].name != NULL)
        {
            pk = (command_t *) &known_commands[pcmd->indx];

        if (( strcmp(pcmd->cmd, pk->name) == 0 ) &&\
            ( strlen(pcmd->cmd) == strlen(pk->name)) )
        {
            pcmd->equal = _COMMAND_FOUND;

            /* check command execution permissions.
             * some commands can be executed only from Idle system mode:
             * i.e. when no active processes are running.
             * other commands can be executed at any time.
             * */
            if (pk->mode == app.mode || pk->mode == mANY)
            {
                pcmd->equal = _COMMAND_ALLOWED;
                break;
            }
        }

            pcmd->indx++;
        }

        switch (pcmd->equal)
        {
            case (_COMMAND_FOUND) :
            {	
//            	printf("%s,%d mode=%d mANY mIDLE mTWR mUSB2SPI mUART2SPI mTCWM mTCFM \r\n",__FUNCTION__,__LINE__,app.mode);
                cmd_onERROR(" incompatible mode", pcmd);
                break;
            }
            case (_COMMAND_ALLOWED):
            {
                /* execute corresponded fn() */
                param_block_t *pbss = get_pbssConfig();
                pcmd->ret = pk->fn(pcmd->token, pbss, pcmd->val);

                if (pcmd->ret)
                {
                    port_tx_msg((uint8_t*)pcmd->ret, strlen(pcmd->ret));
                }
                else
                {
                    cmd_onERROR(" function", pcmd);
                }
                break;
            }
            default:
                break;
        }

        pcmd->token = strtok(NULL, "\n");

    }while(strlen(pcmd->token));
}


/* end of cmd.c */
