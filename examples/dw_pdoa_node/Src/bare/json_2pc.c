/*
 * @file     json_2pc.c
 * @brief    collection of JSON formatted functions which used
 *           to report from Node application to the PC
 *
 * @author Decawave Software
 *
 * @attention Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *            All rights reserved.
 *
 */
#include "node.h"
#include "cmd_fn.h"
#include "task_imu.h"
#include "math.h"
/*
typedef struct
{
	uint16_t head;	   
	uint8_t  len;   
	uint8_t cmd;	
	uint16_t x_cm;	
	uint16_t y_cm;
	uint16_t d_cm;
	uint16_t degre;
	uint16_t crc16;
}pdoa_hex_pack_t;
*/

#define CRC_16_MODBUS 0xA001	   /*!< CRC mode: CRC-16-MODBUS poly:0x8005 INIT:0XFFFF*/
#define CRC_16_MODE (CRC_16_MODBUS)
#define TABLE_SIZE 16

//#define RADICAL_SIGN_7 	(2.645751311f)
//#define RADICAL_SIGN_15 (3.872983346f)

#define SIN_15	(0.258819f)
#define SIN_30	(0.5f)
#define SIN_45	(0.707107f)
#define SIN_60 	(0.866025f)
#define SIN_75	(0.965926f)

static uint32_t crc_table[TABLE_SIZE];
static bool init_done;


extern uint16_t dis_1st, dis_2nd;

extern uint16_t dis_1st;
extern uint16_t dis_2nd;

void soft_crc16_init(void)
{
	uint16_t crc = 0;
	uint8_t i;
	uint8_t j;

	if (init_done)
		return;

	for (j = 0; j < TABLE_SIZE; j++) {
		crc = 0;
		for (i = 0x01; i != 0x10; i <<= 1) {
			if ((crc & 0x0001) != 0) {
				crc >>= 1;
				crc ^= CRC_16_MODE;
			} else {
				crc >>= 1;
			}
			if ((j & i) != 0) {
				crc ^= CRC_16_MODE;
			}
		}
		crc_table[j] = crc;
	}
	init_done = true;
}

uint32_t soft_crc16(const void *__buf, uint16_t len, uint32_t crc_init)
{
	const uint8_t *buf = __buf;
	uint8_t crc_H8;
	uint16_t crc = (uint16_t) crc_init;

	//ASSERT(init_done == 1);

	while (len--) {
		crc_H8 = (uint8_t)(crc & 0x000F);
		crc >>= 4;
		crc ^= crc_table[crc_H8 ^ (*buf & 0x0F)];
		crc_H8 = (uint8_t)(crc & 0x000F);
		crc >>= 4;
		crc ^= crc_table[crc_H8 ^ (*buf >> 4)];
		buf++;
	}

	return (uint32_t)crc;
}

extern double	signalPower_master[2], signalPower_slave[2];

uint8_t Crc8(uint8_t data)
{
	uint8_t crc;
	//int i;
	int Com_Code = 0;
	/*for(i = 0; i < 8; i++)
	{
		Com_Code |= (~ (data >> i) & 0x1) << i;
	}*/
	if(data == 0)
	{
		return '@';	
	}
	else
	{
		Com_Code = ~data;
		Com_Code += 1;
		crc = (uint8_t) Com_Code & 0xFF;
	}

	return crc;
}

#define SORT_SIZE 10
//wheel_control_t wheel_control;
static int sort_X[SORT_SIZE] = {0}, sort_Y[SORT_SIZE] = {0}, sort_D[SORT_SIZE] = {0};

void swap(int *a, int *b)
{
    int temp;
    temp = *a;
    *a = *b;
    *b = temp;
}


void quickSort(int arr[] ,int start, int end)
{
    int arrBase, arrMiddle;

    int tempStart = start,
        tempEnd = end;

    //对于这种递归的函数，内部必须要有一个函数返回的条件
    if(tempStart >= tempEnd)
        return;

    //拷贝一个基准值作为后面比较的参数
    arrBase = arr[start];
    while(start < end)
    {
        while(start < end && arr[end] > arrBase)
            end--;
        if(start < end)
        {
            swap(&arr[start], &arr[end]);
            start++;
        }

        while(start < end && arr[start] < arrBase)
            start++;
        if(start < end)
        {
            swap(&arr[start], &arr[end]);
            end--;
        }
    }
    arr[start] = arrBase;
    arrMiddle = start;

    //分治方法进行递归
    quickSort(arr,tempStart,arrMiddle-1);
    quickSort(arr,arrMiddle+1,tempEnd);
}

void send_to_wheelchair(result_t *pRes)
{

	wheelchair_t wheelchair = { TURNING_ZERO, 
								'!', 'X', 0, 0, 
								'!', 'x', 0, 0,
								'!', 'Y', 0, 0,
								'!', 'y', 0, 0, 
								'#' };

	uint16_t real_x = 2048;
	uint16_t real_y = 2048;

	static int sort_counter = 0;
	double current_x, current_y, current_d, current_a;

	int sign_bit = 0;

	if (++sort_counter == SORT_SIZE)
		sort_counter = 0;

	sort_X[sort_counter] = (int)pRes->x_cm;
	sort_Y[sort_counter] = (int)pRes->y_cm;
	sort_D[sort_counter] = (int)pRes->dist_cm;

	quickSort(sort_X, 0, SORT_SIZE-1);
	quickSort(sort_Y, 0, SORT_SIZE-1);
	quickSort(sort_D, 0, SORT_SIZE-1);

	current_x = sort_X[SORT_SIZE/2];
	current_y = sort_Y[SORT_SIZE/2];
	current_d = sort_D[SORT_SIZE/2];

	current_a = atan2f(current_x, current_y)*(180/M_PI);

	if (current_a > 0)
		sign_bit = 1;
	else
		sign_bit = (-1);

	printf("1st= %d, 2nd= %d\r\n", dis_1st, dis_2nd);

	if (dis_1st >= 85 && dis_2nd >= 85)
	{
		if (current_d > 300 /* && dis_1st>150 && dis_2nd>150 */)
		{
	//		if (current_a+15 > 0 && current_a < 15)
	//			wheelchair.level = TURNING_ONE;
	//		else if ((current_a > 15 && current_a < 30) || (current_a+15 < 0 && current_a+30 > 0))
	//			wheelchair.level = TURNING_TWO;
	//		else if ((current_a > 30 && current_a < 45) || (current_a+30 < 0 && current_a+45 > 0))
	//			wheelchair.level = TURNING_TWO;
	//		else if ((current_a > 45 && current_a < 60) || (current_a+45 < 0 && current_a+60 > 0))
	//			wheelchair.level = TURNING_THREE;
	//		else if ((current_a > 60 && current_a < 75) || (current_a+60 < 0 && current_a+75 > 0))
	//			wheelchair.level = TURNING_THREE;
	//		else if ((current_a > 75 && current_a < 90) || (current_a+75 < 0 && current_a+90 > 0))
	//			wheelchair.level = TURNING_FOUR;

			if (1 == sign_bit)
			{
				if (current_a < 15)
					wheelchair.level = TURNING_ONE;
				else if (current_a > 15 && current_a < 30)
					wheelchair.level = TURNING_TWO;
				else if (current_a > 30 && current_a < 45)
					wheelchair.level = TURNING_THREE;
				else if (current_a > 45 && current_a < 60)
					wheelchair.level = TURNING_FOUR;
				else if (current_a > 60 && current_a < 75)
					wheelchair.level = TURNING_FOUR;
				else if (current_a > 75 && current_a < 90)
					wheelchair.level = TURNING_FOUR;
			}
			else
			{
				if (current_a+15 > 0)
					wheelchair.level = TURNING_ONE;
				else if (current_a+15 < 0 && current_a+30 > 0)
					wheelchair.level = TURNING_TWO;
				else if (current_a+30 < 0 && current_a+45 > 0)
					wheelchair.level = TURNING_THREE;
				else if (current_a+45 < 0 && current_a+60 > 0)
					wheelchair.level = TURNING_FOUR;
				else if (current_a+60 < 0 && current_a+75 > 0)
					wheelchair.level = TURNING_FOUR;
				else if (current_a+75 < 0 && current_a+90 > 0)
					wheelchair.level = TURNING_FOUR;
			}

			switch (wheelchair.level)
			{
			case TURNING_ZERO:
				real_x = (uint16_t)2048;
				real_y = (uint16_t)4096;
				break;
			case TURNING_ONE:
				real_x = (uint16_t)(2048-2048*SIN_15*sign_bit);
				real_y = (uint16_t)(2048+2048*SIN_75);
				break;
			case TURNING_TWO:
				real_x = (uint16_t)(2048-2048*SIN_30*sign_bit);
				real_y = (uint16_t)(2048+2048*SIN_60);
				break;
			case TURNING_THREE:
				real_x = (uint16_t)(2048-2048*SIN_45*sign_bit);
				real_y = (uint16_t)(2048+2048*SIN_45);
				break;
			case TURNING_FOUR:
				real_x = (uint16_t)(2048-1536*SIN_60*sign_bit);
	//			real_x = (uint16_t)real_x*3/4;
				real_y = (uint16_t)(2048+1536*SIN_30);
	//			real_y = (uint16_t)real_y*3/4;
				break;
			case TURNING_FIVE:
				real_x = (uint16_t)(2048-2048*SIN_75*sign_bit);
				real_y = (uint16_t)(2048+2048*SIN_15);
				break;
			default:
				return;
			}  
		}
		else if (current_d<300 && current_d>200)
		{
	//		real_x = (uint16_t)(2048-2048*current_x/current_d*sign_bit);
	//		real_y = (uint16_t)(2048+2048*current_y/current_d);

			if (1 == sign_bit)
			{
				if (current_a < 15)
					wheelchair.level = TURNING_ONE;
				else if (current_a > 15 && current_a < 30)
					wheelchair.level = TURNING_TWO;
				else if (current_a > 30 && current_a < 45)
					wheelchair.level = TURNING_THREE;
				else if (current_a > 45 && current_a < 60)
					wheelchair.level = TURNING_FOUR;
				else if (current_a > 60 && current_a < 75)
					wheelchair.level = TURNING_FOUR;
				else if (current_a > 75 && current_a < 90)
					wheelchair.level = TURNING_FOUR;
			}
			else
			{
				if (current_a+15 > 0)
					wheelchair.level = TURNING_ONE;
				else if (current_a+15 < 0 && current_a+30 > 0)
					wheelchair.level = TURNING_TWO;
				else if (current_a+30 < 0 && current_a+45 > 0)
					wheelchair.level = TURNING_THREE;
				else if (current_a+45 < 0 && current_a+60 > 0)
					wheelchair.level = TURNING_FOUR;
				else if (current_a+60 < 0 && current_a+75 > 0)
					wheelchair.level = TURNING_FOUR;
				else if (current_a+75 < 0 && current_a+90 > 0)
					wheelchair.level = TURNING_FOUR;
			}

			switch (wheelchair.level)
			{
			case TURNING_ZERO:
				real_x = (uint16_t)2048;
				real_y = (uint16_t)3072;
				break;
			case TURNING_ONE:
				real_x = (uint16_t)(2048-1536*SIN_15*sign_bit);
				real_y = (uint16_t)(2048+1536*SIN_75);
				break;
			case TURNING_TWO:
				real_x = (uint16_t)(2048-1536*SIN_30*sign_bit);
				real_y = (uint16_t)(2048+1536*SIN_60);
				break;
			case TURNING_THREE:
				real_x = (uint16_t)(2048-1536*SIN_45*sign_bit);
				real_y = (uint16_t)(2048+1536*SIN_45);
				break;
			case TURNING_FOUR:
				real_x = (uint16_t)(2048-1536*SIN_60*sign_bit);
	//			real_x = (uint16_t)real_x*3/4;
				real_y = (uint16_t)(2048+1536*SIN_30);
	//			real_y = (uint16_t)real_y*3/4;
				break;
			case TURNING_FIVE:
				real_x = (uint16_t)(2048-1536*SIN_75*sign_bit);
				real_y = (uint16_t)(2048+1536*SIN_15);
				break;
			default:
				return;
			}

		}
		else if (current_d<200 && current_d>150)
		{
	//		real_x = (uint16_t)(2048-2048*current_x/current_d*sign_bit);
	//		real_y = (uint16_t)(2048+2048*current_y/current_d);

			if (1 == sign_bit)
			{
				if (current_a < 15)
					wheelchair.level = TURNING_ONE;
				else if (current_a > 15 && current_a < 30)
					wheelchair.level = TURNING_TWO;
				else if (current_a > 30 && current_a < 45)
					wheelchair.level = TURNING_THREE;
				else if (current_a > 45 && current_a < 60)
					wheelchair.level = TURNING_FOUR;
				else if (current_a > 60 && current_a < 75)
					wheelchair.level = TURNING_FOUR;
				else if (current_a > 75 && current_a < 90)
					wheelchair.level = TURNING_FOUR;
			}
			else
			{
				if (current_a+15 > 0)
					wheelchair.level = TURNING_ONE;
				else if (current_a+15 < 0 && current_a+30 > 0)
					wheelchair.level = TURNING_TWO;
				else if (current_a+30 < 0 && current_a+45 > 0)
					wheelchair.level = TURNING_THREE;
				else if (current_a+45 < 0 && current_a+60 > 0)
					wheelchair.level = TURNING_FOUR;
				else if (current_a+60 < 0 && current_a+75 > 0)
					wheelchair.level = TURNING_FOUR;
				else if (current_a+75 < 0 && current_a+90 > 0)
					wheelchair.level = TURNING_FOUR;
			}

			switch (wheelchair.level)
			{
			case TURNING_ZERO:
				real_x = (uint16_t)2048;
				real_y = (uint16_t)3072;
				break;
			case TURNING_ONE:
				real_x = (uint16_t)(2048-1024*SIN_15*sign_bit);
				real_y = (uint16_t)(2048+1024*SIN_75);
				break;
			case TURNING_TWO:
				real_x = (uint16_t)(2048-1024*SIN_30*sign_bit);
				real_y = (uint16_t)(2048+1024*SIN_60);
				break;
			case TURNING_THREE:
				real_x = (uint16_t)(2048-1024*SIN_45*sign_bit);
				real_y = (uint16_t)(2048+1024*SIN_45);
				break;
			case TURNING_FOUR:
				real_x = (uint16_t)(2048-1024*SIN_60*sign_bit);
	//			real_x = (uint16_t)real_x*3/4;
				real_y = (uint16_t)(2048+1024*SIN_30);
	//			real_y = (uint16_t)real_y*3/4;
				break;
			case TURNING_FIVE:
				real_x = (uint16_t)(2048-1024*SIN_75*sign_bit);
				real_y = (uint16_t)(2048+1024*SIN_15);
				break;
			default:
				return;
			}

		}
		else
		{
			real_x = 2048;
			real_y = 1000;
		}
	}
	else
	{
		real_x = 2048;
		real_y = 1000;
	}

//	real_x = 2048;
//	real_y = 4096;
	
	wheelchair.data_X = (uint8_t)(real_x/100)&0xffff;
	wheelchair.verify_X = Crc8(wheelchair.data_X);
	wheelchair.data_x = (uint8_t)(real_x%100)&0xffff;
	wheelchair.verify_x = Crc8(wheelchair.data_x);
	wheelchair.data_Y = (uint8_t)(real_y/100)&0xffff;
	wheelchair.verify_Y = Crc8(wheelchair.data_Y);
	wheelchair.data_y = (uint8_t)(real_y%100)&0xffff;
	wheelchair.verify_y = Crc8(wheelchair.data_y);

	port_tx_msg((uint8_t *)&wheelchair.head_X, sizeof(wheelchair)-sizeof(TURNING_LEVEL));

}



void send_to_pc_hex(result_t *pRes)
{
//#if(NORMAL_USE==1)

	static int sort_counter;

        if (sort_counter < SORT_SIZE-1)
            sort_counter++;
        else
            sort_counter = 0;

	sort_X[sort_counter] = (int)pRes->x_cm;
	sort_Y[sort_counter] = (int)pRes->y_cm;
	sort_D[sort_counter] = (int)pRes->dist_cm;
//
	quickSort(sort_X, 0, SORT_SIZE-1);
	quickSort(sort_Y, 0, SORT_SIZE-1);
	quickSort(sort_D, 0, SORT_SIZE-1);

	pdoa_hex_pack_t packt;

	double tempDegree = atan2f(sort_X[SORT_SIZE/2], sort_Y[SORT_SIZE/2]);
//	
    tempDegree *= (180/M_PI);
	packt.head   = 0xA55A;
	packt.len    = 0x0F;
	packt.cmd    = 0x80;
	packt.x_cm 	 = sort_X[SORT_SIZE/2];
	packt.y_cm = sort_Y[SORT_SIZE/2];
//	packt.x_cm 	 = sort_D[SORT_SIZE/2];
//	packt.y_cm = (uint16_t)(tempDegree);
//	packt.signal_strength[1] = (int)signalPower_master[1];
//	packt.signal_strength[0] = (int)signalPower_master[0];
	packt.data_quality = (int)(signalPower_master[0] - signalPower_master[1]);
	packt.battery_power = (uint8_t)(pRes->acc_x >> 8 & 0xff);
	packt.key_value = (uint8_t)(pRes->acc_x & 0xff);
	packt.tag_ID = pRes->addr16;
	packt.base_status = 0xff;
	packt.reserved[1] = (int)pRes->tMaster_C;
	packt.reserved[0] = (int)pRes->tSlave_C;
	packt.crc16 = soft_crc16((uint8_t* )&packt.cmd, 13, 0xFFFF);
	port_tx_msg((uint8_t* )(&packt), sizeof(packt));

//	printf("ID = %s\r\n", dwt_readdevid());
	
	memset(signalPower_master, 0, sizeof(signalPower_master));
	memset(signalPower_slave, 0, sizeof(signalPower_slave));

//#else

//        static int sort_counter;
//	wheel_control.x_head = '!';
//	wheel_control.y_head = '!';
//	wheel_control.x_cmd = 'X';
//	wheel_control.y_cmd = 'Y';
//	wheel_control.end = '#';


//	if (pRes->dist_cm > 80)
//	{
//				if (sort_counter < SORT_SIZE)
//					sort_counter++;
//                                else
//                                      sort_counter = 0;

//				printf("counter = %d\r\n", sort_counter);

//				sort_X[sort_counter] = (int)pRes->x_cm;
//				sort_Y[sort_counter] = (int)pRes->y_cm;
//				sort_D[sort_counter] = (int)pRes->dist_cm;

//				quickSort(sort_X, 0, SORT_SIZE-1);
//				quickSort(sort_Y, 0, SORT_SIZE-1);
//				quickSort(sort_D, 0, SORT_SIZE-1);

//                                for (int i = 0; i < SORT_SIZE; i++)
//                                    printf("%d ", sort_X[i]);
//                                printf("\r\n");

//                                for (int i = 0; i < SORT_SIZE; i++)
//                                    printf("%d ", sort_Y[i]);
//                                printf("\r\n");

//				int x_cor = sort_X[SORT_SIZE/2];
//				int y_cor = sort_Y[SORT_SIZE/2];
//				int dis = sort_D[SORT_SIZE/2];

//                                printf("x=%d, y=%d, d=%d\r\n", x_cor, y_cor, dis);

//                if (x_cor != 0)    //turning
//                {
//                    wheel_control.x_data = (uint8_t)(100 - 100*x_cor/dis);
//                    wheel_control.x_data -= 20;
//                }
//                else
//                {
//                    wheel_control.x_data = 100;
//                }

//                wheel_control.y_data = (uint8_t)(100*y_cor/dis + 100);

//                if (pRes->dist_cm < 200 && pRes->dist_cm > 100)
//                {
//                    if (wheel_control.x_data < 30)
//                        wheel_control.x_data = 30;
//                    else if (wheel_control.x_data > 170)
//                        wheel_control.x_data = 170;
//                    else if (wheel_control.y_data > 170)
//                        wheel_control.y_data = 170;
//                }
//                else if (pRes->dist_cm < 100)
//                {
//                    if (wheel_control.x_data < 60)
//                        wheel_control.x_data = 60;
//                    else if (wheel_control.x_data > 140)
//                        wheel_control.x_data = 140;
//                    else if (wheel_control.y_data > 130)
//                        wheel_control.y_data = 130;
//                }

//                if (wheel_control.y_data < 100)
//                    wheel_control.y_data = 100;
//                else if (wheel_control.y_data > 200) 
//                    wheel_control.y_data = 200;
//#if(USE_ULTRASONIC==1)
//                if (wheel_control.collision_avoid == 0xFF)
//                {
//                    wheel_control.x_data = 100;
//                    wheel_control.y_data = 100;
//                }
//#endif
//	}
//        else
//        {
//            wheel_control.x_data = 100;
//            wheel_control.y_data = 100;
//        }

//        wheel_control.x_verify = Crc8(wheel_control.x_data);
//        wheel_control.y_verify = Crc8(wheel_control.y_data);

//        port_tx_msg((uint8_t* )(&wheel_control), sizeof(wheel_control_t)-1);

//#endif
}
/*
 * @brief function to report to PC a new tag was discovered
 *
 * 'JSxxxx{"NewTag":
 *             <string>//address64, string
 *        }'
 *
 * */
void signal_to_pc_new_tag_discovered(uint64_t addr64)
{
    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        int  hlen;

        hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object

        sprintf(&str[strlen(str)],"{\"NewTag\":\"%08lX%08lX\"}", (uint32_t)(addr64>>32), (uint32_t)addr64);

        sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will kill first '{'
        str[hlen]='{';                            //restore the start bracket

        sprintf(&str[strlen(str)],"\r\n");
        port_tx_msg((uint8_t*)str, strlen(str));

        CMD_FREE(str);
    }
}


/*
 * @brief This is a report of twr to pc
 *
 * There are two modes of operation: JSON(long output) or plain(short output)
 * JSON (default):
 *  'JSxxxx{"TWR":
 *    {     "a16":%04X, //addr16
 *          "R":%d,//range num
 *          "T":%d,//sys timestamp of Final WRTO Node's SuperFrame start, us
 *          "D":%f,//distance
 *          "P":%f,//raw pdoa
 *          "A":%f,//corrected angle
 *          "O":%f,//clock offset in hundreds part of ppm
 *          "V":%d //service message data from the tag: (stationary, etc)
 *          "X":%d //service message data from the tag: (stationary, etc)
 *          "Y":%d //service message data from the tag: (stationary, etc)
 *          "Z":%d //service message data from the tag: (stationary, etc)
 *    }
 *   }'
 *
 * Plain:
 * used if any from below is true:
 * diag, acc,
 * */
 static char g_twr_str[MAX_STR_SIZE];
void send_to_pc_twr(result_t *pRes)
{
    char *str = g_twr_str;//CMD_MALLOC(MAX_STR_SIZE);
    int  hlen;
	static int sort_counter = 0;

    if(str)
    {
        if (app.pConfig->s.accEn ==1 || \
            app.pConfig->s.diagEn ==1 || \
            app.pConfig->s.reportLevel > 1)
        {
            if(app.pConfig->s.reportLevel == 3)
            {
                /* shortest "AR" output: 18 chars per location: ~640 locations per second
                 * */
                sprintf(str, "AR%04X%02X%08lX%08lX",
                        (uint16_t)(pRes->addr16),
                        (uint8_t) (pRes->rangeNum),
                        (long int)(pRes->x_cm),
                        (long int)(pRes->y_cm));
            }
            else
            {
                /* optimum "RA" output: 58 chars per location: ~200 locations per second
                 * */
                sprintf(str, "RA%04X %02X %08lX %08lX %08lX %1X X:%04X Y:%04X Z:%04X",
                        (uint16_t)(pRes->addr16),
                        (uint8_t) (pRes->rangeNum),
                        (long int)(pRes->x_cm),
                        (long int)(pRes->y_cm),
                        (long int)(pRes->clockOffset_pphm),
                        (uint8_t) (pRes->flag),
                        (uint16_t)(pRes->acc_x),
                        (uint16_t)(pRes->acc_y),
                        (uint16_t)(pRes->acc_z));
            }
            sprintf(&str[strlen(str)],"\r\n");
            port_tx_msg((uint8_t*)str, strlen(str));
        }
        else if (app.pConfig->s.reportLevel == 1)
        {
            /* use JSON type of output during a normal operation
             *
             * This is not very efficient, as one TWR location is 107 chars, as per format below
             * JS  62{"TWR": {"a16":"2E5C","R":3,"T":8605,"D":343,"P":1695,"O":14,"V":1,"X":53015,"Y":60972,"Z":10797}}
             *
             * For pure UART, with limit of 115200b/s, the channel can handle ~100 locations per second,
             * i.e. 10 tags ranging on maximum rate of 10 times a second.
             * For higher throughput limit amount of JSON TWR object or use plain output instead.
             *
             * */

            /* Floating point values are standard for JSON objects, however the floating point printing
             * is not used in current application.
             * If the fixed point printing required, will need to add "-u _printf_float" to the
             * linker string, to include "floating printf" to the nano.spec of the stdlib.
             * This will increase the size of application by ~6kBytes and the floating printing also requires
             * much more stack space.
             * Use this with caution, as this might result unpredictable stack overflow / hard fault.
             * */
            hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object

            sprintf(&str[strlen(str)],"{\"TWR\": ");

            sprintf(&str[strlen(str)],
                    "{\"a16\":\"%04X\","
                    "\"R\":%d," //range number
                    "\"T\":%d,",//sys timestamp of Final WRTO Node's SuperFrame start, us
                    (int)(pRes->addr16),
                    (int)(pRes->rangeNum),
                    (int)(pRes->resTime_us));

            if(app.pConfig->s.debugEn)
            {
                sprintf(&str[strlen(str)],
                    "\"Tm\":%d,"    //Master's temperature, in degree centigrade
                    "\"Ts\":%d,",   //Slave's temperature, in degree centigrade
                    (int)(pRes->tMaster_C),
                    (int)(pRes->tSlave_C));
            }

			if (++sort_counter == SORT_SIZE)
				sort_counter = 0;
			
			sort_X[sort_counter] = (int)pRes->x_cm;
			sort_Y[sort_counter] = (int)pRes->y_cm;
			sort_D[sort_counter] = (int)pRes->dist_cm;

			quickSort(sort_X, 0, SORT_SIZE-1);
			quickSort(sort_Y, 0, SORT_SIZE-1);
			quickSort(sort_D, 0, SORT_SIZE-1);
			

            sprintf(&str[strlen(str)],
                    "\"D\":%d," //distance as int
                    "\"P\":%d," //pdoa  as int in milli-radians
                    "\"Xcm\":%d,"   //X distance wrt Node in cm
                    "\"Ycm\":%d,",  //Y distance wrt Node in cm
//                    (int)(pRes->dist_cm),
					(sort_D[SORT_SIZE/2]),
                    (int)(pRes->pdoa_raw_deg),
//                    (int)(pRes->x_cm),
//                    (int)(pRes->y_cm));
					(sort_X[SORT_SIZE/2]),
					(sort_Y[SORT_SIZE/2]));

            sprintf(&str[strlen(str)],
                    "\"O\":%d,"//clock offset as int
                    "\"V\":%d," //service message data from the tag: (bitmask: bit0 = stationary, bit15 = zeroed pdoaOffset used; bit14 = zeroed rngOffset used)
                    "\"X\":%d," //Normalized accel data X from the Tag, mg
                    "\"Y\":%d," //Normalized accel data Y from the Tag, mg
                    "\"Z\":%d"  //Normalized accel data Z from the Tag, mg
                    "}",
                    (int)(pRes->clockOffset_pphm),
                    (int)(pRes->flag),
                    (int)(pRes->acc_x),
                    (int)(pRes->acc_y),
                    (int)(pRes->acc_z));

            sprintf(&str[strlen(str)],"}");

            sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will kill first '{'
            str[hlen]='{';                            //restore the start bracket

            sprintf(&str[strlen(str)],"\r\n");
			//printf("dataRate = %u\r\n", app.pConfig->dwt_config.dataRate);
            port_tx_msg((uint8_t*)str, strlen(str));
           // printf("%s", str);
        }
        else if(app.pConfig->s.reportLevel==0)
        {
#if(NORMAL_USE==1)
                send_to_pc_hex(pRes);
//				  osDelay (800);
#else
				send_to_wheelchair(pRes);
#endif
        }
        else
        {
            //no output
        }

       // CMD_FREE(str);
    }
}


/* @brief input "str" must be a null-terminated string with enough space in it.
 *        this is slow output of accumulator from the chip, starting with ACC_OFFSET value.
 *        To be used solely for debug purposes.
 * */
static void send_acc(char       *str,
                     uint16_t   maxLen,
                     uint8_t    *pAcc)
{
    int     n;
    int16   cmplex_m[2];

    cmplex_m[0] = 0;
    cmplex_m[1] = 0;

    n = strlen(str);

    for(int i = 1; i <= (FULL_ACC_LEN*4); i+=4)
    {
        if(n >= (maxLen - 4))
        {
            while(port_tx_msg((uint8_t*)str, n) !=_NO_ERR)
            {
                osThreadYield();//force switch content
                osDelay(5);     //wait 5ms for Flush thread freed the buffer
            }
            n = 0;
        }

        if(i > (ACC_OFFSET*4))
        {
            memcpy(&cmplex_m[0], &pAcc[i], 4);
        }

        n += sprintf(&str[n], "%04X%04X", (cmplex_m[0] & 0xFFFF), (cmplex_m[1]&0xFFFF));
    }

    n += sprintf(&str[n], "\r\n");

    while(port_tx_msg((uint8_t*)str, n) !=_NO_ERR)
    {
        osThreadYield();    //force switch content
        osDelay(5);         //wait 5ms for Flush thread freed the buffer
    }
}

/* @brief input "str" must be a null-terminated string with enough space in it.
 *        To be used solely for debug purposes.
 * */
static void send_diag(char    *str,
                      uint16_t maxLen,
                      uint8_t *pDiag,
                      uint8_t *acc5,
                      uint8_t sfdangle)
{
    if ((strlen(str)+ 2*sizeof(diag_v5_t) + 1 + 10 + 6) < maxLen)
    {
        for(int i=0; i<sizeof(diag_v5_t); i++)
        {
            sprintf(&str[strlen(str)], "%02X", pDiag[i]);
        }

        sprintf(&str[strlen(str)], " ");

        for(int i=0; i<5; i++)
        {
            sprintf(&str[strlen(str)], "%02X", acc5[i]);
        }

        sprintf(&str[strlen(str)], " %02X;\r\n", sfdangle);

        while(port_tx_msg((uint8_t*)str, strlen(str)) !=_NO_ERR)
        {
            osThreadYield();    //force switch content
            osDelay(5);         //wait 5ms for Flush thread freed the buffer
        }
    }
}

/* @brief send acc & diagnostics information
 *           these are blocking operations
 *
 * */
void send_to_pc_diag_acc(rx_mail_t *pRxMailPckt)
{
    static int logNum = 0;

    char *str = CMD_MALLOC(MAX_STR_SIZE);
    uint8_t *p;

    if(str)
    {
        //send the Accumulator information from the pRxMailPckt
        if(app.pConfig->s.accEn == 1)
        {
            /* "master chip" */
            p = (uint8_t*)&pRxMailPckt->acc[0];

            sprintf(str, "\r\nAM%04X %02X CLKOFF: %d\r\n", logNum, pRxMailPckt->res.rangeNum,
                    (int)(pRxMailPckt->res.clockOffset_pphm));

            send_acc(str, MAX_STR_SIZE, p);

            /* "slave chip" */
            p = (uint8_t*)&pRxMailPckt->acc[1];
            sprintf(str, "\r\nAS%04X %02X\r\n", logNum, pRxMailPckt->res.rangeNum);

            send_acc(str, MAX_STR_SIZE, p);
        }

        //send the Diagnostics information from the pRxMailPckt
        if(app.pConfig->s.diagEn == 1)
        {
            /* "master chip" */
            p = (uint8_t*)&pRxMailPckt->diagnostics[0];
            sprintf(str, "DM%04X %02X ", logNum, pRxMailPckt->res.rangeNum);
            send_diag(str, MAX_STR_SIZE, p,
                      pRxMailPckt->pdoa_info.acc_master,
                      pRxMailPckt->pdoa_info.sfdangle_master);

            /* "slave chip" */
            p = (uint8_t*)&pRxMailPckt->diagnostics[1];
            sprintf(str, "DS%04X %02X ", logNum, pRxMailPckt->res.rangeNum);
            send_diag(str, MAX_STR_SIZE, p,
                    pRxMailPckt->pdoa_info.acc_slave,
                    pRxMailPckt->pdoa_info.sfdangle_slave);
        }

        CMD_FREE(str);
    }

    logNum++;
}


/* @brief
 *  Send Service message from Node:
 *  Currently sending stationary to the PC
 *
 *  'JSxxxx{"SN":
 *    {     "a16": %04X, //addr16 of the Node
 *            "V":%d //service message from the Node (stationary is bit 0)
 *          "X":%d //Normalized accel data X from the Node, mg
 *          "Y":%d //Normalized accel data Y from the Node, mg
 *          "Z":%d //Normalized accel data Z from the Node, mg
 *    }
 *   }'
 * */
void send_to_pc_stationary(stationary_res_t *p)
{
    int hlen;

    char *str = CMD_MALLOC(MAX_STR_SIZE);

    if(str)
    {
        if (app.pConfig->s.accEn == 1 || \
            app.pConfig->s.diagEn == 1 || \
            app.pConfig->s.reportLevel > 1)
        {

            /* shortest service "SN" output */
            sprintf(str, "SN%1X %04X%04X%04X\r\n",
                    (uint8_t)  (p->flag),
                    (uint16_t) (p->acc_x),
                    (uint16_t) (p->acc_y),
                    (uint16_t) (p->acc_z));
            port_tx_msg((uint8_t*)str, strlen(str));
        }
        else if (app.pConfig->s.reportLevel == 1)
        {
            /* use JSON type of output during a normal operation */
            hlen = sprintf(str,"JS%04X", 0x5A5A);    // reserve space for length of JS object

            sprintf(&str[strlen(str)],"{\"SN\": ");

            sprintf(&str[strlen(str)],
                    "{\"a16\":\"%04X\","
                    "\"V\":%d," //service message data from the Node: (stationary is bit 0)
                    "\"X\":%d," //Normalized accel data X from the Node, mg
                    "\"Y\":%d," //Normalized accel data Y from the Node, mg
                    "\"Z\":%d"  //Normalized accel data Z from the Node, mg
                    "}}",
                    (int)(p->addr),
                    (int)(p->flag),
                    (int)(p->acc_x),
                    (int)(p->acc_y),
                    (int)(p->acc_z));

            sprintf(&str[2],"%04X",strlen(str)-hlen);//add formatted 4X of length, this will kill first '{'
            str[hlen]='{';                          //restore the start bracket

            sprintf(&str[strlen(str)],"\r\n");
            port_tx_msg((uint8_t*)str, strlen(str));
        }				
        else
        {
            //no output
        }
        CMD_FREE(str);
    }
}
