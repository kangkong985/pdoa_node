#ifndef __DATA_FILTER_H
#define __DATA_FILTER_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"

#define DYNAMIC_MSG_STR_ALLOW

#if defined(DYNAMIC_MSG_STR_ALLOW)
#define MYQUEUE_MALLOC              pvPortMalloc
#define MYQUEUE_FREE                vPortFree
#define MAXQUEUE_SIZE				10
#endif

#define MOTIONFILTER_SIZE			4

typedef struct
{
	bool motionFilter_Ready;
	uint8_t  filterHis_index;
	double estXHis[MOTIONFILTER_SIZE];
	double estYHis[MOTIONFILTER_SIZE];
}__attribute__((packed))tag_report_t;

typedef enum
{
	SUCCESSED = 0,
	FAILED,
}myQueue_status;

typedef struct
{
	float *pBase;
	float front;
	float rear;
	int maxSize;
}myQueue;

//创建空队�
myQueue_status initQueue(myQueue *pQueue, int queueCapacity);
//删除队列
void destroyQueue(myQueue *pQueue);
//清空队列
void clearQueue(myQueue *pQueue);
//队列是否为空
myQueue_status isEmpty(myQueue *pQueue);
//队列是否为满
myQueue_status isFull(myQueue *pQueue);
//新元素入队，并返回数�
float enQueue(myQueue *pQueue, float *element);

void motionFilter(double* x, double* y);

#endif
