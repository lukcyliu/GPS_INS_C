//
// Created by aos on 17-6-5.
//

#ifndef QUEUETEST_QUEUE_H
#define QUEUETEST_QUEUE_H

#include <stdio.h>
#include "FileStruct.h"
typedef struct CircleQueue{
    struct Data Qdata[1001];
    int front,rear,count;
};
void InitQueue(struct CircleQueue *queue);//初始化队列
void EnQueue(struct CircleQueue *queue, struct Data e);//进队列
struct Data DeQueue(struct CircleQueue *queue);//出队列
int isEmpty(struct CircleQueue *queue);
#endif //QUEUETEST_QUEUE_H
