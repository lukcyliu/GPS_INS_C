//
// Created by AosCh on 2017/11/13.
//

#ifndef REAL_TIME_QUEUEWINDOW_H
#define REAL_TIME_QUEUEWINDOW_H
#include <stdio.h>
#include <stdlib.h>
typedef struct QueueWindow{
    double *data;
    int front,rear,count,size;
    double sum;
};
void w_InitQueue(struct QueueWindow *queue,int size);//初始化队列
void w_EnQueue(struct QueueWindow *queue, double e);//进队列
double w_DeQueue(struct QueueWindow *queue);//出队列
int w_isEmpty(struct QueueWindow *queue);
int w_isFull(struct QueueWindow *queue);
double w_getAVG(struct QueueWindow *queue);
#endif //REAL_TIME_QUEUEWINDOW_H
