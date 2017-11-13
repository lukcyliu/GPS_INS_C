//
// Created by AosCh on 2017/11/13.
//

#include "QueueWindow.h"
void w_InitQueue(struct QueueWindow *queue,int size){
    queue->size = size;
    queue->sum = 0.0;
    queue->data = (double*)malloc(sizeof(double) * (size + 1));
    queue->count = queue->front = queue->rear = 0;
}

void w_EnQueue(struct QueueWindow *queue, double e){
    if(queue->count == queue->size){
        printf("Queue is full\n");
        return;
    }else{
        queue->rear = (queue->rear + 1) % (queue->size + 1);
        queue->data[queue->rear] = e;
        queue->sum += e;
        queue->count++;
    }
}
double w_DeQueue(struct QueueWindow *queue){
    if(queue->front == queue->rear){
        printf("Queue is empty\n");
        double result;
        return result;
    }
    queue->front = (queue->front + 1) % (queue->size + 1);
    double result = queue->data[queue->front];
    queue->count--;
    queue->sum -= result;
    return result;
}
int w_isEmpty(struct QueueWindow *queue){
    if(queue->front == queue->rear)
        return 1;
    else
        return 0;
}
int w_isFull(struct QueueWindow *queue){
    if(queue->count == queue->size)
        return 1;
    else
        return 0;
}
double w_getAVG(struct QueueWindow *queue){
    if(w_isEmpty(queue))
        return 0;
    return queue->sum / queue->count;
}