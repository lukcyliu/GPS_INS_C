//
// Created by aos on 17-6-5.
//

#include "Queue.h"

void InitQueue(struct CircleQueue *queue){
    queue->count = queue->front = queue->rear = 0;
}

void EnQueue(struct CircleQueue *queue, struct Data e){
    if(queue->count == 1000){
        printf("Queue is full\n");
        return;
    }else{
        queue->rear = (queue->rear + 1) % 1001;
        queue->Qdata[queue->rear] = e;
        queue->count++;
    }
}
struct Data DeQueue(struct CircleQueue *queue){
    if(queue->front == queue->rear){
        printf("Queue is empty\n");
        struct Data result;
        return result;
    }
    queue->front = (queue->front + 1) % 1001;
    struct Data result = queue->Qdata[queue->front];
    queue->count--;
    return result;
}
int isEmpty(struct CircleQueue *queue){
    if(queue->front == queue->rear)
        return 1;
    else
        return 0;
}
