#ifndef QUEUE_H
#define QUEUE_H

#define MAX_SIZE 100

#include <stdbool.h>
#include <stddef.h>

typedef struct
{
    void* items[MAX_SIZE];
    int front;
    int rear;
} Queue;

// Queue operations
void initQueue(Queue *q);
bool isEmpty(Queue *q);
bool isFull(Queue *q);
void enqueue(Queue* q, void* value);
void* dequeue(Queue* q);  // Modified to return value if needed

#endif // QUEUE_H