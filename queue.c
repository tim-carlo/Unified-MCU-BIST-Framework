#include "queue.h"
#define MAX_SIZE 100


void initQueue(Queue *q) {
    q->front = -1;
    q->rear = 0;
}

bool isEmpty(Queue *q) {
    return (q->front == q->rear - 1);
}

bool isFull(Queue *q) {
    return (q->rear == MAX_SIZE);
}

// Enqueue a pointer to any type of value
void enqueue(Queue* q, void* value) {
    if (isFull(q)) {
        return;
    }
    q->items[q->rear] = value;
    q->rear++;
}

// Dequeue does not free memory, just advances the pointer
void dequeue(Queue* q) {
    if (isEmpty(q)) {
        return;
    }
    q->front++;
    free(q->items[q->front]); 
    q->items[q->front] = NULL;
}