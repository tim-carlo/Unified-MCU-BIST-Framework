#ifndef STACK_H
#define STACK_H

#include <stdlib.h>
#include <string.h>


/**
 * @brief Stack data structure for managing a collection of elements
 */
typedef struct {
    int top;
    int capacity;
    size_t element_size;
    void *array;
} Stack;

Stack* createStack(int capacity, size_t element_size);
int isFull(Stack* stack);
int isStackEmpty(Stack* stack);
void push(Stack* stack, void *item);
void pop(Stack* stack, void *out);
void peek(Stack* stack, void *out);
void freeStack(Stack* stack);

#endif // STACK_H
