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

Stack* create_stack(int capacity, size_t element_size);
int stack_is_full(Stack* stack);
int isStackEmpty(Stack* stack);
void stack_push(Stack* stack, void *item);
void stack_pop(Stack* stack, void *out);
void peek(Stack* stack, void *out);
void freeStack(Stack* stack);

#endif // STACK_H
