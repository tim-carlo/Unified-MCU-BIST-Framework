#include "stack.h"


Stack* createStack(int capacity, size_t element_size) {
    Stack *stack = (Stack*)malloc(sizeof(Stack));
    stack->capacity = capacity;
    stack->top = -1;
    stack->element_size = element_size;
    stack->array = malloc(stack->capacity * stack->element_size);
    return stack;
}


int isFull(Stack* stack) {
    return stack->top == stack->capacity - 1;
}


int isEmpty(Stack* stack) {
    return stack->top == -1;
}


void push(Stack* stack, void *item) {
    if (isFull(stack))
        return;
    void *target = (char*)stack->array + (++stack->top * stack->element_size);
    memcpy(target, item, stack->element_size);
}


void pop(Stack* stack, void *out) {
    if (isEmpty(stack)) {
        if (out) memset(out, 0, stack->element_size);
        return;
    }
    void *source = (char*)stack->array + (stack->top * stack->element_size);
    memcpy(out, source, stack->element_size);
    stack->top--;
}

int isStackEmpty(Stack* stack) {
    return stack->top == -1;
}


void peek(Stack* stack, void *out) {
    if (isEmpty(stack)) {
        if (out) memset(out, 0, stack->element_size);
        return;
    }
    void *source = (char*)stack->array + (stack->top * stack->element_size);
    memcpy(out, source, stack->element_size);
}


void freeStack(Stack* stack) {
    free(stack->array);
    free(stack);
}
