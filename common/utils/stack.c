#include "stack.h"
#include "printf.h"

/**
 * @brief Create a Stack object
 * 
 * @param capacity Maximum number of elements in the stack
 * @param element_size Size of each element in the stack
 * @return Stack* A pointer to the created stacks
 */
Stack* createStack(int capacity, size_t element_size) {
    printf("Creating stack with capacity %d and element size %zu\n", capacity, element_size);

    Stack *stack = (Stack*)malloc(sizeof(Stack));
    if (!stack) {
        printf("ERROR: malloc for stack failed!\n");
        return NULL;
    }

    stack->capacity = capacity;
    stack->top = -1;
    stack->element_size = element_size;
    stack->array = malloc(stack->capacity * stack->element_size);

    if (!stack->array) {
        printf("ERROR: malloc for stack array failed!\n");
        free(stack);
        return NULL;
    }

    return stack;
}

/**
 * @brief Check if the stack is full
 * 
 * @param stack Pointer to the stack
 * @return int 1 if full, 0 otherwise
 */
int isFull(Stack* stack) {
    return stack->top == stack->capacity - 1;
}

/**
 * @brief Check if the stack is empty
 * 
 * @param stack Pointer to the stack
 * @return int 1 if empty, 0 otherwise
 */
int isEmpty(Stack* stack) {
    return stack->top == -1;
}

/**
 * @brief Push an item onto the stack
 * 
 * @param stack Pointer to the stack
 * @param item Pointer to the item to be pushed onto the stack
 */
void push(Stack* stack, void *item) {
    if (isFull(stack))
        return;
    void *target = (char*)stack->array + (++stack->top * stack->element_size);
    memcpy(target, item, stack->element_size);
}

/**
 * @brief Pop an item from the stack
 * 
 * @param stack Pointer to the stack
 * @param out Pointer to the location where the popped item will be stored
 */
void pop(Stack* stack, void *out) {
    if (isEmpty(stack)) {
        if (out) memset(out, 0, stack->element_size);
        return;
    }
    void *source = (char*)stack->array + (stack->top * stack->element_size);
    memcpy(out, source, stack->element_size);
    stack->top--;
}

/**
 * @brief Peek at the top item of the stack without removing it
 * 
 * @param stack Pointer to the stack
 * @param out Pointer to the location where the top item will be stored
 */
int isStackEmpty(Stack* stack) {
    return stack->top == -1;
}

/**
 * @brief Peek at the top item of the stack without removing it
 * 
 * @param stack Pointer to the stack
 * @param out Pointer to the location where the top item will be stored
 */
void peek(Stack* stack, void *out) {
    if (isEmpty(stack)) {
        if (out) memset(out, 0, stack->element_size);
        return;
    }
    void *source = (char*)stack->array + (stack->top * stack->element_size);
    memcpy(out, source, stack->element_size);
}

/**
 * @brief Free the stack and its resources
 * 
 * @param stack Pointer to the stack to be freed
 */
void freeStack(Stack* stack) {
    free(stack->array);
    free(stack);
}
