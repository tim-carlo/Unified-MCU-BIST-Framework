#include "stack.h"
#include "printf.h"

/**
 * @brief Create a Stack object
 * 
 * @param capacity Maximum number of elements in the stack
 * @param element_size Size of each element in the stack
 * @return Stack* A pointer to the created stacks
 */
Stack* create_stack(int capacity, size_t element_size) {

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
int stack_is_full(Stack* stack) {
    return stack->top == stack->capacity - 1;
}

/**
 * @brief Check if the stack is empty
 * 
 * @param stack Pointer to the stack
 * @return int 1 if empty, 0 otherwise
 */
int stack_is_empty(Stack* stack) {
    return stack->top == -1;
}

/**
 * @brief stack_push an item onto the stack
 * 
 * @param stack Pointer to the stack
 * @param item Pointer to the item to be pushed onto the stack
 */
void stack_push(Stack* stack, void *item) {
    if (stack_is_full(stack))
        return;
    void *target = (char*)stack->array + (++stack->top * stack->element_size);
    memcpy(target, item, stack->element_size);
}

/**
 * @brief stack_pop an item from the stack
 * 
 * @param stack Pointer to the stack
 * @param out Pointer to the location where the popped item will be stored
 */
void stack_pop(Stack* stack, void *out) {
    if (stack_is_empty(stack)) {
        if (out) memset(out, 0, stack->element_size);
        return;
    }
    void *source = (char*)stack->array + (stack->top * stack->element_size);
    memcpy(out, source, stack->element_size);
    stack->top--;
}

/**
 * @brief stack_peek at the top item of the stack without removing it
 * 
 * @param stack Pointer to the stack
 * @param out Pointer to the location where the top item will be stored
 */
int is_stack_empty(Stack* stack) {
    return stack->top == -1;
}

/**
 * @brief stack_peek at the top item of the stack without removing it
 * 
 * @param stack Pointer to the stack
 * @param out Pointer to the location where the top item will be stored
 */
void stack_peek(Stack* stack, void *out) {
    if (stack_is_empty(stack)) {
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
void free_stack(Stack* stack) {
    free(stack->array);
    free(stack);
}
