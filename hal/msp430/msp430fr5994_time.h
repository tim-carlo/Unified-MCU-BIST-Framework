typedef enum
{
    TIMER_A0,
    TIMER_A1,
    TIMER_A2,
    TIMER_A4,
    TIMER_B0
} timer_type;

#define GET_TxxCTL(timer) ((volatile uint16_t *)((timer) == TIMER_A0 ? &TA0CTL : (timer) == TIMER_A1 ? &TA1CTL \
                                                       : (timer) == TIMER_A2   ? &TA2CTL \
                                                       : (timer) == TIMER_A4   ? &TA4CTL \
                                                                               : &TB0CTL))

#define GET_TxxR(timer) ((volatile uint16_t *)((timer) == TIMER_A0 ? &TA0R : (timer) == TIMER_A1 ? &TA1R \
                                                   : (timer) == TIMER_A2   ? &TA2R \
                                                   : (timer) == TIMER_A4   ? &TA4R \
                                                                           : &TB0R))

#define GET_TxxCCTL0(timer) ((volatile uint16_t *)((timer) == TIMER_A0 ? &TA0CCTL0 : (timer) == TIMER_A1 ? &TA1CCTL0 \
                                                           : (timer) == TIMER_A2   ? &TA2CCTL0 \
                                                           : (timer) == TIMER_A4   ? &TA4CCTL0 \
                                                                                   : &TB0CCTL0))
#define GET_TxxCCR0(timer) ((volatile uint16_t *)((timer) == TIMER_A0 ? &TA0CCR0 : (timer) == TIMER_A1 ? &TA1CCR0 \
                                                   : (timer) == TIMER_A2   ? &TA2CCR0 \
                                                   : (timer) == TIMER_A4   ? &TA4CCR0 \
                                                                           : &TB0CCR0))                                                                                   

#define TICKS_PER_OVERFLOW 65536UL