#include "delay.h"

static const int LOOPS_FOR_1_MS = 2450;
static const int LOOPS_FOR_1_US = 2;


void delay_ms(unsigned long delay)
{
    delay *= LOOPS_FOR_1_MS;

    while (delay) {
        delay--;
    }
}

void delay_us(unsigned long delay)
{
    delay *= LOOPS_FOR_1_US;

    while (delay) {
        delay--;
    }
}
