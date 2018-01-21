#ifndef CATFEEDER_INTERRUPTS_H
#define CATFEEDER_INTERRUPTS_H

#include <stm32f10x.h>


void set_exti_line_interrupt_callback(int line, void *callback);
void set_timer2_interrupt_callback(void *callback);

#endif
