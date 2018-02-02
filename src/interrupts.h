#ifndef STEPPER_CONTROL_INTERRUPTS_H
#define STEPPER_CONTROL_INTERRUPTS_H

#include <stm32f10x.h>


void interrupt_set_exti_line_callback(int line, void *callback);
void interrupt_set_timer2_callback(void *callback);

#endif
