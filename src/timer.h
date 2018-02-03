#ifndef STEPPER_CONTROL_TIMER_H
#define STEPPER_CONTROL_TIMER_H


void timer2_init(uint16_t period, uint16_t prescaler);
void timer2_start(void);
void timer2_stop(void);
uint16_t timer2_get_current_counter(void);
void timer2_set_period(uint16_t period);

#endif