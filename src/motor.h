#ifndef STEPPER_CONTROL_MOTOR_H
#define STEPPER_CONTROL_MOTOR_H

static const long HIGHEST_SQRT_RESULT_BIT = 0x40000000L;

static void update_output();
static void timer_interrupt_handler();

void motor_init();
void motor_goto(int new_position);
void motor_off();

#endif
