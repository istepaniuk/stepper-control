#ifndef CATFEEDER_MOTOR_H
#define CATFEEDER_MOTOR_H

static void update_output();

void motor_off();
void motor_setup();
void motor_step_forward();
void motor_step_backwards();
void motor_goto(int new_position);

#endif
