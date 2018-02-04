#ifndef STEPPER_CONTROL_MOTOR_H
#define STEPPER_CONTROL_MOTOR_H

//Constants from the AVR App note
#define SPR 96                                        // steps per rev
#define T1_FREQ 500000                               // timer freq


// ALPHA is the angle of a single step, in radians
// 2 * pi / SPR
#define ALPHA (2 * 3.14159 / SPR)
// = 0.065449792 rad

// ALPHA * 2 * 10000000000
#define ALPHA_SQR (uint32_t)(ALPHA * 2 * 10000000000)
// = 1,308,995,840 ?

// ALPHA * 20000
#define ALPHA_20K (uint32_t)(ALPHA * 20000)
// = 1308 ?


// (ALPHA / T1_FREQ) * 100
#define A_T_x100 ((uint32_t)(ALPHA * T1_FREQ * 100))
// used as (A_T_x100 / speed) to get the minimum period (cruise speed)
// 0.0654 rad * 1000000 Hz * 100
// = 6544979.167 rad/s

// divided by 100 and scaled by 0.676
#define T1_FREQ_148 ((uint32_t)((T1_FREQ * 0.676) / 100))
// used to calculate first first step period (c0)
// = 6760 ?

void motor_init();
void motor_goto(uint32_t step, uint16_t speed, uint32_t acceleration, uint32_t deceleration);
void motor_off();
bool motor_is_running();

#endif
