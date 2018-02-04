#ifndef STEPPER_CONTROL_MOTOR_H
#define STEPPER_CONTROL_MOTOR_H

//Constants from the AVR App note
#define SPR 96                                        //steps per rev
#define T1_FREQ 1382400                               // timer freq
#define ALPHA (2*3.14159/SPR)                         // 2*pi/spr
#define A_T_x100 ((uint32_t)(ALPHA*T1_FREQ*100))      // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((uint32_t)((T1_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676
#define A_SQ (uint32_t)(ALPHA*2*10000000000)          // ALPHA*2*10000000000
#define A_x20000 (uint32_t)(ALPHA*20000)              // ALPHA*20000


void motor_init();
void motor_goto(uint32_t step, uint16_t speed, uint32_t acceleration, uint32_t deceleration);
void motor_off();
bool motor_is_running();

#endif
