#include <stdbool.h>
#include <stm32f10x.h>
#include "motor.h"
#include "hardware.h"
#include "delay.h"
#include "timer.h"
#include "interrupts.h"
#include "usart.h"

static const uint8_t STEP_OUTPUTS[] = {
        0b0001,
        0b0011,
        0b0010,
        0b0110,
        0b0100,
        0b1100,
        0b1000,
        0b1001
};

typedef enum { STOP, ACCEL, RUN, DECEL } status_t;

typedef struct {
    status_t status;
    uint16_t step_period;
    uint16_t min_step_period;
    uint16_t period_rest;
    uint32_t decel_start_position;
    int16_t accel_count;
    uint32_t step_count;
    int16_t decel_initial_accel_count;
    uint16_t last_period_during_accel;
} speed_ramp_t;

speed_ramp_t ramp;

static uint32_t current_position = 0;
static uint32_t motor_ticks = 0;


static void update_step_period(speed_ramp_t *ramp);
static void timer_interrupt_handler();
static void update_output();
static uint32_t sqrt(uint32_t x);


void motor_init()
{
    current_position = 0;

    gpio_set_pin_mode(&MOTOR0_PIN, GPIO_MODE_OUT_PUSH_PULL);
    gpio_set_pin_mode(&MOTOR1_PIN, GPIO_MODE_OUT_PUSH_PULL);
    gpio_set_pin_mode(&MOTOR2_PIN, GPIO_MODE_OUT_PUSH_PULL);
    gpio_set_pin_mode(&MOTOR3_PIN, GPIO_MODE_OUT_PUSH_PULL);

    ramp.status = STOP;

    //F = CLK/((PSC + 1)*(ARR + 1))
    //24MHz CLK/(1000-1)*(24-1) = 1 KHz ISR
    //24MHz CLK/(100-1)*(24-1) = 10 KHz ISR
    //24MHz CLK/(10-1)*(24-1) = 100 KHz ISR
    timer2_init(100 - 1, 24 - 1);
    timer2_start();
    interrupt_set_timer2_callback(timer_interrupt_handler);

    motor_off();
}

void motor_goto(uint32_t step, uint16_t speed, uint32_t acceleration, uint32_t deceleration)
{
    current_position = 0;
    motor_ticks = 0;

    //speed   in 0.01 * rad/sec
    //acc/dec in 0.01 * rad/sec^2

    usart_puts("\n\n======== GOTO\n");
    ramp.period_rest = 0;
    ramp.accel_count = 0;

    // Cruise speed
    ramp.min_step_period = (uint16_t) (A_T_x100 / speed);
    usart_puts("cruise:  ");
    printint(ramp.min_step_period); usart_puts("\n");


    // Set acelration by calc the first (c0) step delay .
    // step_delay = 1 / tt * sqrt(2 * alpha / acceleration)
    // step_delay =
    //      (tfreq * 0.676 / 100)
    //      * 100
    //      * sqrt( (2 * alpha * 10000000000) / (accel * 100) )
    //      / 10000
    ramp.step_period = (uint16_t) ((T1_FREQ_148 * sqrt(ALPHA_SQR / acceleration)) / 100);
    usart_puts("period:  ");
    printint(ramp.step_period); usart_puts("\n");


    // Find out after how many steps does the speed hit the max speed limit.
    // max_s_lim = speed^2 / (2 * alpha * accel)
    uint32_t max_s_lim;
    max_s_lim = speed * speed / (uint32_t) (((uint32_t) ALPHA_20K * acceleration) / 100);
    // If we hit max speed limit before 0,5 step it will round to 0.
    // But in practice we need to move at least 1 step.
    if (max_s_lim == 0) {
        max_s_lim = 1;
    }


    // Find out after how many steps we must start deceleration.
    // n1 = (n1+n2) decel / (accel + decel)
    uint32_t accel_lim;
    accel_lim = ((uint32_t) step * deceleration) / (acceleration + deceleration);
    // We must accelrate at least 1 step before we can start deceleration.
    if (accel_lim == 0) {
        accel_lim = 1;
    }


    // Use the limit we hit first to calc decel.
    if (accel_lim <= max_s_lim) {
        ramp.decel_initial_accel_count = (int16_t) (accel_lim - step);
    }
    else {
        ramp.decel_initial_accel_count = (int16_t) (-(max_s_lim * acceleration) / deceleration);
    }
    // We must decelerate at least 1 step to stop.
    if (ramp.decel_initial_accel_count == 0) {
        ramp.decel_initial_accel_count = -1;
    }
    usart_puts("decini:  ");
    printint(ramp.decel_initial_accel_count); usart_puts("\n");

    // Find step to start decleration.
    ramp.decel_start_position = step + ramp.decel_initial_accel_count;
    usart_puts("decfro:  ");
    printint(ramp.decel_start_position); usart_puts("\n");


    ramp.step_count = 0;
    ramp.status = ACCEL;
}

void motor_off()
{
    gpio_set_pin_low(&MOTOR0_PIN);
    gpio_set_pin_low(&MOTOR1_PIN);
    gpio_set_pin_low(&MOTOR2_PIN);
    gpio_set_pin_low(&MOTOR3_PIN);
}

bool motor_is_running()
{
    return ramp.status != STOP;
}

static void update_step_period(speed_ramp_t *ramp)
{
    uint16_t new_step_period;

    /* Calculations for the new step period are an approximation
     * for constant acceleration using a Taylor series, described in
     * "Generate stepper-motor speed profiles in real time"
     * by David Austin, Embedded Systems Programming, January 2005
     * https://www.embedded.com/4006438 and further explored in the
     * AVR app note "AVR446: Linear speed control of stepper motor"
     * with some code examples that are... "different from great".*/

    new_step_period = (uint16_t) (ramp->step_period -
            (((2 * ramp->step_period) + ramp->period_rest)
                    / (4 * ramp->accel_count + 1)));

    ramp->period_rest =
            (uint16_t) (((2 * ramp->step_period) + ramp->period_rest)
                    % (4 * ramp->accel_count + 1));

    ramp->step_period = new_step_period;
}

static void motor_thingy()
{
    switch (ramp.status) {
        case STOP:
            usart_puts("S ");
            ramp.step_count = 0;
            ramp.period_rest = 0;

            break;

        case ACCEL:
            usart_puts("A ");
            printint(ramp.step_period);usart_puts("\n");

            current_position++;
            ramp.step_count++;
            ramp.accel_count++;

            update_step_period(&ramp);

            if (ramp.step_count >= ramp.decel_start_position) {
                ramp.accel_count = ramp.decel_initial_accel_count;
                ramp.status = DECEL;
            }

            else if (ramp.step_period <= ramp.min_step_period) {
                ramp.last_period_during_accel = ramp.step_period;
                ramp.step_period = ramp.min_step_period;
                ramp.period_rest = 0;
                ramp.status = RUN;

                usart_puts("\nR ");
                printint(ramp.step_period);
            }

            break;

        case RUN:
            current_position++;
            ramp.step_count++;
            ramp.step_period = ramp.min_step_period;

            if (ramp.step_count >= ramp.decel_start_position) {
                ramp.accel_count = ramp.decel_initial_accel_count;
                ramp.step_period = ramp.last_period_during_accel;
                ramp.status = DECEL;
            }

            break;

        case DECEL:
            usart_puts("D ");
            printint(ramp.step_period);usart_puts("\n");

            current_position++;
            ramp.step_count++;
            ramp.accel_count++;

            update_step_period(&ramp);

            if (ramp.accel_count >= 0) {
                ramp.status = STOP;
            }

            break;
    }

    update_output();
}

static void timer_interrupt_handler()
{
    if (ramp.status != STOP) {
        motor_ticks++;

        if (motor_ticks >= ramp.step_period){
            motor_ticks = 0;
            motor_thingy();
        }
    }
}

static void update_output()
{
    uint8_t output;

    if (current_position >= 0) {
        output = STEP_OUTPUTS[current_position % 8];
    }
    else {
        output = STEP_OUTPUTS[-current_position % 8];
    }

    gpio_set_pin_state(&MOTOR0_PIN, (bool) (output & 0b0001));
    gpio_set_pin_state(&MOTOR1_PIN, (bool) (output & 0b0010));
    gpio_set_pin_state(&MOTOR2_PIN, (bool) (output & 0b0100));
    gpio_set_pin_state(&MOTOR3_PIN, (bool) (output & 0b1000));
}

static uint32_t sqrt(uint32_t x)
{
    uint32_t result;
    uint32_t scan_bit;
    bool flag;

    result = 0;
    scan_bit = 0x40000000L;           // higest possible result bit

    do {
        if ((result + scan_bit) <= x) {
            x -= result + scan_bit;
            flag = true;
        }
        else {
            flag = false;
        }

        result >>= 1;

        if (flag) {
            result += scan_bit;
        }
    }
    while (scan_bit >>= 2);

    if (result < x) {
        // add for rounding
        return result + 1;
    }

    return result;
}
