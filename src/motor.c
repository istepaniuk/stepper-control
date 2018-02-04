#include <stdbool.h>
#include <stm32f10x.h>
#include "motor.h"
#include "hardware.h"
#include "delay.h"
#include "timer.h"
#include "interrupts.h"

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
    int decel_start_position;
    int16_t accel_count;
    int step_count;
    int16_t decel_initial_accel_count;
    uint16_t last_period_during_accel;
} speed_ramp_t;

speed_ramp_t ramp;

static int current_position = 0;

static void update_step_period(speed_ramp_t *ramp);
static void timer_interrupt_handler();
static void update_output();


void motor_init()
{
    current_position = 0;

    gpio_set_pin_mode(&MOTOR0_PIN, GPIO_MODE_OUT_PUSH_PULL);
    gpio_set_pin_mode(&MOTOR1_PIN, GPIO_MODE_OUT_PUSH_PULL);
    gpio_set_pin_mode(&MOTOR2_PIN, GPIO_MODE_OUT_PUSH_PULL);
    gpio_set_pin_mode(&MOTOR3_PIN, GPIO_MODE_OUT_PUSH_PULL);

    motor_off();
}

void motor_goto()
{
    //TODO: calculate these values from new_position, speed, accel & decel.
    ramp.status = ACCEL;
    ramp.min_step_period = 800;
    ramp.step_period = 20000;
    ramp.period_rest = 0;
    ramp.accel_count = 0;
    ramp.decel_initial_accel_count = -50;
    ramp.decel_start_position = 200;
    ramp.step_count = 0;

    //F = CLK/((PSC + 1)*(ARR + 1))
    //24MHz CLK/(1000-1)*(24-1) = 1 KHz ISR
    timer2_init(ramp.step_period - (int16_t) 1, 240 - 1);
    timer2_start();
    interrupt_set_timer2_callback(timer_interrupt_handler);
}

void motor_off()
{
    gpio_set_pin_low(&MOTOR0_PIN);
    gpio_set_pin_low(&MOTOR1_PIN);
    gpio_set_pin_low(&MOTOR2_PIN);
    gpio_set_pin_low(&MOTOR3_PIN);
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

static void timer_interrupt_handler()
{
    switch (ramp.status) {
        case STOP:
            ramp.step_count = 0;
            ramp.period_rest = 0;
            timer2_stop();

            break;

        case ACCEL:
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
            current_position++;
            ramp.step_count++;
            ramp.accel_count++;

            update_step_period(&ramp);

            if (ramp.accel_count >= 0) {
                ramp.status = STOP;
            }

            break;
    }

    timer2_set_period(ramp.step_period - (int16_t) 1);
    update_output();
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
