#include <stdbool.h>
#include <stm32f10x.h>
#include <stdlib.h>
#include "motor.h"
#include "hardware.h"
#include "delay.h"
#include "timer.h"
#include "interrupts.h"

static const uint8_t steps[] = {
        0b0001,
        0b0011,
        0b0010,
        0b0110,
        0b0100,
        0b1100,
        0b1000,
        0b1001
};

static enum { STOP, ACCEL, RUN, DECEL } status = STOP;
static int current_position = 0;
static int end_position = 0;
static int start_position = 0;
static uint16_t speed = 1000;
void advance_position();


void motor_init()
{
    current_position = 0;

    gpio_set_pin_mode(&MOTOR0_PIN, GPIO_MODE_OUT_PUSH_PULL);
    gpio_set_pin_mode(&MOTOR1_PIN, GPIO_MODE_OUT_PUSH_PULL);
    gpio_set_pin_mode(&MOTOR2_PIN, GPIO_MODE_OUT_PUSH_PULL);
    gpio_set_pin_mode(&MOTOR3_PIN, GPIO_MODE_OUT_PUSH_PULL);

    motor_off();

    //F = CLK/((PSC + 1)*(ARR + 1))
    //24MHz CLK/(1000-1)*(24-1) = 1 KHz ISR
    timer2_init((uint16_t) (10000 - speed - 1), 240 - 1);
    timer2_start();

    interrupt_set_timer2_callback(timer_interrupt_handler);
}

int difference(int a, int b)
{
    return a > b ? a - b : b - a;
}

static void timer_interrupt_handler()
{
    int went = difference(current_position, start_position);
    int to_go = difference(current_position, end_position);
    int START_SLOPE_SIZE = 150;
    int END_SLOPE_SIZE = 150;
    
    switch (status) {
        case STOP:
            break;
        case ACCEL:
            speed += 8;
            advance_position();
            if(went > START_SLOPE_SIZE){
                status = RUN;
            }
            break;
        case DECEL:
            speed -= 8;
            advance_position();
            break;
        case RUN:
            advance_position();
            if(to_go < END_SLOPE_SIZE){
                status = DECEL;
            }
            break;
    }

    update_output();
    timer2_set_period((uint16_t) (10000 - speed - 1));
}

void advance_position()
{
    if (current_position < end_position) {
        current_position++;
    }
    else if (current_position > end_position) {
        current_position--;
    }
    else {
        status = STOP;
    }
}

void motor_goto(int new_position)
{
    speed = 7000;
    if (new_position == current_position) {
        return;
    }

    start_position = current_position;
    end_position = new_position;
    status = ACCEL;
}

void motor_off()
{
    gpio_set_pin_low(&MOTOR0_PIN);
    gpio_set_pin_low(&MOTOR1_PIN);
    gpio_set_pin_low(&MOTOR2_PIN);
    gpio_set_pin_low(&MOTOR3_PIN);
}

static void update_output()
{
    uint8_t output;

    if (current_position >= 0) {
        output = steps[current_position % 8];
    }
    else {
        output = steps[-current_position % 8];
    }

    gpio_set_pin_state(&MOTOR0_PIN, (bool) (output & 0b0001));
    gpio_set_pin_state(&MOTOR1_PIN, (bool) (output & 0b0010));
    gpio_set_pin_state(&MOTOR2_PIN, (bool) (output & 0b0100));
    gpio_set_pin_state(&MOTOR3_PIN, (bool) (output & 0b1000));
}

static unsigned int sqrt(unsigned int x)
{
    register unsigned int result;
    register unsigned int q2scan_bit;
    register unsigned char flag;

    result = 0;
    q2scan_bit = HIGHEST_SQRT_RESULT_BIT;

    do {
        if ((result + q2scan_bit) <= x) {
            x -= result + q2scan_bit;
            flag = 1;
        }
        else {
            flag = 0;
        }

        result >>= 1;

        if (flag) {
            result += q2scan_bit;
        }

    }
    while (q2scan_bit >>= 2);

    if (result < x) {
        return result + 1;
    }

    return result;
}

static int min(int x, int y)
{
    if (x < y) {
        return x;
    }
    else {
        return y;
    }
}