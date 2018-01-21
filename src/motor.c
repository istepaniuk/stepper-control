#include <stdbool.h>
#include <stm32f10x.h>
#include "motor.h"
#include "hardware.h"
#include "delay.h"

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

static int motor_pos = 0;

void motor_setup()
{
    gpio_set_pin_mode(&MOTOR0_PIN, GPIO_MODE_OUT_PUSH_PULL);
    gpio_set_pin_mode(&MOTOR1_PIN, GPIO_MODE_OUT_PUSH_PULL);
    gpio_set_pin_mode(&MOTOR2_PIN, GPIO_MODE_OUT_PUSH_PULL);
    gpio_set_pin_mode(&MOTOR3_PIN, GPIO_MODE_OUT_PUSH_PULL);

    gpio_set_pin_low(&MOTOR0_PIN);
    gpio_set_pin_low(&MOTOR1_PIN);
    gpio_set_pin_low(&MOTOR2_PIN);
    gpio_set_pin_low(&MOTOR3_PIN);
}

void motor_off(){
    gpio_set_pin_low(&MOTOR0_PIN);
    gpio_set_pin_low(&MOTOR1_PIN);
    gpio_set_pin_low(&MOTOR2_PIN);
    gpio_set_pin_low(&MOTOR3_PIN);
}

static void update_output()
{
    uint8_t output = steps[motor_pos % 8];
    gpio_set_pin_state(&MOTOR0_PIN, (bool) (output & 0b0001));
    gpio_set_pin_state(&MOTOR1_PIN, (bool) (output & 0b0010));
    gpio_set_pin_state(&MOTOR2_PIN, (bool) (output & 0b0100));
    gpio_set_pin_state(&MOTOR3_PIN, (bool) (output & 0b1000));
}

static int difference(int a, int b)
{
    return a > b ? a - b : b - a;
}

 enum { STOP, ACCEL, RUN, DECEL } status = STOP;

// Direction stepper motor should move.
unsigned char dir;
// Peroid of next timer delay. At start this value set the accelration rate.
unsigned int step_delay;
// What step_pos to start decelaration
unsigned int decel_start;
// Sets deceleration rate.
signed int decel_val;
// Minimum time delay (max speed)
signed int min_delay;
//! Counter used when accelerateing/decelerateing to calculate step_delay.
signed int accel_count;


void motor_int(int new_position)
{
    switch (status){
        case STOP:
            break;
        case ACCEL:
            break;
        case RUN:
            break;
        case DECEL:
            break;
    }

    int speed = 200;
    int LEAD = 200;
    int TAIL = 200;
    int MAX_SPEED = 1000;
    int moved = 0;
    int deceleration_start = difference(new_position, motor_pos) - TAIL;

    while (difference(new_position, motor_pos) > 0) {
        moved++;

        if (moved < LEAD && moved < deceleration_start) {
            speed += 3;
        }
        else if (moved > LEAD && moved < deceleration_start) {
            //
        }
        else if (moved > deceleration_start) {
            speed -= 3;
        }

        if (speed > MAX_SPEED) {
            speed = MAX_SPEED;
        }

        if (motor_pos < new_position) {
            motor_pos++;
        }
        else {
            motor_pos--;
        }

        update_output();
        delay_us(4000 + 30 * (MAX_SPEED - speed));
    }
}

static unsigned long sqrt(unsigned long x)
{
    register unsigned long result;  // result register
    register unsigned long q2;  // scan-bit register
    register unsigned char flag;   // flag (one bit)

    result = 0;                     // clear result
    q2 = 0x40000000L;           // higest possible result bit
    do {
        if ((result + q2) <= x) {
            x -= result + q2;
            flag = 1;                  // set flag
        }
        else {
            flag = 0;                  // clear flag
        }
        result >>= 1;
        if (flag) {
            result += q2;               // test flag
        }
    }
    while (q2 >>= 2);                // shift twice
    if (result < x) {
        return result + 1;             // add for rounding
    }
    else {
        return result;
    }
}

static unsigned int min(unsigned int x, unsigned int y)
{
    if (x < y) {
        return x;
    }
    else {
        return y;
    }
}