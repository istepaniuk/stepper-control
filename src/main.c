#include "hardware.h"
#include "leds.h"
#include "delay.h"
#include "button.h"
#include "usart.h"
#include "motor.h"

int main(void)
{
    gpio_init();
    button_init();
    leds_init();
    usart_init();
    motor_init();

    int x = 0;

    while (1) {
        motor_goto(x);
        x += 500;
        delay_ms(10000);
    }
}
