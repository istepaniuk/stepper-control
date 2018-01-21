#include "hardware.h"
#include "leds.h"
#include "delay.h"
#include "button.h"
#include "usart.h"
#include "motor.h"

int main(void)
{
    setup_gpio();
    setup_button();
    setup_leds();
    motor_setup();
    usart_init();

    int x = 0;
    while (1) {
        x += 96*8;
        motor_goto(x);
        motor_goto(x - 96);
        delay_ms(1500);
    }
}
