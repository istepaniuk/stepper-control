#include "hardware.h"
#include "leds.h"
#include "delay.h"
#include "button.h"
#include "motor.h"
#include "usart.h"

int main(void)
{
    gpio_init();
    button_init();
    usart_init();
    leds_init();
    delay_init();
    motor_init();

    while(true){
        leds_turn_on_green();

        //acc/dec in 0.01 * rad/sec^2
        //speed   in 0.01 * rad/sec
        motor_goto(96 * 6, 500, 70, 70);

        while(motor_is_running()) {};

        leds_turn_off_green();
        delay_ms(1500);
    };
}
