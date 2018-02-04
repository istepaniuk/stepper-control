#include "hardware.h"
#include "leds.h"
#include "delay.h"
#include "button.h"
#include "motor.h"

int main(void)
{
    gpio_init();
    button_init();
    leds_init();
    delay_init();
    motor_init();

    while(true){
        leds_turn_on_green();
        motor_goto(1000, 1000, 10, 10);
        while(motor_is_running()) {};
        leds_turn_off_green();
        delay_ms(1500);
    };
}
