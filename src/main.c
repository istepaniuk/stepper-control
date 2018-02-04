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

    while(true){
        motor_goto();
        delay_ms(6000);
    };
}
