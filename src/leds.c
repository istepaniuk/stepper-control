#include <stdbool.h>
#include <stm32f10x.h>
#include "leds.h"
#include "hardware.h"

void setup_leds()
{
    gpio_set_pin_mode(&BLUE_LED_PIN, GPIO_MODE_OUT_PUSH_PULL);
    gpio_set_pin_mode(&GREEN_LED_PIN, GPIO_MODE_OUT_PUSH_PULL);

    gpio_set_pin_low(&BLUE_LED_PIN);
    gpio_set_pin_low(&GREEN_LED_PIN);
}

void turn_off_green_led()
{
    gpio_set_pin_low(&GREEN_LED_PIN);
}

void turn_on_green_led()
{
    gpio_set_pin_high(&GREEN_LED_PIN);
}

static int is_blue_led_on = false;

void toggle_blue_led()
{
    is_blue_led_on = !is_blue_led_on;
    if (is_blue_led_on) {
        gpio_set_pin_high(&BLUE_LED_PIN);
    } else {
        gpio_set_pin_low(&BLUE_LED_PIN);
    }
}
