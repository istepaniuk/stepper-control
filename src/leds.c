#include <stdbool.h>
#include <stm32f10x.h>
#include "leds.h"
#include "hardware.h"

void leds_init()
{
    gpio_set_pin_mode(&BLUE_LED_PIN, GPIO_MODE_OUT_PUSH_PULL);
    gpio_set_pin_mode(&GREEN_LED_PIN, GPIO_MODE_OUT_PUSH_PULL);

    gpio_set_pin_low(&BLUE_LED_PIN);
    gpio_set_pin_low(&GREEN_LED_PIN);
}

void leds_turn_on_green()
{
    gpio_set_pin_low(&GREEN_LED_PIN);
}

void leds_turn_off_green()
{
    gpio_set_pin_high(&GREEN_LED_PIN);
}

static bool is_blue_led_on = false;

void leds_toggle_green()
{
    is_blue_led_on = !is_blue_led_on;
    if (is_blue_led_on) {
        leds_turn_on_green();
    } else {
        leds_turn_off_green();
    }
}
