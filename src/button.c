#include <stm32f10x.h>
#include <stdbool.h>
#include "button.h"
#include "platform.h"
#include "hardware.h"

void button_init()
{
    gpio_set_pin_mode(&BUTTON_PIN, GPIO_MODE_IN_FLOATING);
}

bool button_is_pressed()
{
    return gpio_get_pin_state(&BUTTON_PIN);
}