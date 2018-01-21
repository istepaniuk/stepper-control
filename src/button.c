#include <stm32f10x.h>
#include <stdbool.h>
#include "button.h"
#include "platform.h"
#include "hardware.h"

void setup_button()
{
    gpio_set_pin_mode(&BUTTON_PIN, GPIO_MODE_IN_FLOATING);
}

bool is_button_pressed()
{
    return gpio_get_pin_state(&BUTTON_PIN);
}