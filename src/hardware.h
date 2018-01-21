#ifndef CATFEEDER_HARDWARE_H
#define CATFEEDER_HARDWARE_H

#include <stm32f10x.h>
#include "platform.h"


#define BLUE_LED_PIN GPIO_PIN_C13
#define GREEN_LED_PIN GPIO_PIN_C9

#define USART_TX_PIN GPIO_PIN_A9
#define USART_RX_PIN GPIO_PIN_A10

#define BUTTON_PIN GPIO_PIN_A0

#define MOTOR0_PIN GPIO_PIN_B12
#define MOTOR1_PIN GPIO_PIN_B13
#define MOTOR2_PIN GPIO_PIN_B14
#define MOTOR3_PIN GPIO_PIN_B15


void setup_gpio();

#endif
