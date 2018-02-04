#include "delay.h"
#include <stm32f10x_tim.h>


static uint32_t delay_counter;


/* called by startup_stm32f10x_md_vl.S */
void SysTick_Handler(void) {
    delay_counter++;
}

void delay_init()
{
    // Set reload register to generate IRQ every millisecond
    SysTick->LOAD = (uint32_t)(SystemCoreClock / (1000UL - 1UL));

    // Set priority for SysTick IRQ
    NVIC_SetPriority(SysTick_IRQn,(1 << __NVIC_PRIO_BITS) - 1);

    // Set the SysTick counter value
    SysTick->VAL = 0UL;

    // Set SysTick source and IRQ
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk);
}

void delay_ms(uint32_t milliseconds) {
    // Enable the SysTick timer
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    // Wait for a specified number of milliseconds
    delay_counter = 0;
    while (delay_counter < milliseconds) {};

    // Disable the SysTick timer
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}