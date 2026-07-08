#ifndef BUTTON_H
#define BUTTON_H

#include "main.h"

#define BUTTON_PORT GPIOC
#define BUTTON_PIN  GPIO_PIN_4

// Set by the EXTI ISR, cleared once the main loop / Actuator task has handled it
extern volatile uint8_t emergency_stop_flag;

#endif
