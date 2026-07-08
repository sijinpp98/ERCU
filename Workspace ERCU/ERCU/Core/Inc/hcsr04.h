#ifndef HCSR04_H
#define HCSR04_H

#include "main.h"

extern TIM_HandleTypeDef htim2;

#define HCSR04_TRIG_PORT GPIOC
#define HCSR04_TRIG_PIN  GPIO_PIN_12

void HCSR04_Init(void);
void HCSR04_Trigger(void);
float HCSR04_GetDistance(void);
uint8_t HCSR04_IsHealthy(void);   // 0 = "sensor not responding" fault
void HCSR04_CheckTimeout(void);   // call periodically to detect a missing echo

#endif
