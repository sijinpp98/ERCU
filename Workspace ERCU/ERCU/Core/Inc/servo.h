#ifndef SERVO_H
#define SERVO_H

#include "main.h"

extern TIM_HandleTypeDef htim3;

void Servo_SetAngle(uint16_t angle_deg);
void Servo_SweepStep(void);
uint16_t Servo_GetAngle(void);

#endif
