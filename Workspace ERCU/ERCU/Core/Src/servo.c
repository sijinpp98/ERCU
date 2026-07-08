#include "servo.h"

static int16_t angle = 0;
static int8_t dir = 1;

void Servo_SetAngle(uint16_t angle_deg) {
    if (angle_deg > 180) angle_deg = 180;
    /* SG90: 500us (0 deg) to 2500us (180 deg), timer tick = 1us */
    uint16_t pulse = 500 + ((uint32_t)angle_deg * 2000 / 180);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
    angle = angle_deg;
}

void Servo_SweepStep(void) {
    angle += dir * 5;
    if (angle >= 180) { angle = 180; dir = -1; }
    if (angle <= 0)   { angle = 0;   dir = 1;  }
    Servo_SetAngle(angle);
}

uint16_t Servo_GetAngle(void) { return angle; }
