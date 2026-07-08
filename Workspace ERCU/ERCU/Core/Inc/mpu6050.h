#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

extern I2C_HandleTypeDef hi2c1;
#define MPU6050_ADDR (0x68 << 1)

uint8_t MPU6050_Init(void);                 // returns 1 = OK, 0 = FAIL
uint8_t MPU6050_ReadTilt(float *tilt_deg);  // returns 1 = OK, 0 = FAIL

#endif
