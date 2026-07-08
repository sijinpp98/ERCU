#ifndef DHT11_H
#define DHT11_H

#include "main.h"

#define DHT11_PORT GPIOC
#define DHT11_PIN  GPIO_PIN_8

typedef struct {
    float temperature;
    float humidity;
    uint8_t valid;   // 1 = last read OK, 0 = checksum/timeout fail
} DHT11_Data;

void DHT11_Init(void);
uint8_t DHT11_Read(DHT11_Data *out); // returns 1 = OK, 0 = FAIL

#endif
