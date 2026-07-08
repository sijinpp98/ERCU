#ifndef BUZZER_H
#define BUZZER_H

#include "main.h"

#define BUZZER_PORT GPIOB
#define BUZZER_PIN  GPIO_PIN_6

void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_ShortBeep(void);      // single short beep - button press
void Buzzer_SlowIntermittent(void); // call periodically in WARNING state (overheat)

#endif
