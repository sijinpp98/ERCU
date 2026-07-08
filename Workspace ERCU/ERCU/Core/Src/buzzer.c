#include "buzzer.h"
#include "cmsis_os.h"

void Buzzer_On(void) {
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}
void Buzzer_Off(void) {
  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
}

void Buzzer_ShortBeep(void) {
  Buzzer_On();
  osDelay(150);
  Buzzer_Off();
}

static uint8_t warn_toggle = 0;
/* Non-blocking toggle for warning states */
void Buzzer_SlowIntermittent(void) {
  warn_toggle = !warn_toggle;
  if (warn_toggle)
    Buzzer_On();
  else
    Buzzer_Off();
}
