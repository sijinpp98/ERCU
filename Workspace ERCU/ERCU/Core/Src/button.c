#include "button.h"

volatile uint8_t emergency_stop_flag = 0;
static volatile uint32_t last_press_tick = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == BUTTON_PIN) {
        /* Simple debounce filter (200ms) */
        uint32_t now = HAL_GetTick();
        if (now - last_press_tick < 200) return;
        last_press_tick = now;

        emergency_stop_flag = !emergency_stop_flag;
    }
}
