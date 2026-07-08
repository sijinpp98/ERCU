#include "hcsr04.h"

static volatile uint32_t ic_val1 = 0, ic_val2 = 0;
static volatile uint8_t capture_state = 0;
static volatile float distance_cm = 999.0f;
static volatile uint8_t healthy = 0;
static volatile uint32_t last_trigger_tick = 0;
static volatile uint8_t echo_received = 0;

void HCSR04_Init(void) {
    /* Reconfigure TIM2 to 1 MHz (1 tick = 1 us) to match distance calculation */
    htim2.Instance->PSC = (SystemCoreClock / 1000000) - 1; 
    htim2.Instance->ARR = 0xFFFFFFFF;
    htim2.Instance->EGR = TIM_EGR_UG;
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
}

static void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

void HCSR04_Trigger(void) {
    echo_received = 0;
    last_trigger_tick = HAL_GetTick();
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        if (capture_state == 0) {
            ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
            capture_state = 1;
        } else {
            ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);

            /* Calculate pulse width in microseconds */
            uint32_t pulse_width = ic_val2 - ic_val1;
            
            distance_cm = (pulse_width * 0.0343f) / 2.0f;
            healthy = 1;
            echo_received = 1;
            capture_state = 0;
        }
    }
}

/* Detect missing echoes (e.g. sensor disconnected or out of range) */
void HCSR04_CheckTimeout(void) {
    if (!echo_received && (HAL_GetTick() - last_trigger_tick > 30)) {
        healthy = 0;
    }
}

float HCSR04_GetDistance(void) { return distance_cm; }
uint8_t HCSR04_IsHealthy(void) { return healthy; }
