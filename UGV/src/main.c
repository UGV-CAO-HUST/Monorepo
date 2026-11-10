#include "stm32h7xx_hal.h"
#include "init.h"
#include "motor.h"

volatile uint32_t debug_stage = 0;

static void ramp_to(uint32_t target)
{
    uint32_t cur = Motor_GetPulseUs();
    while (cur != target)
    {
        if (cur > target) { cur = (cur - 10U < target) ? target : cur - 10U; }
        else              { cur = (cur + 10U > target) ? target : cur + 10U; }
        Motor_SetPulseUs(cur);
        HAL_Delay(50U);
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    Board_Init();

    if (Motor_Init(&htim3, MOTOR_PWM_TIM_CHANNEL) != MOTOR_OK)
    {
        Error_Handler();
    }

    while (1)
    {

        // once, at startup
        Motor_SetPulseUs(1500U); HAL_Delay(5000U);

        // forward
        ramp_to(1550U); HAL_Delay(3000U);

        // back to neutral, wait
        Motor_SetPulseUs(1500U); HAL_Delay(200U);

        // double-tap
        Motor_SetPulseUs(1000U); HAL_Delay(200U);
        Motor_SetPulseUs(1500U); HAL_Delay(200U);

        // reverse, ramped
        ramp_to(1420U); HAL_Delay(3000U);
        Motor_SetPulseUs(1500U);
    }
}
