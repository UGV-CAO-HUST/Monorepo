#include "stm32h7xx_hal.h"
#include "init.h"
#include "motor.h"

volatile uint32_t debug_stage = 0;

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
        debug_stage = 1;
        if (Motor_SetPulseUs(1550U) != MOTOR_OK)
        {
            Error_Handler();
        }
        HAL_Delay(5000U);

        debug_stage = 2;
        if (Motor_SetPulseUs(1400U) != MOTOR_OK)
        {
            Error_Handler();
        }
        HAL_Delay(5000U);

        debug_stage = 3;
        if (Motor_SetPulseUs(1400U) != MOTOR_OK)
        {
            Error_Handler();
        }
        HAL_Delay(5000U);

        debug_stage = 4;
        if (Motor_SetPulseUs(1500U) != MOTOR_OK)
        {
            Error_Handler();
        }
        HAL_Delay(5000U);
    }
}
