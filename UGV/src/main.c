#include "stm32h7xx_hal.h"
#include "init.h"
#include "imu.h"
#include "motor.h"
#include "motor_test.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    SystemClock_Config();
    Board_Init();

    if (MotorTest_Init(&htim3, TIM_CHANNEL_1) == MOTOR_OK)
    {
        MotorTest_RunSequence();
    }

    while (1)
    {
        /* Test finished. PWM remains at neutral. */
    }
}