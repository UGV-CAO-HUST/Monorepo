#include "motor_test.h"
#include <stdint.h>

/* Internal state */
static uint8_t motor_test_initialized = 0U;

Motor_Status MotorTest_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
    Motor_Status status;

    motor_test_initialized = 0U;

    /* Initialize the low-level motor driver and start PWM. */
    status = Motor_Init(htim, channel);
    if (status != MOTOR_OK)
    {
        return status;
    }

    /* Configure the ESC command signal period. */
    status = Motor_SetPeriodUs(MOTOR_TEST_PERIOD_US);
    if (status != MOTOR_OK)
    {
        return status;
    }

    /* Start at neutral. */
    status = Motor_SetPulseUs(MOTOR_TEST_NEUTRAL_US);
    if (status != MOTOR_OK)
    {
        return status;
    }

    motor_test_initialized = 1U;

    return MOTOR_OK;
}

Motor_Status MotorTest_RunSequence(void)
{
    Motor_Status status;

    if (motor_test_initialized == 0U)
    {
        return MOTOR_ERROR_INVALID_ARGUMENT;
    }

    /* 1. Keep neutral while the ESC arms. */
    status = Motor_SetPulseUs(MOTOR_TEST_NEUTRAL_US);
    if (status != MOTOR_OK)
    {
        return status;
    }

    HAL_Delay(MOTOR_TEST_ARM_DELAY_MS);

    /* 2. Forward command. */
    status = Motor_SetPulseUs(MOTOR_TEST_FORWARD_US);
    if (status != MOTOR_OK)
    {
        return status;
    }

    HAL_Delay(MOTOR_TEST_RUN_DELAY_MS);

    /* 3. Return to neutral. */
    status = Motor_SetPulseUs(MOTOR_TEST_NEUTRAL_US);
    if (status != MOTOR_OK)
    {
        return status;
    }

    HAL_Delay(MOTOR_TEST_NEUTRAL_DELAY_MS);

    /* 4. Reverse command. */
    status = Motor_SetPulseUs(MOTOR_TEST_REVERSE_US);
    if (status != MOTOR_OK)
    {
        return status;
    }

    HAL_Delay(MOTOR_TEST_RUN_DELAY_MS);

    /* 5. Finish at neutral. */
    status = Motor_SetPulseUs(MOTOR_TEST_NEUTRAL_US);
    if (status != MOTOR_OK)
    {
        return status;
    }

    return MOTOR_OK;
}

Motor_Status MotorTest_SetCommand(uint32_t pulse_us)
{
    if (motor_test_initialized == 0U)
    {
        return MOTOR_ERROR_INVALID_ARGUMENT;
    }

    return Motor_SetPulseUs(pulse_us);
}