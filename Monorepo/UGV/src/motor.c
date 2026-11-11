#include "motor.h"

// Internal state 
static TIM_HandleTypeDef *motor_htim = NULL;
static uint32_t motor_channel = 0U;

static uint32_t motor_pulse_us = MOTOR_NEUTRAL_PULSE_US;
static uint32_t motor_period_us = MOTOR_PWM_PERIOD_US;

/*
 * Initialize the PWM output.
 */
Motor_Status Motor_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (htim == NULL)
    {
        return MOTOR_ERROR_INVALID_ARGUMENT;
    }

    motor_htim = htim;
    motor_channel = channel;

    motor_pulse_us = MOTOR_NEUTRAL_PULSE_US;
    motor_period_us = MOTOR_PWM_PERIOD_US;

    /*
     * Set the period:
     * ARR = period - 1 because the timer counts from 0 to ARR.
     */
    __HAL_TIM_SET_AUTORELOAD(motor_htim, motor_period_us - 1U);

    /* Set the initial pulse width. */
    __HAL_TIM_SET_COMPARE(
        motor_htim,
        motor_channel,
        motor_pulse_us
    );

    /* Start PWM generation. */
    if (HAL_TIM_PWM_Start(motor_htim, motor_channel) != HAL_OK)
    {
        motor_htim = NULL;
        return MOTOR_ERROR_HAL;
    }

    return MOTOR_OK;
}


/*
 * Change the PWM period.
 */
Motor_Status Motor_SetPeriodUs(uint32_t period_us)
{
    if (motor_htim == NULL)
    {
        return MOTOR_ERROR_INVALID_ARGUMENT;
    }

    /*
     * Reject zero-length periods and periods that cannot
     * accommodate the current pulse.
     */
    if ((period_us == 0U) || (period_us <= motor_pulse_us))
    {
        return MOTOR_ERROR_INVALID_ARGUMENT;
    }

    /*
     * This implementation assumes a 16-bit timer.
     * ARR must fit in 0..65535.
     *
     * At a 1 MHz counter rate, the maximum period is 65536 us.
     */
    if (period_us > 65536U)
    {
        return MOTOR_ERROR_INVALID_ARGUMENT;
    }

    motor_period_us = period_us;

    __HAL_TIM_SET_AUTORELOAD(motor_htim, period_us - 1U);

    return MOTOR_OK;
}


/*
 * Change the PWM pulse width.
 */
Motor_Status Motor_SetPulseUs(uint32_t pulse_us)
{
    if (motor_htim == NULL)
    {
        return MOTOR_ERROR_INVALID_ARGUMENT;
    }

    /*
     * Safety check:
     * Reject anything outside the permitted ESC command range.
     */
    if ((pulse_us < MOTOR_MIN_PULSE_US) ||
        (pulse_us > MOTOR_MAX_PULSE_US))
    {
        return MOTOR_ERROR_INVALID_ARGUMENT;
    }

    /*
     * The pulse must fit inside the PWM period.
     */
    if (pulse_us >= motor_period_us)
    {
        return MOTOR_ERROR_INVALID_ARGUMENT;
    }

    motor_pulse_us = pulse_us;

    __HAL_TIM_SET_COMPARE(
        motor_htim,
        motor_channel,
        pulse_us
    );

    return MOTOR_OK;
}


/*
 * Return the current pulse width.
 */
uint32_t Motor_GetPulseUs(void)
{
    return motor_pulse_us;
}


/*
 * Return the current period.
 */
uint32_t Motor_GetPeriodUs(void)
{
    return motor_period_us;
}