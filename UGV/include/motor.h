#ifndef MOTOR_H
#define MOTOR_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* ESC pulse-width safety limits, in microseconds */
#define MOTOR_MIN_PULSE_US  1000U
#define MOTOR_MAX_PULSE_US  2000U

typedef enum
{
    MOTOR_OK = 0,
    MOTOR_ERROR_INVALID_ARGUMENT,
    MOTOR_ERROR_HAL
} Motor_Status;

/*
 * Initialize the motor PWM output.
 *
 * htim:    Pointer to the configured STM32 timer handle.
 * channel: Timer channel, e.g. TIM_CHANNEL_1.
 *
 * REQUIREMENT:
 * Configure the timer counter to tick at exactly 1 MHz
 * (one counter tick per microsecond).
 *
 * Starts PWM with a 1500 us pulse.
 */
Motor_Status Motor_Init(TIM_HandleTypeDef *htim, uint32_t channel);

/*
 * Set the PWM period in microseconds.
 *
 * Example:
 *     Motor_SetPeriodUs(20000);  // 50 Hz
 *     Motor_SetPeriodUs(10000);  // 100 Hz
 *
 * Returns MOTOR_ERROR_INVALID_ARGUMENT if the requested
 * period cannot accommodate the current pulse width.
 */
Motor_Status Motor_SetPeriodUs(uint32_t period_us);

/*
 * Set the PWM pulse width in microseconds.
 *
 * Accepts only 1000-2000 us, inclusive.
 * Invalid values are rejected; the previous output command
 * is retained.
 */
Motor_Status Motor_SetPulseUs(uint32_t pulse_us);

/* Read the currently configured pulse width and period. */
uint32_t Motor_GetPulseUs(void);
uint32_t Motor_GetPeriodUs(void);

#endif /* MOTOR_H */