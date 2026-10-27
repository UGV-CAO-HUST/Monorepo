#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include "motor.h"

/* ESC test configuration */
#define MOTOR_TEST_PERIOD_US       20000U  /* 50 Hz */
#define MOTOR_TEST_NEUTRAL_US      1500U
#define MOTOR_TEST_FORWARD_US      1400U
#define MOTOR_TEST_REVERSE_US      1600U

#define MOTOR_TEST_ARM_DELAY_MS    3000U
#define MOTOR_TEST_RUN_DELAY_MS    2000U
#define MOTOR_TEST_NEUTRAL_DELAY_MS 2000U

/*
 * Initialize the motor driver for testing.
 *
 * htim:     Pointer to the configured STM32 timer handle.
 * channel:  PWM channel, e.g. TIM_CHANNEL_1.
 */
Motor_Status MotorTest_Init(TIM_HandleTypeDef *htim, uint32_t channel);

/*
 * Run a basic ESC test sequence:
 *   1. Neutral while the ESC arms.
 *   2. Forward command.
 *   3. Neutral.
 *   4. Reverse command.
 *   5. Neutral.
 *
 * This function blocks while the sequence runs.
 */
Motor_Status MotorTest_RunSequence(void);

#endif /* MOTOR_TEST_H */