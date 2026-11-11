#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "stm32h7xx_hal.h"

/*
 * SPI1 (BMI088) uses PA5/PA6/PA7, so motor PWM cannot use TIM3_CH1 on PA6.
 * TIM3_CH1 AF2 on PC6 is free.
 */
#define MOTOR_PWM_GPIO_PORT      GPIOC
#define MOTOR_PWM_GPIO_PIN       GPIO_PIN_6
#define MOTOR_PWM_GPIO_AF        GPIO_AF2_TIM3
#define MOTOR_PWM_TIM_CHANNEL    TIM_CHANNEL_1

/* Motor PWM counter tick: 1 MHz so ARR/CCR values are microseconds. */
#define MOTOR_PWM_TICK_HZ        1000000UL

#endif
