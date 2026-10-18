#ifndef INIT_H
#define INIT_H

#include "stm32h7xx_hal.h"

extern SPI_HandleTypeDef hspi1;

// Initialises the board, including clocks, GPIOs, and peripherals.
void Board_Init(void);

// Handles errors
void Error_Handler(void);

// Configures the system clock
void SystemClock_Config(void);

#endif