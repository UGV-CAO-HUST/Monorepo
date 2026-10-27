#include "init.h"
#include "motor.h"

SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim3;

static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM3_GPIO_Init(void);
static uint32_t Get_APB1TimerClockHz(void);

void Board_Init(void)
{
    MX_SPI1_Init();
    MX_TIM3_Init();
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;

    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
 * TIM3 is on APB1. When APB1 is divided, the timer kernel clock
 * is a multiple of PCLK1 (see RCC TIMPRE).
 */
static uint32_t Get_APB1TimerClockHz(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t d2ppre1 = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1);

    if (d2ppre1 == RCC_D2CFGR_D2PPRE1_DIV1)
    {
        return pclk1;
    }

    if ((RCC->CFGR & RCC_CFGR_TIMPRE) == 0U)
    {
        return (pclk1 * 2U);
    }

    if ((d2ppre1 == RCC_D2CFGR_D2PPRE1_DIV8) ||
        (d2ppre1 == RCC_D2CFGR_D2PPRE1_DIV16))
    {
        return (pclk1 * 4U);
    }

    return HAL_RCC_GetHCLKFreq();
}

static void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    uint32_t timclk_hz;
    uint32_t prescaler;

    __HAL_RCC_TIM3_CLK_ENABLE();

    timclk_hz = Get_APB1TimerClockHz();

    /*
     * Motor PWM counts microseconds: tick must be exactly 1 MHz.
     * With SYSCLK=480 MHz, AHB/2, APB1/2 this is 240 MHz / 240.
     */
    if ((timclk_hz < MOTOR_PWM_TICK_HZ) ||
        ((timclk_hz % MOTOR_PWM_TICK_HZ) != 0U))
    {
        Error_Handler();
    }

    prescaler = (timclk_hz / MOTOR_PWM_TICK_HZ) - 1U;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler         = prescaler;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = MOTOR_PWM_PERIOD_US - 1U;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = MOTOR_NEUTRAL_PULSE_US;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(
            &htim3,
            &sConfigOC,
            MOTOR_PWM_TIM_CHANNEL) != HAL_OK)
    {
        Error_Handler();
    }

    MX_TIM3_GPIO_Init();
}

static void MX_TIM3_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin       = MOTOR_PWM_GPIO_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = MOTOR_PWM_GPIO_AF;

    HAL_GPIO_Init(MOTOR_PWM_GPIO_PORT, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    /*
     * Do not silently continue after a peripheral
     * initialization failure.
     */
    __disable_irq();

    while (1)
    {
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }

    /*
     * HSI 64 MHz / 4 = 16 MHz PLL input.
     * PLLN=60, PLLP=2 -> SYSCLK = 480 MHz.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 60;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK |
                                   RCC_CLOCKTYPE_HCLK |
                                   RCC_CLOCKTYPE_PCLK1 |
                                   RCC_CLOCKTYPE_PCLK2 |
                                   RCC_CLOCKTYPE_D1PCLK1 |
                                   RCC_CLOCKTYPE_D3PCLK1);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}
