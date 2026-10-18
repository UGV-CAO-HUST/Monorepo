#include "stm32h7xx_hal.h"
#include "init.h"
#include "imu.h"

volatile IMU_RawData imu_data;
volatile uint8_t imu_ready = 0U;
volatile uint8_t imu_error = 0U;

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    Board_Init();

    IMU_Identity identity;

    if (IMU_Init(&identity) != HAL_OK)
    {
        Error_Handler();
    }

    while (1)
    {
        if (IMU_ReadRaw((IMU_RawData *)&imu_data) == HAL_OK)
        {
            imu_ready = 1U;
        }
        else
        {
            imu_error = 1U;
        }

        HAL_Delay(10U);
    }
}