#ifndef IMU_H
#define IMU_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

// Structure to hold raw accelerometer and gyroscope data
typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;
} IMU_RawData;

// Structure to hold IMU identity information
typedef struct
{
    uint8_t accel_id;
    uint8_t gyro_id;
} IMU_Identity;

// Checks the status of the IMU operations
HAL_StatusTypeDef IMU_Init(IMU_Identity *identity);
HAL_StatusTypeDef IMU_ReadRaw(IMU_RawData *data);
HAL_StatusTypeDef IMU_ReadAccelID(uint8_t *id);
HAL_StatusTypeDef IMU_ReadGyroID(uint8_t *id);

#endif