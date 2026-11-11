#include "imu.h"
#include "init.h"

/* BMI088 chip-select pins */
#define ACC_CS_PORT GPIOB
#define ACC_CS_PIN  GPIO_PIN_0

#define GYRO_CS_PORT GPIOB
#define GYRO_CS_PIN  GPIO_PIN_1

/* Accelerometer registers */
#define ACC_CHIP_ID       0x00U
#define ACC_DATA_START    0x12U
#define ACC_PWR_CONF      0x7CU
#define ACC_PWR_CTRL      0x7DU
#define ACC_SOFTRESET     0x7EU

/* Gyroscope registers */
#define GYRO_CHIP_ID      0x00U
#define GYRO_DATA_START   0x02U
#define GYRO_SOFTRESET    0x14U

#define ACC_EXPECTED_ID   0x1EU
#define GYRO_EXPECTED_ID  0x0FU

#define SPI_TIMEOUT_MS    100U

static void IMU_SelectAccel(void)
{
    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_RESET);
}

static void IMU_DeselectAccel(void)
{
    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_SET);
}

static void IMU_SelectGyro(void)
{
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
}

static void IMU_DeselectGyro(void)
{
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

/* Write one accelerometer register. */
static HAL_StatusTypeDef Accel_WriteReg(
    uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7FU), value };

    IMU_SelectAccel();

    HAL_StatusTypeDef status = HAL_SPI_Transmit(
        &hspi1, tx, sizeof(tx), SPI_TIMEOUT_MS);

    IMU_DeselectAccel();

    return status;
}

/* Write one gyroscope register. */
static HAL_StatusTypeDef Gyro_WriteReg(
    uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7FU), value };

    IMU_SelectGyro();

    HAL_StatusTypeDef status = HAL_SPI_Transmit(
        &hspi1, tx, sizeof(tx), SPI_TIMEOUT_MS);

    IMU_DeselectGyro();

    return status;
}

/*
 * BMI088 accelerometer SPI read:
 * address byte + dummy byte + returned data.
 */
static HAL_StatusTypeDef Accel_ReadRegs(
    uint8_t reg, uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U)
        return HAL_ERROR;

    uint8_t tx[8] = {0};
    uint8_t rx[8] = {0};

    if (length > 6U)
        return HAL_ERROR;

    tx[0] = (uint8_t)(reg | 0x80U);

    IMU_SelectAccel();

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        &hspi1, tx, rx, (uint16_t)(length + 2U),
        SPI_TIMEOUT_MS);

    IMU_DeselectAccel();

    if (status == HAL_OK)
    {
        for (uint16_t i = 0; i < length; i++)
            data[i] = rx[i + 2U];
    }

    return status;
}

/*
 * BMI088 gyroscope SPI read:
 * address byte + returned data.
 */
static HAL_StatusTypeDef Gyro_ReadRegs(
    uint8_t reg, uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U)
        return HAL_ERROR;

    uint8_t tx[7] = {0};
    uint8_t rx[7] = {0};

    if (length > 6U)
        return HAL_ERROR;

    tx[0] = (uint8_t)(reg | 0x80U);

    IMU_SelectGyro();

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        &hspi1, tx, rx, (uint16_t)(length + 1U),
        SPI_TIMEOUT_MS);

    IMU_DeselectGyro();

    if (status == HAL_OK)
    {
        for (uint16_t i = 0; i < length; i++)
            data[i] = rx[i + 1U];
    }

    return status;
}

HAL_StatusTypeDef IMU_ReadAccelID(uint8_t *id)
{
    return Accel_ReadRegs(ACC_CHIP_ID, id, 1U);
}

HAL_StatusTypeDef IMU_ReadGyroID(uint8_t *id)
{
    return Gyro_ReadRegs(GYRO_CHIP_ID, id, 1U);
}

HAL_StatusTypeDef IMU_Init(IMU_Identity *identity)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint8_t accel_id = 0U;
    uint8_t gyro_id = 0U;

    /* Configure the two chip-select pins. */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = ACC_CS_PIN | GYRO_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Keep both devices unselected while idle. */
    HAL_GPIO_WritePin(GPIOB,
        ACC_CS_PIN | GYRO_CS_PIN, GPIO_PIN_SET);

    HAL_Delay(1U);

    /* Verify communication before issuing reset commands. */
    if (IMU_ReadAccelID(&accel_id) != HAL_OK)
        return HAL_ERROR;

    if (IMU_ReadGyroID(&gyro_id) != HAL_OK)
        return HAL_ERROR;

    if (accel_id != ACC_EXPECTED_ID ||
        gyro_id != GYRO_EXPECTED_ID)
        return HAL_ERROR;

    /* Reset the accelerometer and gyroscope independently. */
    if (Accel_WriteReg(ACC_SOFTRESET, 0xB6U) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(50U);

    if (Gyro_WriteReg(GYRO_SOFTRESET, 0xB6U) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(50U);

    /*
     * Put the accelerometer into active power mode.
     * ACC_PWR_CONF = 0x00: suspend mode disabled.
     */
    if (Accel_WriteReg(ACC_PWR_CONF, 0x00U) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(5U);

    /*
     * ACC_PWR_CTRL bit 2 enables the accelerometer.
     */
    if (Accel_WriteReg(ACC_PWR_CTRL, 0x04U) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(50U);

    /* Verify that both devices respond after reset. */
    if (IMU_ReadAccelID(&accel_id) != HAL_OK)
        return HAL_ERROR;

    if (IMU_ReadGyroID(&gyro_id) != HAL_OK)
        return HAL_ERROR;

    if (accel_id != ACC_EXPECTED_ID ||
        gyro_id != GYRO_EXPECTED_ID)
        return HAL_ERROR;

    if (identity != NULL)
    {
        identity->accel_id = accel_id;
        identity->gyro_id = gyro_id;
    }

    return HAL_OK;
}

HAL_StatusTypeDef IMU_ReadRaw(IMU_RawData *data)
{
    if (data == NULL)
        return HAL_ERROR;

    uint8_t a[6];
    uint8_t g[6];

    if (Accel_ReadRegs(ACC_DATA_START, a, 6U) != HAL_OK)
        return HAL_ERROR;

    if (Gyro_ReadRegs(GYRO_DATA_START, g, 6U) != HAL_OK)
        return HAL_ERROR;

    /* BMI088 data registers are little-endian, signed 16-bit. */
    data->ax = (int16_t)((uint16_t)a[0] |
                         ((uint16_t)a[1] << 8));

    data->ay = (int16_t)((uint16_t)a[2] |
                         ((uint16_t)a[3] << 8));

    data->az = (int16_t)((uint16_t)a[4] |
                         ((uint16_t)a[5] << 8));

    data->gx = (int16_t)((uint16_t)g[0] |
                         ((uint16_t)g[1] << 8));

    data->gy = (int16_t)((uint16_t)g[2] |
                         ((uint16_t)g[3] << 8));

    data->gz = (int16_t)((uint16_t)g[4] |
                         ((uint16_t)g[5] << 8));

    return HAL_OK;
}