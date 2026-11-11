#include "stm32h7xx_hal.h"
#include "init.h"
#include "motor.h"

volatile uint32_t debug_stage = 0;

#ifdef UGV_BENCH_TEST
/* Chỉ dùng cho chế độ test trên bàn (thêm -DUGV_BENCH_TEST vào build_flags). */
static void ramp_to(uint32_t target)
{
    uint32_t cur = Motor_GetPulseUs();
    while (cur != target)
    {
        if (cur > target) { cur = (cur - 10U < target) ? target : cur - 10U; }
        else              { cur = (cur + 10U > target) ? target : cur + 10U; }
        Motor_SetPulseUs(cur);
        HAL_Delay(50U);
    }
}
#endif

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    Board_Init();

    if (Motor_Init(&htim3, MOTOR_PWM_TIM_CHANNEL) != MOTOR_OK)
    {
        Error_Handler();
    }

    /* Motor_Init() đã phát 1500 us. Chờ 5 giây để ESC arm ở neutral (chỉ 1 lần).
     * Trong lúc chờ, HAL_Delay() đã chạy micro-ROS nên agent kết nối sẵn. */
    HAL_Delay(5000U);

    while (1)
    {
#ifdef UGV_BENCH_TEST
        /* Chuỗi test cũ: tiến, neutral, double-tap, lùi. */
        ramp_to(1550U);            HAL_Delay(3000U);
        Motor_SetPulseUs(1500U);   HAL_Delay(200U);

        Motor_SetPulseUs(1000U);   HAL_Delay(200U);
        Motor_SetPulseUs(1500U);   HAL_Delay(200U);

        ramp_to(1420U);            HAL_Delay(3000U);
        Motor_SetPulseUs(1500U);   HAL_Delay(200U);
#else
        /* Chế độ ROS: lệnh đến từ /ugv/motor_pulse_us, xử lý trong callback
         * của Uros_Spin() (được HAL_Delay gọi). Vòng lặp chỉ cần chờ. */
        HAL_Delay(10U);
#endif
    }
}