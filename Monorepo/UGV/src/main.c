#include "stm32h7xx_hal.h"
#include "init.h"
#include "motor.h"
#include "motor_test.h" 

volatile uint32_t debug_stage = 0;

int main(void)
{
    /* Khởi tạo hệ thống cơ bản */
    HAL_Init();
    SystemClock_Config();
    Board_Init();

    /* 1. Khởi tạo tầng phần cứng (Timer PWM) */
    if (Motor_Init(&htim3, MOTOR_PWM_TIM_CHANNEL) != MOTOR_OK)
    {
        Error_Handler();
    }

    /* 2. Khởi tạo tầng logic điều khiển thuật toán ESC */
    ESC_Init();

    /* Biến dùng để chạy kịch bản test mà không làm treo code */
    uint32_t last_test_ms = HAL_GetTick();

    while (1)
    {
        uint32_t current_ms = HAL_GetTick();

        /* BẮT BUỘC: Gọi liên tục trong while(1) để ESC tính toán gia tốc (ramp) và nhấp phanh */
        ESC_Update(current_ms);

        /* 
         * Kịch bản test 5 giây/lần thay thế cho HAL_Delay() cũ 
         * Bạn có thể xóa phần này khi ghép code giao tiếp UART với Orange Pi
         */
        if (current_ms - last_test_ms >= 5000U)
        {
            last_test_ms = current_ms;
            debug_stage++;

            if (debug_stage == 1)
            {
                /* Tiến từ từ (mức ga 100/1000) */
                ESC_RequestThrottle(100, false);
            }
            else if (debug_stage == 2)
            {
                /* Chuyển sang lùi (mức ga -200/1000). 
                 * ESC_Update sẽ tự chặn lại, nhấp phanh và nhả phanh an toàn. */
                ESC_RequestThrottle(-200, false);
            }
            else if (debug_stage == 3)
            {
                /* Vẫn giữ lùi để xem động cơ chạy */
                ESC_RequestThrottle(-200, false);
            }
            else if (debug_stage == 4)
            {
                /* Dừng ngay lập tức (instant = true) về trung tính */
                ESC_RequestThrottle(0, true);
                debug_stage = 0; /* Lặp lại kịch bản test */
            }
        }
    }
}