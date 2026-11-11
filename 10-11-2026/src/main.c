#include "stm32h7xx_hal.h"
#include "init.h"
#include "motor.h"
#include "motor_control.h"

volatile uint32_t debug_stage = 0;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    Board_Init();

    if (MotorControl_Init(&htim3, MOTOR_PWM_TIM_CHANNEL) != MOTOR_OK)
    {
        Error_Handler();
    }

    uint32_t seq_timer = HAL_GetTick();
    uint8_t seq_step = 0;

    // Chờ 3 giây đầu tiên để ESC khởi động xong hẳn rồi mới bắt đầu chạy kịch bản
    uint8_t startup_delay = 1; 

    while (1)
    {
        // Hệ thống duy trì các hiệu ứng tốc độ, phanh, nhấp nhả ngầm ở đây
        MotorControl_Process();

        uint32_t now = HAL_GetTick();

        if (startup_delay) {
            if (now - seq_timer >= 3000U) {
                startup_delay = 0;
                seq_timer = now;
            }
            continue; // Bỏ qua kịch bản bên dưới cho đến khi hết 3s khởi động
        }

        switch (seq_step) {
            case 0: 
                // 1. Bắt đầu: Tiến chậm 2s
                MotorControl_Forward(1540U); 
                seq_timer = now;
                seq_step = 1;
                break;

            case 1: 
                // 2. Sau 2s tiến, ra lệnh Lùi
                if (now - seq_timer >= 2000U) {
                    MotorControl_Reverse(1450U); // Lùi chậm
                    seq_timer = now;
                    seq_step = 2;
                }
                break;

            case 2: 
                // 3. Đợi 2s (tính cả thời gian ESC tự động nhấp nhả) rồi Phanh gấp
                if (now - seq_timer >= 2000U) {
                    MotorControl_Brake(); 
                    seq_timer = now;
                    seq_step = 3;
                }
                break;

            case 3: 
                // 4. Hàm Brake giữ phanh 500ms ngầm, ta đợi 1s cho an toàn rồi Lùi tiếp
                if (now - seq_timer >= 1000U) { 
                    MotorControl_Reverse(1450U);
                    seq_timer = now;
                    seq_step = 4;
                }
                break;

            case 4: 
                // 5. Đợi 2s lùi rồi chuyển sang Tiến 5s
                if (now - seq_timer >= 2000U) {
                    MotorControl_Forward(1550U); 
                    seq_timer = now;
                    seq_step = 5;
                }
                break;

            case 5: 
                // 6. Đợi 5s tiến rồi phanh dừng kịch bản
                if (now - seq_timer >= 5000U) {
                    MotorControl_Brake(); 
                    seq_step = 6;
                }
                break;

            case 6:
                // Kết thúc, xe đứng yên tại Neutral
                break;
        }
    }
}