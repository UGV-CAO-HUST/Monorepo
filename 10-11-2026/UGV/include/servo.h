/**
  ******************************************************************************
  * @file    servo.h
  * @brief   Thư viện điều khiển Servo lái UGV 1/10 (PWM 50 Hz, đơn vị micro-giây)
  *
  * Yêu cầu phần cứng/cấu hình Timer (đã dùng trong dự án gốc):
  *   - Timer chạy PWM, tick = 1 us  (VD: TIM2, Prescaler = 63 @ 64 MHz)
  *   - Period (ARR) = 19999  -> chu kỳ 20 ms (50 Hz)
  *   - Giá trị CCR = độ rộng xung tính bằng us
  *
  * Cách dùng:
  *   Servo_Init(&htim2, TIM_CHANNEL_1);   // sau MX_TIM2_Init()
  *   while (1) { Servo_Task(); }          // không chặn, tự giữ nhịp 20 ms
  *
  * Debug: thêm biến g_servo vào Live Expressions để chỉnh trực tiếp
  *        mode, pwm_neutral, sweep_amp_us, input_pwm_us, ...
  ******************************************************************************
  */
#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ---- Hằng số ------------------------------------------------------------ */
#define SERVO_LOOP_DT_MS        20U     /* Chu kỳ cập nhật (50 Hz)                  */

#define SERVO_ABS_MIN_US        500U    /* Giới hạn phần cứng tuyệt đối của servo   */
#define SERVO_ABS_MAX_US        2500U

/* Giá trị mặc định đã hiệu chỉnh trên xe thực tế */
#define SERVO_DEF_NEUTRAL_US    1400U   /* Vị trí 2 bánh thẳng                      */
#define SERVO_DEF_MIN_SAFE_US   700U    /* Kẹp cơ khí phía bên 1                    */
#define SERVO_DEF_MAX_SAFE_US   1900U   /* Kẹp cơ khí phía bên kia                  */
#define SERVO_DEF_SWEEP_AMP_US  500U    /* = min(1400-700, 1900-1400): quét đối xứng */
#define SERVO_DEF_SINE_MS       6000U
#define SERVO_DEF_STEP_US       25U
#define SERVO_DEF_STEP_HOLD_MS  1500U
#define SERVO_DEF_RAMP_US       4U      /* us tối đa mỗi 20 ms (chống giật)         */

/* ---- Kiểu dữ liệu ------------------------------------------------------- */
typedef enum
{
    SERVO_MODE_MANUAL = 0U,  /* Nhập tay qua input_pwm_us / Servo_SetPulseUs()  */
    SERVO_MODE_SINE   = 1U,  /* Quét hình sin quanh pwm_neutral                  */
    SERVO_MODE_STEP   = 2U   /* Quét từng nấc, dừng step_hold_ms ở mỗi nấc       */
} Servo_Mode_t;

typedef struct
{
    /* Phần cứng */
    TIM_HandleTypeDef *htim;
    uint32_t           channel;

    /* Cấu hình (có thể sửa lúc chạy qua Live Expressions) */
    volatile uint8_t   mode;
    volatile uint16_t  pwm_neutral;
    volatile uint16_t  pwm_min_safe_us;
    volatile uint16_t  pwm_max_safe_us;
    volatile uint16_t  sweep_amp_us;
    volatile uint32_t  sine_period_ms;
    volatile uint16_t  step_size_us;
    volatile uint32_t  step_hold_ms;
    volatile uint16_t  input_pwm_us;     /* Dùng cho MANUAL */
    volatile uint16_t  ramp_step_us;

    /* Trạng thái (chỉ đọc) */
    volatile uint16_t  target_pwm_us;    /* Sau bộ tạo sóng + kẹp an toàn */
    volatile uint16_t  current_pwm_us;   /* Giá trị đang nằm trong CCR    */

    /* Nội bộ */
    uint32_t           last_update_tick;
    uint32_t           sine_elapsed_ms;
    uint32_t           step_last_tick;
    int16_t            step_offset_us;
    int8_t             step_direction;
} Servo_t;

extern Servo_t g_servo;   /* Instance duy nhất, dùng cho Live Expressions */

/* ---- API ---------------------------------------------------------------- */
void     Servo_Init(TIM_HandleTypeDef *htim, uint32_t channel);
void     Servo_Task(void);                         /* Gọi liên tục trong while(1) */
void     Servo_SetMode(Servo_Mode_t mode);
void     Servo_SetPulseUs(uint16_t pulse_us);      /* Chuyển sang MANUAL + đặt xung */
void     Servo_SetSteering(float value);           /* -1.0 .. +1.0, 0 = thẳng       */
void     Servo_Center(void);                       /* Về trung hòa (MANUAL)         */
uint16_t Servo_ClampSafe(int32_t pulse_us);        /* Kẹp phần cứng + cơ khí        */

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */
