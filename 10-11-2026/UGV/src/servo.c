/**
  ******************************************************************************
  * @file    servo.c
  * @brief   Thư viện điều khiển Servo lái UGV 1/10 - xem servo.h
  ******************************************************************************
  */
#include "servo.h"
#include <math.h>

#define TWO_PI_F  6.2831853f

Servo_t g_servo;

/* ------------------------------------------------------------------------- */
/* Kẹp an toàn: nhận int32 để tránh tràn/wrap khi tính toán ra số âm          */
/* ------------------------------------------------------------------------- */
uint16_t Servo_ClampSafe(int32_t pulse_us)
{
    int32_t lo = (int32_t)g_servo.pwm_min_safe_us;
    int32_t hi = (int32_t)g_servo.pwm_max_safe_us;

    /* Ngưỡng phần cứng tuyệt đối */
    if (pulse_us < (int32_t)SERVO_ABS_MIN_US) pulse_us = (int32_t)SERVO_ABS_MIN_US;
    if (pulse_us > (int32_t)SERVO_ABS_MAX_US) pulse_us = (int32_t)SERVO_ABS_MAX_US;

    /* Bảo vệ khi lỡ nhập sai min > max trên Live Expressions */
    if (lo < (int32_t)SERVO_ABS_MIN_US) lo = (int32_t)SERVO_ABS_MIN_US;
    if (hi > (int32_t)SERVO_ABS_MAX_US) hi = (int32_t)SERVO_ABS_MAX_US;
    if (lo > hi) { lo = (int32_t)SERVO_DEF_MIN_SAFE_US; hi = (int32_t)SERVO_DEF_MAX_SAFE_US; }

    /* Ngưỡng cơ khí đã khảo sát */
    if (pulse_us < lo) pulse_us = lo;
    if (pulse_us > hi) pulse_us = hi;

    return (uint16_t)pulse_us;
}

/* ------------------------------------------------------------------------- */
void Servo_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
    g_servo.htim    = htim;
    g_servo.channel = channel;

    g_servo.mode            = SERVO_MODE_MANUAL;   /* An toàn: khởi động ở chế độ tay */
    g_servo.pwm_neutral     = SERVO_DEF_NEUTRAL_US;
    g_servo.pwm_min_safe_us = SERVO_DEF_MIN_SAFE_US;
    g_servo.pwm_max_safe_us = SERVO_DEF_MAX_SAFE_US;
    g_servo.sweep_amp_us    = SERVO_DEF_SWEEP_AMP_US;
    g_servo.sine_period_ms  = SERVO_DEF_SINE_MS;
    g_servo.step_size_us    = SERVO_DEF_STEP_US;
    g_servo.step_hold_ms    = SERVO_DEF_STEP_HOLD_MS;
    g_servo.ramp_step_us    = SERVO_DEF_RAMP_US;

    uint16_t init_pwm = Servo_ClampSafe((int32_t)g_servo.pwm_neutral);
    g_servo.input_pwm_us   = init_pwm;
    g_servo.target_pwm_us  = init_pwm;
    g_servo.current_pwm_us = init_pwm;

    g_servo.sine_elapsed_ms = 0U;
    g_servo.step_offset_us  = 0;
    g_servo.step_direction  = 1;
    g_servo.step_last_tick  = HAL_GetTick();
    g_servo.last_update_tick = g_servo.step_last_tick;

    __HAL_TIM_SET_COMPARE(htim, channel, init_pwm);
    HAL_TIM_PWM_Start(htim, channel);
}

/* ------------------------------------------------------------------------- */
/* Tạo xung mục tiêu theo chế độ                                              */
/* ------------------------------------------------------------------------- */
static void Servo_UpdateGenerator(uint32_t now, uint32_t dt_ms)
{
    int32_t raw = (int32_t)g_servo.pwm_neutral;

    switch (g_servo.mode)
    {
    case SERVO_MODE_MANUAL:
        raw = (int32_t)g_servo.input_pwm_us;
        break;

    case SERVO_MODE_SINE:
    {
        /* pwm(t) = neutral + A * sin(2*pi*t/T), t tính theo thời gian thực */
        uint32_t period = (g_servo.sine_period_ms >= 500U) ? g_servo.sine_period_ms : 500U;
        g_servo.sine_elapsed_ms = (g_servo.sine_elapsed_ms + dt_ms) % period;

        float phase  = (TWO_PI_F * (float)g_servo.sine_elapsed_ms) / (float)period;
        float offset = (float)g_servo.sweep_amp_us * sinf(phase);
        raw = (int32_t)g_servo.pwm_neutral + (int32_t)lroundf(offset);
        break;
    }

    case SERVO_MODE_STEP:
    {
        if ((now - g_servo.step_last_tick) >= g_servo.step_hold_ms)
        {
            g_servo.step_last_tick = now;

            int16_t step    = (g_servo.step_size_us > 0U) ? (int16_t)g_servo.step_size_us : 10;
            int16_t max_amp = (int16_t)g_servo.sweep_amp_us;

            g_servo.step_offset_us += (int16_t)(g_servo.step_direction * step);

            /* Đảo chiều khi chạm biên */
            if (g_servo.step_offset_us >= max_amp)
            {
                g_servo.step_offset_us = max_amp;
                g_servo.step_direction = -1;
            }
            else if (g_servo.step_offset_us <= -max_amp)
            {
                g_servo.step_offset_us = (int16_t)(-max_amp);
                g_servo.step_direction = 1;
            }
        }
        raw = (int32_t)g_servo.pwm_neutral + (int32_t)g_servo.step_offset_us;
        break;
    }

    default:
        break;
    }

    /* Luôn qua chốt chặn an toàn trước khi vào bộ Ramp */
    g_servo.target_pwm_us = Servo_ClampSafe(raw);
}

/* ------------------------------------------------------------------------- */
/* Bộ trượt (ramp) chống giật                                                 */
/* ------------------------------------------------------------------------- */
static void Servo_RampToTarget(void)
{
    uint16_t step = (g_servo.ramp_step_us > 0U) ? g_servo.ramp_step_us : 1U;
    uint16_t cur  = g_servo.current_pwm_us;
    uint16_t tgt  = g_servo.target_pwm_us;

    if (cur < tgt)
    {
        cur = ((uint16_t)(tgt - cur) > step) ? (uint16_t)(cur + step) : tgt;
    }
    else if (cur > tgt)
    {
        cur = ((uint16_t)(cur - tgt) > step) ? (uint16_t)(cur - step) : tgt;
    }

    g_servo.current_pwm_us = cur;
    __HAL_TIM_SET_COMPARE(g_servo.htim, g_servo.channel, cur);
}

/* ------------------------------------------------------------------------- */
/* Gọi liên tục trong while(1); tự giữ nhịp 20 ms bằng HAL_GetTick()          */
/* (không dùng HAL_Delay nên không chặn các tác vụ khác)                      */
/* ------------------------------------------------------------------------- */
void Servo_Task(void)
{
    if (g_servo.htim == NULL) { return; }   /* Chưa Init */

    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - g_servo.last_update_tick;

    if (dt < SERVO_LOOP_DT_MS) { return; }
    g_servo.last_update_tick = now;

    Servo_UpdateGenerator(now, dt);
    Servo_RampToTarget();
}

/* ------------------------------------------------------------------------- */
void Servo_SetMode(Servo_Mode_t mode)
{
    g_servo.mode = (uint8_t)mode;
    g_servo.sine_elapsed_ms = 0U;
    g_servo.step_offset_us  = 0;
    g_servo.step_direction  = 1;
    g_servo.step_last_tick  = HAL_GetTick();
}

void Servo_SetPulseUs(uint16_t pulse_us)
{
    g_servo.input_pwm_us = pulse_us;
    g_servo.mode = SERVO_MODE_MANUAL;
}

/* value: -1.0 .. +1.0. Dương -> hướng pwm_max_safe, âm -> hướng pwm_min_safe.
 * Nếu chiều lắp servo ngược, chỉ cần đảo dấu khi gọi. */
void Servo_SetSteering(float value)
{
    if (value >  1.0f) value =  1.0f;
    if (value < -1.0f) value = -1.0f;

    float span = (value >= 0.0f)
               ? (float)((int32_t)g_servo.pwm_max_safe_us - (int32_t)g_servo.pwm_neutral)
               : (float)((int32_t)g_servo.pwm_neutral - (int32_t)g_servo.pwm_min_safe_us);

    int32_t pulse = (int32_t)g_servo.pwm_neutral + (int32_t)lroundf(value * span);
    Servo_SetPulseUs(Servo_ClampSafe(pulse));
}

void Servo_Center(void)
{
    Servo_SetPulseUs(g_servo.pwm_neutral);
}
