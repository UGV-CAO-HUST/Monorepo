#include "motor_test.h"
#include "motor.h" // Sử dụng hàm Motor_SetPulseUs() từ phần cứng có sẵn
#include <stdlib.h>

/* Cấu hình các thông số nội bộ */
#define THROTTLE_DEADBAND   30      // Khoảng chết của tay ga (trên thang 1000)
#define RAMP_STEP           10      // Bước tăng/giảm PWM cho mỗi chu kỳ
#define RAMP_DELAY_MS       50      // Thời gian trễ giữa các bước Ramp (ms)
#define DIRECTION_SAFE_MS   700     // Thời gian chờ để mô-tơ dừng hẳn trước khi đảo chiều
#define DOUBLE_TAP_MS       400     // Thời gian giữ phanh trước khi nhả về trung tính để lùi

/* Macro tiện ích */
#define CONSTRAIN(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/* Các bước của máy trạng thái */
typedef enum {
    SEQ_NONE,
    SEQ_DIR_WAIT,
    SEQ_TAP_BRAKE,
    SEQ_TAP_NEUTRAL
} EscSequence_t;

/* Biến cấu hình giới hạn tiến/lùi (có thể linh hoạt thay đổi nều cần) */
static int fwdLimit = 1800;
static int revLimit = 1200;

/* Các biến lưu trữ trạng thái hiện tại */
static int curPWM = ESC_PWM_NEUTRAL;
static int tgtPWM = ESC_PWM_NEUTRAL;
static int pendingPWM = ESC_PWM_NEUTRAL;
static bool pendingInstant = true;
static bool reverseArmed = false;

static EscSequence_t seq = SEQ_NONE;
static uint32_t seqEndMs = 0;
static uint32_t lastRampMs = 0;

/* Các hàm Helper nội bộ */
static inline int minLimit(void) { return (fwdLimit < revLimit) ? fwdLimit : revLimit; }
static inline int maxLimit(void) { return (fwdLimit > revLimit) ? fwdLimit : revLimit; }
static inline int revDir(void)   { return (revLimit < ESC_PWM_NEUTRAL) ? -1 : 1; }
static inline int tapPWM(void)   { return ESC_PWM_NEUTRAL + revDir() * 200; }

static int dirOf(int pwm) {
    if (pwm < ESC_PWM_NEUTRAL) return -1;
    if (pwm > ESC_PWM_NEUTRAL) return 1;
    return 0;
}

/* Hàm ghi xung PWM xuống tầng phần cứng thực tế */
static void setPWM(int us) {
    us = CONSTRAIN(us, ESC_PWM_MIN, ESC_PWM_MAX);
    Motor_SetPulseUs((uint32_t)us); // Gọi hàm ngoại vi từ motor.h
    curPWM = us;
}

/* Hàm tính toán điều khiển ga (Core Logic) */
static void requestPWM(int pwm, bool instant, uint32_t now) {
    pwm = CONSTRAIN(pwm, minLimit(), maxLimit());
    int nxt = dirOf(pwm);

    // Luôn hủy chuỗi đang chạy nếu yêu cầu về trung tính
    if (nxt == 0) {
        seq = SEQ_NONE;
        pendingPWM = ESC_PWM_NEUTRAL;
        tgtPWM = ESC_PWM_NEUTRAL;
        if (instant || curPWM == tapPWM()) {
            setPWM(ESC_PWM_NEUTRAL);
        }
        return;
    }

    pendingPWM = pwm;
    pendingInstant = instant;
    
    // Nếu đang chạy dở một chuỗi trạng thái khác, bỏ qua chờ chuỗi tự xử lý
    if (seq != SEQ_NONE) return;     

    int cur = dirOf(curPWM);

    // Xử lý đảo chiều trực tiếp: Đưa về trung tính và chờ động cơ dừng hẳn
    if (cur != 0 && cur != nxt) {
        setPWM(ESC_PWM_NEUTRAL);
        tgtPWM = ESC_PWM_NEUTRAL;
        seq = SEQ_DIR_WAIT;
        seqEndMs = now + DIRECTION_SAFE_MS;
        return;
    }

    // Xử lý Double-Tap để lùi (nhấp phanh)
    if (nxt == revDir()) {
        if (!reverseArmed) {
            setPWM(tapPWM());
            tgtPWM = ESC_PWM_NEUTRAL;
            seq = SEQ_TAP_BRAKE;
            seqEndMs = now + DOUBLE_TAP_MS;
            return;
        }
    } else {
        reverseArmed = false; // Đang chạy tiến thì lần lùi sau bắt buộc phải chạm phanh lại
    }

    tgtPWM = pwm;
    if (instant) {
        setPWM(pwm);
    }
}

/* --- CÁC HÀM API PUBLIC ĐƯỢC KHAI BÁO TRONG HEADER --- */

void ESC_Init(void) {
    curPWM = ESC_PWM_NEUTRAL;
    tgtPWM = ESC_PWM_NEUTRAL;
    pendingPWM = ESC_PWM_NEUTRAL;
    seq = SEQ_NONE;
    setPWM(ESC_PWM_NEUTRAL);
}

void ESC_RequestThrottle(int16_t throttle, bool instant) {
    throttle = CONSTRAIN(throttle, -1000, 1000);
    int pwm_target;
    
    if (abs(throttle) < THROTTLE_DEADBAND) {
        pwm_target = ESC_PWM_NEUTRAL;
    } else if (throttle > 0) {
        pwm_target = ESC_PWM_NEUTRAL + (int32_t)throttle * (fwdLimit - ESC_PWM_NEUTRAL) / 1000;
    } else {
        pwm_target = ESC_PWM_NEUTRAL + (int32_t)(-throttle) * (revLimit - ESC_PWM_NEUTRAL) / 1000;
    }
    
    // Gọi hàm request nội bộ, truyền thêm thời gian hiện tại
    requestPWM(pwm_target, instant, HAL_GetTick());
}

void ESC_Stop(void) {
    requestPWM(ESC_PWM_NEUTRAL, true, HAL_GetTick());
}

void ESC_ReduceHalf(void) {
    int offset = curPWM - ESC_PWM_NEUTRAL;
    requestPWM(ESC_PWM_NEUTRAL + offset / 2, true, HAL_GetTick());
}

void ESC_RawPulse(uint16_t us) {
    us = CONSTRAIN(us, ESC_PWM_MIN, ESC_PWM_MAX);
    seq = SEQ_NONE;
    tgtPWM = us;
    pendingPWM = us;
    setPWM(us);
}

uint16_t ESC_GetCurrentPWM(void) {
    return (uint16_t)curPWM;
}

void ESC_Update(uint32_t current_time_ms) {
    /* 1. Cập nhật máy trạng thái (Double-tap / Safe delay) */
    if (seq != SEQ_NONE && (int32_t)(current_time_ms - seqEndMs) >= 0) {
        switch (seq) {
            case SEQ_DIR_WAIT:
                seq = SEQ_NONE;
                requestPWM(pendingPWM, pendingInstant, current_time_ms);
                break;
            case SEQ_TAP_BRAKE:
                setPWM(ESC_PWM_NEUTRAL);
                seq = SEQ_TAP_NEUTRAL;
                seqEndMs = current_time_ms + DOUBLE_TAP_MS;
                break;
            case SEQ_TAP_NEUTRAL:
                reverseArmed = true; // Sẵn sàng lùi
                seq = SEQ_NONE;
                requestPWM(pendingPWM, pendingInstant, current_time_ms);
                break;
            default:
                seq = SEQ_NONE;
                break;
        }
    }

    /* 2. Cập nhật hiệu ứng Ramp (tăng/giảm tốc từ từ) */
    if (seq == SEQ_NONE && curPWM != tgtPWM) {
        if (current_time_ms - lastRampMs >= RAMP_DELAY_MS) {
            lastRampMs = current_time_ms;
            int next = curPWM;
            
            if (curPWM < tgtPWM) { 
                next += RAMP_STEP; 
                if (next > tgtPWM) next = tgtPWM; 
            } else { 
                next -= RAMP_STEP; 
                if (next < tgtPWM) next = tgtPWM; 
            }
            
            setPWM(next);
        }
    }
}