#include "motor_control.h"

typedef enum {
    MODE_NORMAL,       // Chạy bình thường (dùng ramp)
    MODE_REV_NEUTRAL,  // Lùi bước 1: Đợi xe giảm tốc về neutral
    MODE_REV_TAP1,     // Lùi bước 2: Nhấp lùi (min pulse)
    MODE_REV_TAP2,     // Lùi bước 3: Nhả về neutral
    MODE_BRAKING       // Phanh gấp
} Motor_Mode_t;

static Motor_Mode_t current_mode = MODE_NORMAL;
static uint32_t state_timer = 0;

static uint32_t current_pulse = MOTOR_NEUTRAL_PULSE_US;
static uint32_t target_pulse = MOTOR_NEUTRAL_PULSE_US;
static uint32_t reverse_target = MOTOR_NEUTRAL_PULSE_US;
static uint32_t last_ramp_tick = 0;

Motor_Status MotorControl_Init(TIM_HandleTypeDef *htim, uint32_t channel) {
    Motor_Status status;

    status = Motor_Init(htim, channel);
    if (status != MOTOR_OK) {
        return status;
    }

    status = Motor_SetPulseUs(MOTOR_NEUTRAL_PULSE_US);
    if (status != MOTOR_OK) {
        return status;
    }

    current_mode = MODE_NORMAL;
    current_pulse = MOTOR_NEUTRAL_PULSE_US;
    target_pulse = MOTOR_NEUTRAL_PULSE_US;
    last_ramp_tick = HAL_GetTick();

    return MOTOR_OK;
}

void MotorControl_Forward(uint32_t speed_pulse) {
    if (speed_pulse < MOTOR_NEUTRAL_PULSE_US) speed_pulse = MOTOR_NEUTRAL_PULSE_US;
    if (speed_pulse > MOTOR_MAX_PULSE_US) speed_pulse = MOTOR_MAX_PULSE_US;
    
    current_mode = MODE_NORMAL;
    target_pulse = speed_pulse;
}

void MotorControl_Reverse(uint32_t speed_pulse) {
    if (speed_pulse > MOTOR_NEUTRAL_PULSE_US) speed_pulse = MOTOR_NEUTRAL_PULSE_US;
    if (speed_pulse < MOTOR_MIN_PULSE_US) speed_pulse = MOTOR_MIN_PULSE_US;
    
    if (current_pulse < MOTOR_NEUTRAL_PULSE_US && target_pulse < MOTOR_NEUTRAL_PULSE_US && current_mode == MODE_NORMAL) {
        target_pulse = speed_pulse;
    } else if (current_mode != MODE_REV_NEUTRAL && current_mode != MODE_REV_TAP1 && current_mode != MODE_REV_TAP2) {
        reverse_target = speed_pulse;
        target_pulse = MOTOR_NEUTRAL_PULSE_US; 
        current_mode = MODE_REV_NEUTRAL;
        state_timer = HAL_GetTick();
    }
}

void MotorControl_Brake(void) {
    current_mode = MODE_BRAKING;
    state_timer = HAL_GetTick();
    target_pulse = MOTOR_NEUTRAL_PULSE_US; 
    
    if (current_pulse > MOTOR_NEUTRAL_PULSE_US) {
        current_pulse = MOTOR_MIN_PULSE_US; 
    } else {
        current_pulse = MOTOR_NEUTRAL_PULSE_US; 
    }
    Motor_SetPulseUs(current_pulse);
}

void MotorControl_Process(void) {
    uint32_t now = HAL_GetTick();

    switch (current_mode) {
        case MODE_NORMAL:
        case MODE_REV_NEUTRAL:
            if (current_pulse != target_pulse) {
                if (now - last_ramp_tick >= 50U) {
                    if (current_pulse < target_pulse) {
                        current_pulse += 10U;
                        if (current_pulse > target_pulse) current_pulse = target_pulse;
                    } else {
                        current_pulse -= 10U;
                        if (current_pulse < target_pulse) current_pulse = target_pulse;
                    }
                    Motor_SetPulseUs(current_pulse);
                    last_ramp_tick = now;
                }
            }
            
            if (current_mode == MODE_REV_NEUTRAL && current_pulse == MOTOR_NEUTRAL_PULSE_US) {
                if (now - state_timer >= 200U) { 
                    current_pulse = MOTOR_MIN_PULSE_US;
                    Motor_SetPulseUs(current_pulse); 
                    current_mode = MODE_REV_TAP1;
                    state_timer = now;
                }
            }
            break;

        case MODE_REV_TAP1:
            if (now - state_timer >= 200U) {
                current_pulse = MOTOR_NEUTRAL_PULSE_US;
                Motor_SetPulseUs(current_pulse);
                current_mode = MODE_REV_TAP2;
                state_timer = now;
            }
            break;

        case MODE_REV_TAP2:
            if (now - state_timer >= 200U) {
                target_pulse = reverse_target; 
                current_mode = MODE_NORMAL;    
            }
            break;

        case MODE_BRAKING:
            if (now - state_timer >= 500U) { 
                current_pulse = MOTOR_NEUTRAL_PULSE_US;
                target_pulse = MOTOR_NEUTRAL_PULSE_US;
                Motor_SetPulseUs(current_pulse);
                current_mode = MODE_NORMAL;
            }
            break;
    }
}