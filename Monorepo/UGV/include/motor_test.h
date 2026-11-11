#ifndef ESC_CONTROLLER_H
#define ESC_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

/* Cấu hình giới hạn xung PWM chuẩn của ESC */
#define ESC_PWM_MIN         1000
#define ESC_PWM_NEUTRAL     1500
#define ESC_PWM_MAX         2000

/* 
 * Khởi tạo trạng thái ban đầu cho bộ điều khiển ESC.
 * Lưu ý: Phần cứng Timer/PWM phải được khởi tạo trước (VD: Motor_Init).
 */
void ESC_Init(void);

/* 
 * Hàm cập nhật trạng thái ESC. 
 * CẦN ĐƯỢC GỌI LIÊN TỤC trong vòng lặp chính (while 1) hoặc ngắt timer.
 * @param current_time_ms: Thời gian hiện tại tính bằng mili-giây (thường dùng HAL_GetTick()).
 */
void ESC_Update(uint32_t current_time_ms);

/*
 * Đặt mức ga dựa trên tỷ lệ (-1000 đến 1000).
 * @param throttle: Dương là tiến, Âm là lùi, 0 là trung tính.
 * @param instant: true = nhảy số ngay lập tức, false = tăng/giảm tốc từ từ qua Ramp.
 */
void ESC_RequestThrottle(int16_t throttle, bool instant);

/*
 * Dừng động cơ ngay lập tức (về mức trung tính).
 */
void ESC_Stop(void);

/*
 * Giảm một nửa mức ga hiện tại (phanh nhẹ).
 */
void ESC_ReduceHalf(void);

/*
 * Phát xung thô trực tiếp xuống ESC, bỏ qua mọi tính năng an toàn và trạng thái.
 * Chỉ nên dùng để Test hoặc Calibrate ESC.
 */
void ESC_RawPulse(uint16_t us);

/* Lấy xung PWM đang xuất ra thực tế */
uint16_t ESC_GetCurrentPWM(void);

#endif /* ESC_CONTROLLER_H */