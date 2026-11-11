#ifndef UROS_APP_H
#define UROS_APP_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Chay micro-ROS o che do "cooperative" (khong RTOS).
 * Ham nay tu khoi tao o lan goi dau tien, khong chan, tu dong ket noi lai agent.
 * Duoc goi tu ban override cua HAL_Delay() (xem uros_app.c),
 * nen main.c khong can sua gi.
 */
void Uros_Spin(void);

/* true neu dang ket noi voi micro-ROS agent. */
bool Uros_IsConnected(void);

#ifdef __cplusplus
}
#endif

#endif /* UROS_APP_H */
