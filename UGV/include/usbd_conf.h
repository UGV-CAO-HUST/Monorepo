#ifndef USBD_CONF_H
#define USBD_CONF_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32h7xx_hal.h"

#define USBD_MAX_NUM_INTERFACES      1U
#define USBD_MAX_NUM_CONFIGURATION   1U
#define USBD_MAX_STR_DESC_SIZ        256U
#define USBD_DEBUG_LEVEL             0U
#define USBD_LPM_ENABLED             0U
#define USBD_SELF_POWERED            1U

#define DEVICE_FS 0

#define USBD_malloc               (void *)USBD_static_malloc
#define USBD_free                 USBD_static_free
#define USBD_memset               memset
#define USBD_memcpy               memcpy
#define USBD_Delay                HAL_Delay

#define USBD_UsrLog(...)
#define USBD_ErrLog(...)
#define USBD_DbgLog(...)

void *USBD_static_malloc(uint32_t size);
void  USBD_static_free(void *p);

#endif /* USBD_CONF_H */
