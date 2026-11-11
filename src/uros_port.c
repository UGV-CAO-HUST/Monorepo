/*
 * Lop port cho micro-ROS tren bare-metal (khong RTOS, khong Arduino):
 *   - clock_gettime(): micro-ROS bat buoc phai co
 *   - _sbrk(): heap cho malloc/free cua newlib (project goc chua co libc)
 */
#include <stddef.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include "stm32h7xx_hal.h"

/* ---------------- clock_gettime ---------------- */
int clock_gettime(clockid_t unused, struct timespec *tp)
{
    static uint32_t last_ms = 0U;
    static uint64_t total_ms = 0U;

    (void)unused;

    /* Mo rong HAL_GetTick() (32 bit, tran sau ~49 ngay) thanh 64 bit. */
    uint32_t now = HAL_GetTick();
    total_ms += (uint32_t)(now - last_ms);
    last_ms = now;

    tp->tv_sec  = (time_t)(total_ms / 1000ULL);
    tp->tv_nsec = (long)((total_ms % 1000ULL) * 1000000ULL);
    return 0;
}

/* ---------------- heap cho malloc ---------------- */
#ifndef UROS_HEAP_SIZE
#define UROS_HEAP_SIZE (64U * 1024U)   /* nam trong .bss -> AXI SRAM (512 KB) */
#endif

static uint8_t heap_area[UROS_HEAP_SIZE] __attribute__((aligned(8)));
static size_t  heap_used = 0U;

void *_sbrk(ptrdiff_t incr)
{
    if ((incr < 0) || ((heap_used + (size_t)incr) > UROS_HEAP_SIZE))
    {
        errno = ENOMEM;
        return (void *)-1;
    }

    void *prev = &heap_area[heap_used];
    heap_used += (size_t)incr;
    return prev;
}
