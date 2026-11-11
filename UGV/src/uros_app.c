/*
 * micro-ROS node cho UGV (khong RTOS).
 *
 *   Subscribe: /ugv/motor_pulse_us  (std_msgs/Int32)  -> Motor_SetPulseUs()
 *
 * Y tuong de KHONG phai doi cau truc main.c:
 *   HAL_Delay() trong HAL la ham __weak. O day ta override no: trong luc cho,
 *   no goi Uros_Spin() de micro-ROS van chay. main.c giu nguyen.
 */
#include "uros_app.h"
#include "uros_transport.h"
#include "motor.h"
#include "stm32h7xx_hal.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#define UROS_NODE_NAME        "ugv_stm32"
#define UROS_TOPIC_MOTOR      "ugv/motor_pulse_us"

#define UROS_PING_WAIT_MS     500U    /* chu ky do agent khi chua ket noi */
#define UROS_PING_ALIVE_MS    1000U   /* chu ky kiem tra agent khi dang ket noi */

/* Neu > 0: qua ngan nay ms khong nhan lenh moi thi tra ESC ve neutral (an toan).
 * Dat 0 de tat. */
#ifndef UROS_CMD_TIMEOUT_MS
#define UROS_CMD_TIMEOUT_MS   0U
#endif

static const uint32_t UROS_MOTOR_MIN_PULSE_US      = MOTOR_MIN_PULSE_US;
static const uint32_t UROS_MOTOR_MAX_PULSE_US      = MOTOR_MAX_PULSE_US;
static const uint32_t UROS_MOTOR_NEUTRAL_PULSE_US  = MOTOR_NEUTRAL_PULSE_US;
static uint32_t       last_motor_pulse_us           = MOTOR_NEUTRAL_PULSE_US;

typedef enum { UROS_INIT = 0, UROS_WAIT_AGENT, UROS_CONNECTED } uros_state_t;

static uros_state_t state = UROS_INIT;
static uint8_t      stage = 0U;       /* so entity da tao, de don dep khi loi */
static uint32_t     last_ping_ms = 0U;
static uint32_t     last_cmd_ms  = 0U;

static rcl_allocator_t allocator;
static rclc_support_t  support;
static rcl_node_t      node;
static rcl_subscription_t sub_motor;
static rclc_executor_t executor;
static std_msgs__msg__Int32 msg_motor;

static void motor_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msgin;

    if (m->data < 0)
    {
        return;
    }

    /* Motor_SetPulseUs() da tu gioi han 1000..2000 us. */
    if (Motor_SetPulseUs((uint32_t)m->data) == MOTOR_OK)
    {
        last_cmd_ms = HAL_GetTick();
    }
}

static void destroy_entities(void)
{
    if (stage >= 1U)
    {
        rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
        (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
    }
    if (stage >= 4U) { (void)rclc_executor_fini(&executor); }
    if (stage >= 3U) { (void)rcl_subscription_fini(&sub_motor, &node); }
    if (stage >= 2U) { (void)rcl_node_fini(&node); }
    if (stage >= 1U) { (void)rclc_support_fini(&support); }
    stage = 0U;
}

static bool create_entities(void)
{
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) { return false; }
    stage = 1U;

    if (rclc_node_init_default(&node, UROS_NODE_NAME, "", &support) != RCL_RET_OK) { return false; }
    stage = 2U;

    if (rclc_subscription_init_best_effort(
            &sub_motor, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            UROS_TOPIC_MOTOR) != RCL_RET_OK) { return false; }
    stage = 3U;

    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) { return false; }
    stage = 4U;

    if (rclc_executor_add_subscription(&executor, &sub_motor, &msg_motor,
                                       &motor_callback, ON_NEW_DATA) != RCL_RET_OK) { return false; }

    return true;
}

void Uros_Spin(void)
{
    static bool busy = false;

    if (busy)                      /* chong de quy: HAL_PCD/UART co the goi HAL_Delay */
    {
        return;
    }
    busy = true;

    uint32_t now = HAL_GetTick();

    switch (state)
    {
    case UROS_INIT:
        if (rmw_uros_set_custom_transport(true, NULL,
                uros_transport_open, uros_transport_close,
                uros_transport_write, uros_transport_read) == RMW_RET_OK)
        {
            last_ping_ms = now - UROS_PING_WAIT_MS;   /* ping ngay lan dau */
            state = UROS_WAIT_AGENT;
        }
        break;

    case UROS_WAIT_AGENT:
        if ((uint32_t)(now - last_ping_ms) >= UROS_PING_WAIT_MS)
        {
            last_ping_ms = now;
            if (rmw_uros_ping_agent(20, 1) == RMW_RET_OK)
            {
                if (create_entities())
                {
                    last_cmd_ms = 0U;
                    last_ping_ms = HAL_GetTick();
                    state = UROS_CONNECTED;
                }
                else
                {
                    destroy_entities();
                }
            }
        }
        break;

    case UROS_CONNECTED:
        (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

#if UROS_CMD_TIMEOUT_MS > 0U
        if ((last_cmd_ms != 0U) && ((uint32_t)(now - last_cmd_ms) > UROS_CMD_TIMEOUT_MS))
        {
            (void)Motor_SetPulseUs(MOTOR_NEUTRAL_PULSE_US);
            last_cmd_ms = 0U;
        }
#endif

        if ((uint32_t)(now - last_ping_ms) >= UROS_PING_ALIVE_MS)
        {
            last_ping_ms = now;
            if (rmw_uros_ping_agent(50, 2) != RMW_RET_OK)
            {
                (void)Motor_SetPulseUs(MOTOR_NEUTRAL_PULSE_US);   /* mat agent -> dung xe */
                destroy_entities();
                state = UROS_WAIT_AGENT;
            }
        }
        break;
    }

    busy = false;
}

bool Uros_IsConnected(void)
{
    return (state == UROS_CONNECTED);
}

/*
 * Override HAL_Delay() (ban goc trong HAL la __weak).
 * Hanh vi giong het ban goc, chi them viec chay micro-ROS trong luc cho.
 */
void HAL_Delay(uint32_t Delay)
{
    uint32_t tickstart = HAL_GetTick();
    uint32_t wait = Delay;

    if (wait < HAL_MAX_DELAY)
    {
        wait += (uint32_t)HAL_GetTickFreq();
    }

    while ((uint32_t)(HAL_GetTick() - tickstart) < wait)
    {
        Uros_Spin();
    }
}
