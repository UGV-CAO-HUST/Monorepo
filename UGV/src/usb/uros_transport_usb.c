/*
 * Transport micro-ROS qua USB CDC (cong USB-C cua DevEBox).
 * Phia Orange Pi: xuat hien /dev/ttyACM0.
 *   - RX: callback trong ISR day byte vao ring buffer.
 *   - TX: gui tung khoi <= 512 byte, cho hoan thanh (co timeout, tu phuc hoi).
 */
#include "uros_transport.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_conf.h"
#include <string.h>

#define RX_BUF_SIZE   2048U            /* luy thua cua 2 */
#define RX_MASK       (RX_BUF_SIZE - 1U)
#define TX_CHUNK      512U
#define TX_TIMEOUT_MS 50U

#define CDC_REQ_SET_LINE_CODING 0x20U
#define CDC_REQ_GET_LINE_CODING 0x21U

static USBD_HandleTypeDef hUsbDeviceFS;

static volatile uint8_t  rx_ring[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0U;
static volatile uint16_t rx_tail = 0U;

__ALIGN_BEGIN static uint8_t usb_rx_pkt[64] __ALIGN_END;
__ALIGN_BEGIN static uint8_t usb_tx_buf[TX_CHUNK] __ALIGN_END;
static volatile uint8_t tx_busy = 0U;

static uint8_t line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08 }; /* 115200 8N1 */
static bool opened = false;

/* ---------- CDC interface callbacks ---------- */
static int8_t cdc_init(void)
{
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, usb_tx_buf, 0U);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, usb_rx_pkt);
    return (int8_t)USBD_OK;
}

static int8_t cdc_deinit(void)
{
    return (int8_t)USBD_OK;
}

static int8_t cdc_control(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    (void)length;
    if (cmd == CDC_REQ_SET_LINE_CODING)      { memcpy(line_coding, pbuf, sizeof(line_coding)); }
    else if (cmd == CDC_REQ_GET_LINE_CODING) { memcpy(pbuf, line_coding, sizeof(line_coding)); }
    return (int8_t)USBD_OK;
}

static int8_t cdc_receive(uint8_t *buf, uint32_t *len)       /* chay trong ISR */
{
    for (uint32_t i = 0U; i < *len; i++)
    {
        uint16_t next = (uint16_t)((rx_head + 1U) & RX_MASK);
        if (next == rx_tail)
        {
            break;                                           /* day -> bo */
        }
        rx_ring[rx_head] = buf[i];
        rx_head = next;
    }
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, usb_rx_pkt);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return (int8_t)USBD_OK;
}

static int8_t cdc_tx_cplt(uint8_t *buf, uint32_t *len, uint8_t epnum)
{
    (void)buf; (void)len; (void)epnum;
    tx_busy = 0U;
    return (int8_t)USBD_OK;
}

static USBD_CDC_ItfTypeDef cdc_fops =
{
    cdc_init, cdc_deinit, cdc_control, cdc_receive, cdc_tx_cplt
};

/* ---------- Transport ---------- */
bool uros_transport_open(struct uxrCustomTransport *transport)
{
    (void)transport;

    if (opened)
    {
        return true;
    }

    if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)        { return false; }
    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)          { return false; }
    if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &cdc_fops) != USBD_OK)  { return false; }
    if (USBD_Start(&hUsbDeviceFS) != USBD_OK)                             { return false; }

    opened = true;
    return true;
}

bool uros_transport_close(struct uxrCustomTransport *transport)
{
    (void)transport;
    return true;                         /* giu USB chay de khong bi re-enumerate */
}

size_t uros_transport_write(struct uxrCustomTransport *transport,
                            const uint8_t *buf, size_t len, uint8_t *err)
{
    (void)transport;
    size_t sent = 0U;

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        *err = 1U;
        return 0U;
    }

    while (sent < len)
    {
        uint16_t n = (uint16_t)(((len - sent) > TX_CHUNK) ? TX_CHUNK : (len - sent));
        memcpy(usb_tx_buf, &buf[sent], n);

        USBD_CDC_SetTxBuffer(&hUsbDeviceFS, usb_tx_buf, n);
        tx_busy = 1U;
        if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK)
        {
            tx_busy = 0U;
            *err = 1U;
            return sent;
        }

        uint32_t t0 = HAL_GetTick();
        while (tx_busy != 0U)
        {
            if ((uint32_t)(HAL_GetTick() - t0) >= TX_TIMEOUT_MS)
            {
                /* Host khong doc (cong chua mo): xoa trang thai de lan sau gui lai duoc. */
                (void)USBD_LL_FlushEP(&hUsbDeviceFS, CDC_IN_EP);
                ((USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData)->TxState = 0U;
                tx_busy = 0U;
                *err = 1U;
                return sent;
            }
        }
        sent += n;
    }
    return sent;
}

size_t uros_transport_read(struct uxrCustomTransport *transport,
                           uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    (void)transport;
    (void)err;

    /* Khong dung HAL_Delay o day: HAL_Delay da goi Uros_Spin -> de quy. */
    uint32_t start = HAL_GetTick();
    while (rx_head == rx_tail)
    {
        if ((uint32_t)(HAL_GetTick() - start) >= (uint32_t)timeout)
        {
            return 0U;
        }
    }

    size_t n = 0U;
    while ((n < len) && (rx_tail != rx_head))
    {
        buf[n++] = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) & RX_MASK);
    }
    return n;
}
