/* Descriptor cho thiet bi CDC (cong COM ao). VID/PID la cua ST (dung cho thu nghiem). */
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"
#include "usbd_ctlreq.h"

#define USBD_VID                     0x0483
#define USBD_PID_FS                  0x5740
#define USBD_LANGID_STRING           1033
#define USBD_MANUFACTURER_STRING     "UGV"
#define USBD_PRODUCT_STRING_FS       "UGV micro-ROS"
#define USBD_SERIAL_STRING_FS        "UGV0001"
#define USBD_CONFIGURATION_STRING_FS "CDC Config"
#define USBD_INTERFACE_STRING_FS     "CDC Interface"

static uint8_t *dev_desc(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *langid_desc(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *manuf_desc(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *product_desc(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *serial_desc(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *config_desc(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *interface_desc(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef FS_Desc =
{
    .GetDeviceDescriptor           = dev_desc,
    .GetLangIDStrDescriptor        = langid_desc,
    .GetManufacturerStrDescriptor  = manuf_desc,
    .GetProductStrDescriptor       = product_desc,
    .GetSerialStrDescriptor        = serial_desc,
    .GetConfigurationStrDescriptor = config_desc,
    .GetInterfaceStrDescriptor     = interface_desc,
};

__ALIGN_BEGIN static uint8_t dev_desc_buf[USB_LEN_DEV_DESC] __ALIGN_END =
{
    0x12, USB_DESC_TYPE_DEVICE,
    0x00, 0x02,                       /* bcdUSB 2.00 */
    0x02, 0x02, 0x00,                 /* class CDC, subclass ACM */
    USB_MAX_EP0_SIZE,
    LOBYTE(USBD_VID), HIBYTE(USBD_VID),
    LOBYTE(USBD_PID_FS), HIBYTE(USBD_PID_FS),
    0x00, 0x02,                       /* bcdDevice */
    USBD_IDX_MFC_STR, USBD_IDX_PRODUCT_STR, USBD_IDX_SERIAL_STR,
    USBD_MAX_NUM_CONFIGURATION
};

__ALIGN_BEGIN static uint8_t langid_buf[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
    USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING,
    LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING)
};

__ALIGN_BEGIN static uint8_t str_buf[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

static uint8_t *dev_desc(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = (uint16_t)sizeof(dev_desc_buf);
    return dev_desc_buf;
}

static uint8_t *langid_desc(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = (uint16_t)sizeof(langid_buf);
    return langid_buf;
}

static uint8_t *manuf_desc(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, str_buf, length);
    return str_buf;
}

static uint8_t *product_desc(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, str_buf, length);
    return str_buf;
}

static uint8_t *serial_desc(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_SERIAL_STRING_FS, str_buf, length);
    return str_buf;
}

static uint8_t *config_desc(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, str_buf, length);
    return str_buf;
}

static uint8_t *interface_desc(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, str_buf, length);
    return str_buf;
}
