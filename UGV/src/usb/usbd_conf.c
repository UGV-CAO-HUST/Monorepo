/*
 * Lop LL cho ST USB Device Library tren USB_OTG_FS cua STM32H750 (PA11 = DM, PA12 = DP).
 * Xung 48 MHz lay tu PLL3Q (chinh xac, khong can CRS), cau hinh ngay trong MspInit
 * nen KHONG phai sua SystemClock_Config() cua project.
 *   PLL3: 64 MHz HSI / 4 = 16 MHz, x24 = 384 MHz VCO, /8 = 48 MHz.
 */
#include "usbd_conf.h"
#include "usbd_core.h"
#include "usbd_cdc.h"

PCD_HandleTypeDef hpcd_USB_OTG_FS;

void Error_Handler(void);

/* ---------------- MSP ---------------- */
void HAL_PCD_MspInit(PCD_HandleTypeDef *pcdHandle)
{
    GPIO_InitTypeDef g = {0};
    RCC_PeriphCLKInitTypeDef clk = {0};

    if (pcdHandle->Instance == USB_OTG_FS)
    {
        clk.PeriphClockSelection = RCC_PERIPHCLK_USB;
        clk.PLL3.PLL3M = 4;
        clk.PLL3.PLL3N = 24;
        clk.PLL3.PLL3P = 2;
        clk.PLL3.PLL3Q = 8;
        clk.PLL3.PLL3R = 2;
        clk.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
        clk.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
        clk.PLL3.PLL3FRACN = 0;
        clk.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
        if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK)
        {
            Error_Handler();
        }

        HAL_PWREx_EnableUSBVoltageDetector();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        g.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
        g.Mode      = GPIO_MODE_AF_PP;
        g.Pull      = GPIO_NOPULL;
        g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        g.Alternate = GPIO_AF10_OTG1_FS;
        HAL_GPIO_Init(GPIOA, &g);

        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

        HAL_NVIC_SetPriority(OTG_FS_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcdHandle)
{
    if (pcdHandle->Instance == USB_OTG_FS)
    {
        __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
        HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    }
}

void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

/* ---------------- HAL PCD callbacks -> USBD core ---------------- */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetupStage((USBD_HandleTypeDef *)hpcd->pData, (uint8_t *)hpcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataOutStage((USBD_HandleTypeDef *)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataInStage((USBD_HandleTypeDef *)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SOF((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetSpeed((USBD_HandleTypeDef *)hpcd->pData, USBD_SPEED_FULL);
    USBD_LL_Reset((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Suspend((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Resume((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_IsoINIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_DevConnected((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_DevDisconnected((USBD_HandleTypeDef *)hpcd->pData);
}

/* ---------------- USBD_LL_* ---------------- */
static USBD_StatusTypeDef to_usbd(HAL_StatusTypeDef s)
{
    switch (s)
    {
    case HAL_OK:    return USBD_OK;
    case HAL_BUSY:  return USBD_BUSY;
    default:        return USBD_FAIL;
    }
}

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    hpcd_USB_OTG_FS.Instance                     = USB_OTG_FS;
    hpcd_USB_OTG_FS.Init.dev_endpoints           = 9;
    hpcd_USB_OTG_FS.Init.speed                   = PCD_SPEED_FULL;
    hpcd_USB_OTG_FS.Init.dma_enable              = DISABLE;
    hpcd_USB_OTG_FS.Init.phy_itface              = PCD_PHY_EMBEDDED;
    hpcd_USB_OTG_FS.Init.Sof_enable              = DISABLE;
    hpcd_USB_OTG_FS.Init.low_power_enable        = DISABLE;
    hpcd_USB_OTG_FS.Init.lpm_enable              = DISABLE;
    hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.vbus_sensing_enable     = DISABLE;  /* VBUS (PA9) khong dung */
    hpcd_USB_OTG_FS.Init.use_dedicated_ep1       = DISABLE;

    hpcd_USB_OTG_FS.pData = pdev;
    pdev->pData = &hpcd_USB_OTG_FS;

    if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x80);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    return to_usbd(HAL_PCD_DeInit((PCD_HandleTypeDef *)pdev->pData));
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
    return to_usbd(HAL_PCD_Start((PCD_HandleTypeDef *)pdev->pData));
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
    return to_usbd(HAL_PCD_Stop((PCD_HandleTypeDef *)pdev->pData));
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps)
{
    return to_usbd(HAL_PCD_EP_Open((PCD_HandleTypeDef *)pdev->pData, ep_addr, ep_mps, ep_type));
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return to_usbd(HAL_PCD_EP_Close((PCD_HandleTypeDef *)pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return to_usbd(HAL_PCD_EP_Flush((PCD_HandleTypeDef *)pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return to_usbd(HAL_PCD_EP_SetStall((PCD_HandleTypeDef *)pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return to_usbd(HAL_PCD_EP_ClrStall((PCD_HandleTypeDef *)pdev->pData, ep_addr));
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    PCD_HandleTypeDef *h = (PCD_HandleTypeDef *)pdev->pData;
    return ((ep_addr & 0x80U) == 0x80U) ? h->IN_ep[ep_addr & 0x7FU].is_stall
                                        : h->OUT_ep[ep_addr & 0x7FU].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
    return to_usbd(HAL_PCD_SetAddress((PCD_HandleTypeDef *)pdev->pData, dev_addr));
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
    return to_usbd(HAL_PCD_EP_Transmit((PCD_HandleTypeDef *)pdev->pData, ep_addr, pbuf, size));
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
    return to_usbd(HAL_PCD_EP_Receive((PCD_HandleTypeDef *)pdev->pData, ep_addr, pbuf, size));
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef *)pdev->pData, ep_addr);
}

void USBD_LL_Delay(uint32_t Delay)
{
    HAL_Delay(Delay);
}

void *USBD_static_malloc(uint32_t size)
{
    static uint32_t mem[(sizeof(USBD_CDC_HandleTypeDef) / 4U) + 1U];
    (void)size;
    return mem;
}

void USBD_static_free(void *p)
{
    (void)p;
}
