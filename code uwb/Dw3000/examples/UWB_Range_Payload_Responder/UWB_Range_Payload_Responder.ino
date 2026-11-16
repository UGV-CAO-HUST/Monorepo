#include "dw3000.h"
#include "uwb_packet.h"
#include "SPI.h"
#include <string.h>

extern SPISettings _fastSPI;
extern dwt_txconfig_t txconfig_options;

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS  21

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define APP_HEADER_IDX UWB_APP_FRAME_PREFIX_LEN
#define APP_PAYLOAD_IDX UWB_APP_PAYLOAD_OFFSET
#define RX_BUFFER_LEN 127
#define POLL_RX_TO_RESP_TX_DLY_UUS 900

#define UWB_SOURCE_ID 1
#define UWB_PACKET_FLAGS 0

static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9,
    1, DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

static uint8_t rx_poll_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0
};

static uint8_t tx_response[RX_BUFFER_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1
};

static uint8_t rx_buffer[RX_BUFFER_LEN];
static uint8_t frame_seq_nb = 0;
static uint32_t status_reg = 0;

static void haltWithMessage(const char *message)
{
    Serial.println(message);
    while (true)
    {
        delay(1000);
    }
}

void setup()
{
    UART_init();
    _fastSPI = SPISettings(7000000L, MSBFIRST, SPI_MODE0);

    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);
    delay(2);

    dwt_softreset();
    delay(2);

    while (!dwt_checkidlerc())
    {
        haltWithMessage("DW3000 IDLE FAILED");
    }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        haltWithMessage("DW3000 INIT FAILED");
    }

    if (dwt_configure(&config) == DWT_ERROR)
    {
        haltWithMessage("DW3000 CONFIG FAILED");
    }

    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    Serial.println("UWB responder ready");
}

void loop()
{
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
    {
    }

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK))
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        return;
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len > sizeof(rx_buffer) || frame_len < ALL_MSG_COMMON_LEN)
    {
        return;
    }

    dwt_readrxdata(rx_buffer, frame_len, 0);
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) != 0)
    {
        return;
    }

    const uint64_t poll_rx_ts = get_rx_timestamp_u64();
    const uint32_t resp_tx_time =
        (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    const uint64_t resp_tx_ts =
        (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
    resp_msg_set_ts(&tx_response[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
    resp_msg_set_ts(&tx_response[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

    // Replace this sample with sensor bytes when connecting an actual data source.
    static const uint8_t sample_payload[] = "TEMP=25.3;HUM=60";
    const uint8_t payload_len = sizeof(sample_payload) - 1;
    if (payload_len > UWB_APP_PAYLOAD_MAX_LEN)
    {
        return;
    }

    tx_response[ALL_MSG_SN_IDX] = frame_seq_nb;
    tx_response[APP_HEADER_IDX + UWB_APP_MAGIC_0_OFFSET] = UWB_APP_MAGIC_0;
    tx_response[APP_HEADER_IDX + UWB_APP_MAGIC_1_OFFSET] = UWB_APP_MAGIC_1;
    tx_response[APP_HEADER_IDX + UWB_APP_VERSION_OFFSET] = UWB_APP_VERSION;
    tx_response[APP_HEADER_IDX + UWB_APP_TYPE_OFFSET] = UWB_APP_TYPE_SENSOR_DATA;
    tx_response[APP_HEADER_IDX + UWB_APP_SEQUENCE_OFFSET] = frame_seq_nb;
    uwb_packet_write_u16_le(&tx_response[APP_HEADER_IDX + UWB_APP_SOURCE_ID_OFFSET], UWB_SOURCE_ID);
    tx_response[APP_HEADER_IDX + UWB_APP_FLAGS_OFFSET] = UWB_PACKET_FLAGS;
    uwb_packet_write_u32_le(&tx_response[APP_HEADER_IDX + UWB_APP_TIMESTAMP_MS_OFFSET], millis());
    tx_response[APP_HEADER_IDX + UWB_APP_PAYLOAD_LEN_OFFSET] = payload_len;
    memcpy(&tx_response[APP_PAYLOAD_IDX], sample_payload, payload_len);

    const uint16_t response_len = APP_PAYLOAD_IDX + payload_len + FCS_LEN;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(response_len, tx_response, 0);
    dwt_writetxfctrl(response_len, 0, 1);

    if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS)
    {
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
        {
        }

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        frame_seq_nb++;
    }
    else
    {
        Serial.println("Delayed response TX failed");
    }
}
