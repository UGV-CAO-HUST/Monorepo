#include <dw3000.h>
#include <dw3000_board_config.h>
#include <dw3000_config_options.h>
#include <dw3000_device_api.h>
#include <dw3000_mac_802_15_4.h>
#include <dw3000_port.h>
#include <dw3000_regs.h>
#include <dw3000_shared_defines.h>
#include <dw3000_shared_functions.h>
#include <dw3000_types.h>
#include <dw3000_uart.h>
#include <dw3000_vals.h>
#include <dw3000_version.h>

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
#define POLL_TX_TO_RESP_RX_DLY_UUS 600
#define RESP_RX_TIMEOUT_UUS 400

static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9,
    1, DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

static uint8_t tx_poll_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0
};

static uint8_t rx_response_msg[] = {
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
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    Serial.println("UWB initiator ready");
}

void loop()
{
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) == DWT_ERROR)
    {
        Serial.println("Poll TX failed");
        return;
    }

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    {
    }
    frame_seq_nb++;

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK))
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        Serial.print("No valid response; SYS_STATUS=0x");
        Serial.println(status_reg, HEX);
        delay(500);
        return;
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    const uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len > sizeof(rx_buffer) ||
        frame_len < APP_HEADER_IDX + UWB_APP_PACKET_HEADER_LEN + FCS_LEN)
    {
        Serial.println("Response frame length invalid");
        return;
    }

    dwt_readrxdata(rx_buffer, frame_len, 0);
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    if (memcmp(rx_buffer, rx_response_msg, ALL_MSG_COMMON_LEN) != 0)
    {
        Serial.println("Unexpected response frame");
        return;
    }

    const uint16_t data_len = frame_len - FCS_LEN;
    if (rx_buffer[APP_HEADER_IDX + UWB_APP_MAGIC_0_OFFSET] != UWB_APP_MAGIC_0 ||
        rx_buffer[APP_HEADER_IDX + UWB_APP_MAGIC_1_OFFSET] != UWB_APP_MAGIC_1 ||
        rx_buffer[APP_HEADER_IDX + UWB_APP_VERSION_OFFSET] != UWB_APP_VERSION)
    {
        Serial.println("Unknown application packet");
        return;
    }

    const uint8_t payload_len = rx_buffer[APP_HEADER_IDX + UWB_APP_PAYLOAD_LEN_OFFSET];
    if (APP_PAYLOAD_IDX + payload_len > data_len)
    {
        Serial.println("Payload length invalid");
        return;
    }

    uint32_t poll_tx_ts;
    uint32_t resp_rx_ts;
    uint32_t poll_rx_ts;
    uint32_t resp_tx_ts;
    resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
    resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);
    poll_tx_ts = dwt_readtxtimestamplo32();
    resp_rx_ts = dwt_readrxtimestamplo32();

    const float clock_offset_ratio =
        ((float)dwt_readclockoffset()) / (uint32_t)(1UL << 26);
    const int32_t round_trip_init = resp_rx_ts - poll_tx_ts;
    const int32_t round_trip_resp = resp_tx_ts - poll_rx_ts;
    const double tof =
        ((round_trip_init - round_trip_resp * (1.0 - clock_offset_ratio)) / 2.0) *
        DWT_TIME_UNITS;
    const double distance_m = tof * SPEED_OF_LIGHT;

    const uint8_t packet_type = rx_buffer[APP_HEADER_IDX + UWB_APP_TYPE_OFFSET];
    const uint8_t packet_sequence = rx_buffer[APP_HEADER_IDX + UWB_APP_SEQUENCE_OFFSET];
    const uint16_t source_id =
        uwb_packet_read_u16_le(&rx_buffer[APP_HEADER_IDX + UWB_APP_SOURCE_ID_OFFSET]);
    const uint8_t flags = rx_buffer[APP_HEADER_IDX + UWB_APP_FLAGS_OFFSET];
    const uint32_t timestamp_ms =
        uwb_packet_read_u32_le(&rx_buffer[APP_HEADER_IDX + UWB_APP_TIMESTAMP_MS_OFFSET]);

    Serial.print("{\"protocol\":\"uwb_serial_v1\",\"version\":");
    Serial.print(UWB_APP_VERSION);
    Serial.print(",\"type\":");
    Serial.print(packet_type);
    Serial.print(",\"sequence\":");
    Serial.print(packet_sequence);
    Serial.print(",\"source_id\":");
    Serial.print(source_id);
    Serial.print(",\"flags\":");
    Serial.print(flags);
    Serial.print(",\"timestamp_ms\":");
    Serial.print(timestamp_ms);
    Serial.print(",\"range_m\":");
    Serial.print(distance_m, 3);
    Serial.print(",\"payload_hex\":\"");
    for (uint8_t i = 0; i < payload_len; i++)
    {
        if (rx_buffer[APP_PAYLOAD_IDX + i] < 0x10)
        {
            Serial.print('0');
        }
        Serial.print(rx_buffer[APP_PAYLOAD_IDX + i], HEX);
    }
    Serial.println("\"}");

    delay(500);
}
