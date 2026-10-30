# ESP32 NodeMCU-32S ↔ DWS3000 UWB Shield Wiring

> **Board identification:** This table assumes the Qorvo DWS3000 / DWM3000EVB Arduino-form-factor shield and the NodeMCU 3S AI thinker ESP32 WROOM Dev board.

## Wiring table

| DWS3000 shield signal / pin label | ESP32 NodeMCU-32S pin (fill in) | Notes |
|---|---|---|
| `3V3` |  | Power input for the shield's 3.3 V rail, if your board's power-selection/jumper configuration calls for it. Verify the board schematic first. |
| `GND` |  | Connect to ESP32 ground. Both boards need a common ground. |
| `SPI CLK` / `SPICLK` |  | SPI clock (SCK). |
| `SPI MISO` / `SPIMISO` |  | SPI data from shield to ESP32 (MISO). |
| `SPI MOSI` / `SPIMOSI` |  | SPI data from ESP32 to shield (MOSI). |
| `SPI CSn` / `SPICSn` |  | Chip select, active low; choose a suitable GPIO. |
| `IRQ` |  | Interrupt output from the UWB shield to the ESP32; choose a suitable input-capable GPIO. |
| `RSTn` |  | Active-low reset. Confirm whether your software will control reset or whether the board handles it. |

## Before powering on

- **Leave all ESP32 pin entries blank until you choose the GPIOs.**
- The SPI bus signals, chip select, IRQ, and any reset/wake-up signals must match the pin assignments in your firmware.
- The DWS3000/DWM3000EVB uses 3.3 V logic. Do not apply 5 V logic signals to ESP32 GPIOs.
- Confirm the shield's jumper/power-selection configuration and its exact revision using the board schematic or silkscreen. The Arduino shield form factor does not guarantee every carrier board is wired identically.
- For basic communication, start with **GND, SPI CLK, SPI MISO, SPI MOSI, SPI CSn, and IRQ**; add reset/wake-up connections according to the driver/library requirements.

## Reference

- Qorvo DWM3000EVB product page (formerly known as DWS3000): https://www.qorvo.com/products/ek/DWM3000EVB

