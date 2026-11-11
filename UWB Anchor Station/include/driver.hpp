#pragma once

#include <Arduino.h>
#include <SPI.h>

class DW3000Driver {
public:
    struct Pins {
        int sck;
        int miso;
        int mosi;
        int cs;
        int reset;  // Use -1 if reset is not connected.
    };

    explicit DW3000Driver(
        const Pins& pins,
        uint32_t spiClockHz = 2000000
    );

    // Initialize SPI and reset the UWB chip if reset is connected.
    bool begin();

    // Reset the chip by pulling RESET low and then releasing it.
    bool hardwareReset();

    // Read bytes from a DW3000 register.
    bool readRegister(
        uint8_t registerId,
        uint16_t subAddress,
        uint8_t* data,
        size_t length
    );

    // Write bytes to a DW3000 register.
    bool writeRegister(
        uint8_t registerId,
        uint16_t subAddress,
        const uint8_t* data,
        size_t length
    );

    // Read the 32-bit device ID register.
    bool readDeviceId(uint32_t& deviceId);

    // Preliminary device-ID prefix check, not definitive identification.
    static bool looksLikeDw3000DeviceId(uint32_t deviceId);

private:
    Pins _pins;
    uint32_t _spiClockHz;
    bool _started;

    size_t makeHeader(
        bool isWrite,
        uint8_t registerId,
        uint16_t subAddress,
        uint8_t header[2]
    ) const;

    void select();
    void deselect();
};