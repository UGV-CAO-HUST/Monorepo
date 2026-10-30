#include "driver.hpp"

DW3000Driver::DW3000Driver(const Pins& pins, uint32_t spiClockHz)
    : _pins(pins),
      _spiClockHz(spiClockHz),
      _started(false) {}

bool DW3000Driver::begin() {
    if (_pins.cs < 0 ||
        _pins.sck < 0 ||
        _pins.miso < 0 ||
        _pins.mosi < 0) {
        return false;
    }

    // CS is active-low.
    pinMode(_pins.cs, OUTPUT);
    digitalWrite(_pins.cs, HIGH);

    if (_pins.reset >= 0) {
        pinMode(_pins.reset, INPUT);
    }

    SPI.begin(
        _pins.sck,
        _pins.miso,
        _pins.mosi,
        _pins.cs
    );

    _started = true;

    if (_pins.reset >= 0) {
        return hardwareReset();
    }

    return true;
}

bool DW3000Driver::hardwareReset() {
    if (_pins.reset < 0) {
        return false;
    }

    // Pull reset low.
    digitalWrite(_pins.reset, LOW);
    pinMode(_pins.reset, OUTPUT);
    delay(10);

    // Release reset. Verify this behavior against your board's
    // reset circuitry before relying on it.
    pinMode(_pins.reset, INPUT);
    delay(10);

    return true;
}

size_t DW3000Driver::makeHeader(
    bool isWrite,
    uint8_t registerId,
    uint16_t subAddress,
    uint8_t header[2]
) const {
    if (registerId > 0x1F || subAddress > 0x7F) {
        return 0;
    }

    const bool fullAddress = (subAddress != 0);

    /*
     * DW3000 SPI header layout:
     *
     * Bit 7:    write flag
     * Bit 6:    full-address flag
     * Bits 5-1: register ID
     * Bit 0:    subaddress bit 6, for full addressing
     */

    header[0] =
        (isWrite ? 0x80 : 0x00) |
        (fullAddress ? 0x40 : 0x00) |
        ((registerId & 0x1F) << 1) |
        (fullAddress ? ((subAddress >> 6) & 0x01) : 0x00);

    if (!fullAddress) {
        return 1;
    }

    // Subaddress bits 5-0 occupy bits 7-2 of byte 2.
    header[1] =
        static_cast<uint8_t>((subAddress & 0x3F) << 2);

    return 2;
}

void DW3000Driver::select() {
    digitalWrite(_pins.cs, LOW);
}

void DW3000Driver::deselect() {
    digitalWrite(_pins.cs, HIGH);
}

bool DW3000Driver::readRegister(
    uint8_t registerId,
    uint16_t subAddress,
    uint8_t* data,
    size_t length
) {
    if (!_started || data == nullptr || length == 0) {
        return false;
    }

    uint8_t header[2] = {0, 0};

    const size_t headerLength = makeHeader(
        false,
        registerId,
        subAddress,
        header
    );

    if (headerLength == 0) {
        return false;
    }

    SPI.beginTransaction(
        SPISettings(_spiClockHz, MSBFIRST, SPI_MODE0)
    );

    select();

    for (size_t i = 0; i < headerLength; i++) {
        SPI.transfer(header[i]);
    }

    // Clock dummy bytes to receive register data.
    for (size_t i = 0; i < length; i++) {
        data[i] = SPI.transfer(0x00);
    }

    deselect();
    SPI.endTransaction();

    return true;
}

bool DW3000Driver::writeRegister(
    uint8_t registerId,
    uint16_t subAddress,
    const uint8_t* data,
    size_t length
) {
    if (!_started || data == nullptr || length == 0) {
        return false;
    }

    uint8_t header[2] = {0, 0};

    const size_t headerLength = makeHeader(
        true,
        registerId,
        subAddress,
        header
    );

    if (headerLength == 0) {
        return false;
    }

    SPI.beginTransaction(
        SPISettings(_spiClockHz, MSBFIRST, SPI_MODE0)
    );

    select();

    for (size_t i = 0; i < headerLength; i++) {
        SPI.transfer(header[i]);
    }

    for (size_t i = 0; i < length; i++) {
        SPI.transfer(data[i]);
    }

    deselect();
    SPI.endTransaction();

    return true;
}

bool DW3000Driver::readDeviceId(uint32_t& deviceId) {
    uint8_t bytes[4] = {0, 0, 0, 0};

    // DEV_ID is assumed to be register file 0x00, subaddress 0x00.
    if (!readRegister(0x00, 0x00, bytes, sizeof(bytes))) {
        return false;
    }

    // Assemble four bytes in little-endian order.
    deviceId =
        static_cast<uint32_t>(bytes[0]) |
        (static_cast<uint32_t>(bytes[1]) << 8) |
        (static_cast<uint32_t>(bytes[2]) << 16) |
        (static_cast<uint32_t>(bytes[3]) << 24);

    return true;
}

bool DW3000Driver::looksLikeDw3000DeviceId(uint32_t deviceId) {
    // Preliminary sanity check only.
    return (deviceId & 0xFFFFFF00UL) == 0xDECA0300UL;
}