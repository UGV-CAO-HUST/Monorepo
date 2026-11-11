#include <Arduino.h>
#include "driver.hpp"
#include "config.hpp"

// SCK, MISO, MOSI, CS, RESET
DW3000Driver uwb({18, 19, 23, 21, 27});

bool radioReady = false;

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\nUWB Anchor Starting");

    // Load the saved anchor ID and coordinates.
    if (!loadConfig()) {
        Serial.println("Configuration storage unavailable.");
        return;
    }

    printConfigHelp();

    // Initialize the DW3000 hardware.
    if (!uwb.begin()) {
        Serial.println("UWB hardware initialization failed.");
        return;
    }

    // Verify that the device responds.
    uint32_t deviceId = 0;

    if (!uwb.readDeviceId(deviceId)) {
        Serial.println("Failed to read UWB device ID.");
        return;
    }

    Serial.printf(
        "DW3000 device ID: 0x%08lX\n",
        static_cast<unsigned long>(deviceId)
    );

    if (!DW3000Driver::looksLikeDw3000DeviceId(deviceId)) {
        Serial.println("Unexpected device ID.");
        return;
    }

    radioReady = true;

    Serial.println("UWB hardware check passed.");

    if (!isConfigured()) {
        Serial.println(
            "Configure this anchor using the Serial Monitor."
        );
    }
}

void loop() {
    // Accept configuration commands at any time.
    processSerialCommands();

    if (!radioReady || !isConfigured()) {
        return;
    }

    // Later:
    // 1. Configure the UWB radio.
    // 2. Receive ranging requests.
    // 3. Respond to other nodes.
    // 4. Transmit measurements when appropriate.
}