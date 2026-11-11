#pragma once

#include <Arduino.h>

// Configuration for one fixed UWB anchor.
struct AnchorConfig {
    uint16_t id = 0;

    // Anchor position in meters, relative to the chosen origin.
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    bool idConfigured = false;
    bool positionConfigured = false;
};

// Load the anchor configuration from ESP32 NVS flash.
// Returns false if NVS cannot be accessed.
bool loadConfig();

// Save the current configuration to ESP32 NVS flash.
bool saveConfig();

// Process commands received through the Serial Monitor.
// Call this repeatedly from loop().
void processSerialCommands();

// Access the current in-memory configuration.
const AnchorConfig& getConfig();

// True only when both the anchor ID and position are configured.
bool isConfigured();

// Print the current configuration and its status.
void printConfig();

// Print available Serial Monitor commands.
void printConfigHelp();